/*
 * bridge.c - Terminates Ethernet + ARP on the USB/RNDIS side so the utun side
 * only ever sees IP.
 *
 * RNDIS carries Ethernet (layer 2) frames; utun is point-to-point IP (layer 3).
 * The drone will ARP for its peer (our host IP) before sending IP, so we must
 * answer ARP ourselves using a synthetic host MAC, and add/strip the 14-byte
 * Ethernet header on the way through.
 */
#include "usbnet.h"

#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define ETHERTYPE_IP  0x0800
#define ETHERTYPE_ARP 0x0806
#define ARP_REQUEST   1
#define ARP_REPLY     2

#pragma pack(push, 1)
typedef struct {
    uint8_t  dst[ETHER_ADDR_LEN];
    uint8_t  src[ETHER_ADDR_LEN];
    uint16_t ethertype;   /* big-endian on the wire */
} eth_hdr;

typedef struct {
    uint16_t htype;
    uint16_t ptype;
    uint8_t  hlen;
    uint8_t  plen;
    uint16_t oper;
    uint8_t  sha[ETHER_ADDR_LEN];
    uint8_t  spa[4];
    uint8_t  tha[ETHER_ADDR_LEN];
    uint8_t  tpa[4];
} arp_pkt;
#pragma pack(pop)

static bool mac_is_zero(const uint8_t m[ETHER_ADDR_LEN]) {
    for (int i = 0; i < ETHER_ADDR_LEN; i++) if (m[i]) return false;
    return true;
}

void bridge_init(bridge_ctx *b, const uint8_t drone_mac[ETHER_ADDR_LEN],
                 const char *host_ip, const char *drone_ip) {
    memset(b, 0, sizeof(*b));
    memcpy(b->drone_mac, drone_mac, ETHER_ADDR_LEN);
    b->mac_known = !mac_is_zero(drone_mac);
    /* Locally administered (0x02 low bit set), unicast synthetic host MAC. */
    static const uint8_t host_mac[ETHER_ADDR_LEN] = {0x02, 0x1a, 0x00, 0x00, 0x00, 0x01};
    memcpy(b->host_mac, host_mac, ETHER_ADDR_LEN);
    b->host_ip_be  = inet_addr(host_ip);
    b->drone_ip_be = inet_addr(drone_ip);
}

int bridge_ip_to_eth(bridge_ctx *b, const uint8_t *ip, size_t iplen,
                     uint8_t *out, size_t cap) {
    if (iplen == 0 || iplen > ETHER_MTU || cap < ETHER_HDR_LEN + iplen)
        return 0;
    eth_hdr *e = (eth_hdr *)out;
    if (b->mac_known)
        memcpy(e->dst, b->drone_mac, ETHER_ADDR_LEN);
    else
        memset(e->dst, 0xff, ETHER_ADDR_LEN); /* broadcast until MAC learned */
    memcpy(e->src, b->host_mac, ETHER_ADDR_LEN);
    e->ethertype = htons(ETHERTYPE_IP);
    memcpy(out + ETHER_HDR_LEN, ip, iplen);
    return (int)(ETHER_HDR_LEN + iplen);
}

int bridge_eth_from_drone(bridge_ctx *b, const uint8_t *eth, size_t ethlen,
                          uint8_t *out, size_t cap, size_t *reply_len) {
    *reply_len = 0;
    if (ethlen < ETHER_HDR_LEN) return BRIDGE_DROP;
    const eth_hdr *e = (const eth_hdr *)eth;
    uint16_t etype = ntohs(e->ethertype);

    /* Learn the drone's real MAC from its source address the first time we
     * hear from it; the RNDIS OID query is not always reliable. */
    if (!b->mac_known && !mac_is_zero(e->src)) {
        memcpy(b->drone_mac, e->src, ETHER_ADDR_LEN);
        b->mac_known = true;
    }

    if (etype == ETHERTYPE_ARP) {
        if (ethlen < ETHER_HDR_LEN + sizeof(arp_pkt)) return BRIDGE_DROP;
        const arp_pkt *a = (const arp_pkt *)(eth + ETHER_HDR_LEN);
        if (ntohs(a->oper) != ARP_REQUEST) return BRIDGE_DROP;
        /* Only answer requests for our host IP. */
        if (memcmp(a->tpa, &b->host_ip_be, 4) != 0) return BRIDGE_DROP;
        if (cap < ETHER_HDR_LEN + sizeof(arp_pkt)) return BRIDGE_DROP;

        eth_hdr *re = (eth_hdr *)out;
        memcpy(re->dst, e->src, ETHER_ADDR_LEN);
        memcpy(re->src, b->host_mac, ETHER_ADDR_LEN);
        re->ethertype = htons(ETHERTYPE_ARP);

        arp_pkt *ra = (arp_pkt *)(out + ETHER_HDR_LEN);
        ra->htype = htons(1);
        ra->ptype = htons(ETHERTYPE_IP);
        ra->hlen = ETHER_ADDR_LEN;
        ra->plen = 4;
        ra->oper = htons(ARP_REPLY);
        memcpy(ra->sha, b->host_mac, ETHER_ADDR_LEN);
        memcpy(ra->spa, &b->host_ip_be, 4);
        memcpy(ra->tha, a->sha, ETHER_ADDR_LEN);
        memcpy(ra->tpa, a->spa, 4);

        *reply_len = ETHER_HDR_LEN + sizeof(arp_pkt);
        return BRIDGE_ARP_REPLY;
    }

    if (etype == ETHERTYPE_IP) {
        size_t iplen = ethlen - ETHER_HDR_LEN;
        if (iplen > cap) return BRIDGE_DROP;
        memcpy(out, eth + ETHER_HDR_LEN, iplen);
        *reply_len = iplen;   /* caller writes this many bytes to utun */
        return BRIDGE_IP;
    }

    return BRIDGE_DROP;
}
