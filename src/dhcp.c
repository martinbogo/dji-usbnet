/*
 * dhcp.c - A minimal DHCP client that runs over the RNDIS link.
 *
 * DJI aircraft that use the RNDIS/IP path typically run a DHCP server at
 * 192.168.42.2 and expect the host to lease its address rather than assume one.
 * Because DHCP is broadcast layer-2 traffic and our utun is layer-3, the bridge
 * itself must be the DHCP client: it builds Ethernet/IP/UDP/BOOTP frames and
 * pushes them over RNDIS via usb_send_frame, then reads replies via
 * usb_recv_frame.
 *
 * This implements just DISCOVER -> OFFER -> REQUEST -> ACK with the handful of
 * options that matter (message type, requested IP, server id, subnet, router).
 * On success the caller configures the utun with the leased address; on timeout
 * it falls back to a static address. A successful lease also proves the drone's
 * RNDIS IP stack is alive.
 */
#include "usbnet.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define ETHERTYPE_IP 0x0800
#define IPPROTO_UDP_ 17
#define DHCP_MAGIC 0x63825363u

#define BOOTREQUEST 1
#define DHCP_DISCOVER 1
#define DHCP_OFFER    2
#define DHCP_REQUEST  3
#define DHCP_ACK      5

#define OPT_SUBNET       1
#define OPT_ROUTER       3
#define OPT_REQUESTED_IP 50
#define OPT_MSG_TYPE     53
#define OPT_SERVER_ID    54
#define OPT_PARAM_LIST   55
#define OPT_END          255

/* A fixed transaction id. A single short-lived client does not need a random
 * xid, and this file avoids Math.random-style calls by design. */
#define DHCP_XID 0x0D301D15u

#pragma pack(push, 1)
typedef struct { uint8_t dst[6], src[6]; uint16_t ethertype; } eth_hdr;
typedef struct {
    uint8_t  vhl, tos;
    uint16_t len, id, frag;
    uint8_t  ttl, proto;
    uint16_t csum;
    uint32_t src, dst;
} ip_hdr;
typedef struct { uint16_t sport, dport, len, csum; } udp_hdr;
typedef struct {
    uint8_t  op, htype, hlen, hops;
    uint32_t xid;
    uint16_t secs, flags;
    uint32_t ciaddr, yiaddr, siaddr, giaddr;
    uint8_t  chaddr[16];
    uint8_t  sname[64];
    uint8_t  file[128];
    uint32_t magic;
    uint8_t  options[312];
} bootp;
#pragma pack(pop)

static uint16_t csum16(const void *data, size_t len) {
    const uint8_t *p = data;
    uint32_t sum = 0;
    while (len > 1) { sum += (uint16_t)((p[0] << 8) | p[1]); p += 2; len -= 2; }
    if (len) sum += (uint16_t)(p[0] << 8);
    while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
    return (uint16_t)~sum;
}

/* Build a full Ethernet+IP+UDP+BOOTP DHCP frame into buf. Returns total length.
 * opts/optlen are the DHCP options (after the magic cookie, before END, which
 * is appended here). */
static size_t build_dhcp(uint8_t *buf, const uint8_t host_mac[6],
                         const uint8_t *opts, size_t optlen) {
    memset(buf, 0, sizeof(eth_hdr) + sizeof(ip_hdr) + sizeof(udp_hdr) + sizeof(bootp));
    eth_hdr *e = (eth_hdr *)buf;
    ip_hdr  *ip = (ip_hdr *)(buf + sizeof(eth_hdr));
    udp_hdr *ud = (udp_hdr *)((uint8_t *)ip + sizeof(ip_hdr));
    bootp   *bp = (bootp *)((uint8_t *)ud + sizeof(udp_hdr));

    memset(e->dst, 0xff, 6);                 /* broadcast */
    memcpy(e->src, host_mac, 6);
    e->ethertype = htons(ETHERTYPE_IP);

    bp->op = BOOTREQUEST; bp->htype = 1; bp->hlen = 6;
    bp->xid = htonl(DHCP_XID);
    bp->flags = htons(0x8000);               /* ask server to broadcast reply */
    memcpy(bp->chaddr, host_mac, 6);
    bp->magic = htonl(DHCP_MAGIC);
    size_t ol = optlen;
    memcpy(bp->options, opts, optlen);
    bp->options[ol++] = OPT_END;

    size_t bootp_len = (size_t)((uint8_t *)bp->options - (uint8_t *)bp) + ol;
    size_t udp_len = sizeof(udp_hdr) + bootp_len;
    ud->sport = htons(68); ud->dport = htons(67);
    ud->len = htons((uint16_t)udp_len);
    ud->csum = 0;                            /* optional in IPv4, leave 0 */

    size_t ip_len = sizeof(ip_hdr) + udp_len;
    ip->vhl = 0x45; ip->tos = 0;
    ip->len = htons((uint16_t)ip_len);
    ip->id = 0; ip->frag = 0; ip->ttl = 64; ip->proto = IPPROTO_UDP_;
    ip->src = 0x00000000; ip->dst = 0xffffffff;
    ip->csum = 0;
    ip->csum = htons(csum16(ip, sizeof(ip_hdr)));

    return sizeof(eth_hdr) + ip_len;
}

/* Find DHCP option `code` in a received bootp; returns pointer to its value and
 * sets *vlen, or NULL. */
static const uint8_t *find_opt(const bootp *bp, size_t opt_space, uint8_t code, uint8_t *vlen) {
    const uint8_t *o = bp->options;
    const uint8_t *end = bp->options + opt_space;
    while (o < end) {
        uint8_t c = *o++;
        if (c == OPT_END) break;
        if (c == 0) continue;               /* pad */
        if (o >= end) break;
        uint8_t l = *o++;
        if (o + l > end) break;
        if (c == code) { *vlen = l; return o; }
        o += l;
    }
    return NULL;
}

/* Parse an incoming Ethernet frame as a DHCP reply of the given message type
 * (OFFER/ACK) matching our xid. Fills *out on success. Returns 1 on match. */
static int parse_reply(const uint8_t *frame, int len, uint8_t want_type,
                       dhcp_lease *out, uint32_t *server_be_io) {
    if (len < (int)(sizeof(eth_hdr) + sizeof(ip_hdr) + sizeof(udp_hdr) + 240)) return 0;
    const eth_hdr *e = (const eth_hdr *)frame;
    if (ntohs(e->ethertype) != ETHERTYPE_IP) return 0;
    const ip_hdr *ip = (const ip_hdr *)(frame + sizeof(eth_hdr));
    if ((ip->vhl >> 4) != 4 || ip->proto != IPPROTO_UDP_) return 0;
    int ihl = (ip->vhl & 0x0f) * 4;
    const udp_hdr *ud = (const udp_hdr *)((const uint8_t *)ip + ihl);
    if (ntohs(ud->dport) != 68) return 0;
    const bootp *bp = (const bootp *)((const uint8_t *)ud + sizeof(udp_hdr));
    if (ntohl(bp->xid) != DHCP_XID || ntohl(bp->magic) != DHCP_MAGIC) return 0;

    size_t opt_space = (size_t)(frame + len - bp->options);
    uint8_t vlen = 0;
    const uint8_t *t = find_opt(bp, opt_space, OPT_MSG_TYPE, &vlen);
    if (!t || vlen < 1 || *t != want_type) return 0;

    struct in_addr a;
    a.s_addr = bp->yiaddr;
    snprintf(out->ip, sizeof(out->ip), "%s", inet_ntoa(a));

    const uint8_t *sm = find_opt(bp, opt_space, OPT_SUBNET, &vlen);
    if (sm && vlen == 4) { memcpy(&a.s_addr, sm, 4); snprintf(out->netmask, sizeof(out->netmask), "%s", inet_ntoa(a)); }
    else snprintf(out->netmask, sizeof(out->netmask), "255.255.255.0");

    const uint8_t *rt = find_opt(bp, opt_space, OPT_ROUTER, &vlen);
    if (rt && vlen >= 4) { memcpy(&a.s_addr, rt, 4); snprintf(out->router, sizeof(out->router), "%s", inet_ntoa(a)); }
    else out->router[0] = '\0';

    const uint8_t *sv = find_opt(bp, opt_space, OPT_SERVER_ID, &vlen);
    if (sv && vlen == 4) {
        memcpy(&a.s_addr, sv, 4);
        snprintf(out->server, sizeof(out->server), "%s", inet_ntoa(a));
        if (server_be_io) memcpy(server_be_io, sv, 4);
    }
    return 1;
}

/* Read frames for up to timeout_ms looking for a DHCP reply of want_type. */
static int wait_reply(usb_ctx *usb, uint8_t want_type, dhcp_lease *out,
                      uint32_t *server_be_io, int timeout_ms) {
    uint8_t frame[ETHER_FRAME_MAX + 64];
    time_t start = time(NULL);
    while ((time(NULL) - start) * 1000 <= timeout_ms) {
        int n = usb_recv_frame(usb, frame, sizeof(frame));
        if (n < 0) return -1;               /* device error */
        if (n == 0) continue;               /* bulk timeout; keep waiting */
        if (parse_reply(frame, n, want_type, out, server_be_io)) return 1;
    }
    return 0;
}

int dhcp_acquire(usb_ctx *usb, const uint8_t host_mac[ETHER_ADDR_LEN], dhcp_lease *out) {
    uint8_t buf[sizeof(eth_hdr) + sizeof(ip_hdr) + sizeof(udp_hdr) + sizeof(bootp)];
    memset(out, 0, sizeof(*out));

    for (int attempt = 0; attempt < 3; attempt++) {
        /* DISCOVER */
        uint8_t opts[] = {
            OPT_MSG_TYPE, 1, DHCP_DISCOVER,
            OPT_PARAM_LIST, 2, OPT_SUBNET, OPT_ROUTER,
        };
        size_t flen = build_dhcp(buf, host_mac, opts, sizeof(opts));
        if (usb_send_frame(usb, buf, flen) < 0) return -1;

        uint32_t server_be = 0;
        int r = wait_reply(usb, DHCP_OFFER, out, &server_be, 2000);
        if (r < 0) return -1;
        if (r == 0) { fprintf(stderr, "[dhcp] no OFFER (attempt %d)\n", attempt + 1); continue; }
        fprintf(stderr, "[dhcp] OFFER %s (mask %s, router %s, server %s)\n",
                out->ip, out->netmask, out->router[0] ? out->router : "-",
                out->server[0] ? out->server : "-");

        /* REQUEST the offered address. */
        uint32_t req_ip = inet_addr(out->ip);
        uint8_t ropts[] = {
            OPT_MSG_TYPE, 1, DHCP_REQUEST,
            OPT_REQUESTED_IP, 4, 0,0,0,0,
            OPT_SERVER_ID, 4, 0,0,0,0,
            OPT_PARAM_LIST, 2, OPT_SUBNET, OPT_ROUTER,
        };
        memcpy(&ropts[5], &req_ip, 4);
        memcpy(&ropts[11], &server_be, 4);
        flen = build_dhcp(buf, host_mac, ropts, sizeof(ropts));
        if (usb_send_frame(usb, buf, flen) < 0) return -1;

        r = wait_reply(usb, DHCP_ACK, out, NULL, 2000);
        if (r < 0) return -1;
        if (r == 1) {
            fprintf(stderr, "[dhcp] ACK: host %s/%s router %s\n",
                    out->ip, out->netmask, out->router[0] ? out->router : "-");
            return 0;
        }
        fprintf(stderr, "[dhcp] no ACK (attempt %d)\n", attempt + 1);
    }
    return -1;
}
