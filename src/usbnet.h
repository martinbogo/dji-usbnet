/*
 * usbnet.h - Interfaces between the USB/RNDIS layer, the utun layer, and the
 * L2<->L3 bridge. Kept deliberately small so each layer can be developed and
 * tested on its own.
 */
#ifndef DJI_USBNET_USBNET_H
#define DJI_USBNET_USBNET_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* DJI drone RNDIS gadget. Confirmed by probe against a live device: it
 * enumerates as VID 0x2ca3 / PID 0x001f, "DJI"/"DJI", a 6-interface composite
 * whose RNDIS function is control iface 0 (0xE0/01/03) + data iface 1 (0x0A,
 * bulk IN 0x81 / OUT 0x08). Other DJI models may differ; override via argv. */
#define DJI_USB_VID 0x2ca3
#define DJI_USB_PID 0x001f

/* Addressing. The drone side is fixed by DJI's firmware. The host side and the
 * netmask are our choice; whether the drone expects us to DHCP or accepts a
 * static host address is the main thing to confirm on real hardware. */
#define DRONE_IP   "192.168.42.2"
#define HOST_IP    "192.168.42.1"
#define NET_MASK   "255.255.255.0"

#define ETHER_ADDR_LEN 6
#define ETHER_HDR_LEN  14
#define ETHER_MTU      1500
#define ETHER_FRAME_MAX (ETHER_HDR_LEN + ETHER_MTU)

/* Opaque handles. */
typedef struct usb_ctx  usb_ctx;
typedef struct utun_ctx utun_ctx;

/* ------------------------------------------------------------------ USB/RNDIS */

/* Open the DJI device, claim the RNDIS data interface, and run the RNDIS
 * control handshake (INIT, SET packet filter, query MAC). Returns NULL on
 * failure. On success *out_drone_mac is filled with the device's Ethernet
 * address (from OID_802_3_CURRENT_ADDRESS). */
usb_ctx *usb_open(uint16_t vid, uint16_t pid, uint8_t out_drone_mac[ETHER_ADDR_LEN]);

/* Send one Ethernet frame to the device, wrapped in an RNDIS_PACKET_MSG. */
int usb_send_frame(usb_ctx *u, const uint8_t *frame, size_t len);

/* Blocking receive of one Ethernet frame from the device (unwraps
 * RNDIS_PACKET_MSG). Returns the frame length, 0 on timeout, <0 on error. */
int usb_recv_frame(usb_ctx *u, uint8_t *frame, size_t cap);

/* Send an RNDIS keepalive; call on a timer (~5s) to keep the link up. */
int usb_keepalive(usb_ctx *u);

void usb_close(usb_ctx *u);

/* --------------------------------------------------------------------- utun */

/* Create a utun interface and configure host IP / netmask + a route to the
 * drone. Returns NULL on failure. *out_name receives e.g. "utun11". */
utun_ctx *utun_open(const char *host_ip, const char *drone_ip,
                    const char *netmask, char out_name[16]);

/* Read one IP packet (no Ethernet header) from utun. macOS prepends a 4-byte
 * address-family header which this strips. Returns IP payload length. */
int utun_read_ip(utun_ctx *t, uint8_t *ip, size_t cap);

/* Write one IP packet (no Ethernet header) to utun, adding the AF header. */
int utun_write_ip(utun_ctx *t, const uint8_t *ip, size_t len);

int utun_fd(utun_ctx *t);
void utun_close(utun_ctx *t);

/* ---------------------------------------------------------------- L2/L3 shim */

/* The bridge terminates Ethernet + ARP on the USB side and speaks pure IP on
 * the utun side. It needs the drone's MAC (learned during usb_open) and a
 * synthetic host MAC (we pick one) to build/answer frames. */
typedef struct {
    uint8_t drone_mac[ETHER_ADDR_LEN];
    uint8_t host_mac[ETHER_ADDR_LEN];
    uint32_t host_ip_be;   /* network byte order */
    uint32_t drone_ip_be;  /* network byte order */
    bool mac_known;        /* true once we have the drone's real MAC */
} bridge_ctx;

void bridge_init(bridge_ctx *b, const uint8_t drone_mac[ETHER_ADDR_LEN],
                 const char *host_ip, const char *drone_ip);

/* utun IP packet -> Ethernet frame for the USB side. Writes into out (cap must
 * be >= ETHER_FRAME_MAX). Returns frame length, or 0 to drop. */
int bridge_ip_to_eth(bridge_ctx *b, const uint8_t *ip, size_t iplen,
                     uint8_t *out, size_t cap);

/* Ethernet frame from the USB side -> action. If it is an ARP request for the
 * host IP, *reply_len is set and out holds an Ethernet ARP reply to send back
 * over USB, and the function returns BRIDGE_ARP_REPLY. If it is IP for us, the
 * IP payload is copied to out (return BRIDGE_IP, reply_len = 0). Otherwise
 * BRIDGE_DROP. */
enum { BRIDGE_DROP = 0, BRIDGE_IP, BRIDGE_ARP_REPLY };
int bridge_eth_from_drone(bridge_ctx *b, const uint8_t *eth, size_t ethlen,
                          uint8_t *out, size_t cap, size_t *reply_len);

#endif /* DJI_USBNET_USBNET_H */
