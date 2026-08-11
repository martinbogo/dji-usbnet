/*
 * usb.c - libusb-backed RNDIS host.
 *
 * Opens the DJI RNDIS gadget in userspace (no kext), runs the RNDIS control
 * handshake, learns the device MAC, and moves Ethernet frames over the bulk
 * endpoints wrapped in RNDIS_PACKET_MSG.
 *
 * Confirmed against a DJI Mavic Pro (2ca3:001f): the RNDIS function is a
 * CDC/union pair - comm interface 0 (class 0xE0, RNDIS-over-Ethernet, with an
 * interrupt notify endpoint) plus data interface 1 (class 0x0A) carrying the
 * two bulk endpoints. detect_endpoints() finds this generically. The control
 * channel is CDC-encapsulated (SEND/GET_ENCAPSULATED_* control transfers), as
 * in HoRNDIS / Linux rndis_host.
 */
#include "usbnet.h"
#include "rndis.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb.h>

#define CTRL_TIMEOUT_MS 1000
#define BULK_TIMEOUT_MS 1000
#define CTRL_BUF_MAX    1025

struct usb_ctx {
    libusb_context       *ctx;
    libusb_device_handle *dev;
    int      iface_comm;   /* CDC communications interface (control channel) */
    int      iface_data;   /* CDC data interface (bulk endpoints) */
    uint8_t  ep_in;        /* bulk IN  (device -> host) */
    uint8_t  ep_out;       /* bulk OUT (host -> device) */
    uint32_t request_id;
    uint8_t  drone_mac[ETHER_ADDR_LEN];
};

/* Class-specific SEND_ENCAPSULATED_COMMAND / GET_ENCAPSULATED_RESPONSE over
 * the control endpoint, addressed to the comm interface. */
static int ctrl_send(usb_ctx *u, const void *buf, int len) {
    int r = libusb_control_transfer(
        u->dev,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        USB_CDC_SEND_ENCAPSULATED_COMMAND, 0, u->iface_comm,
        (unsigned char *)buf, (uint16_t)len, CTRL_TIMEOUT_MS);
    return r < 0 ? r : 0;
}

static int ctrl_recv(usb_ctx *u, void *buf, int cap) {
    int r = libusb_control_transfer(
        u->dev,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        USB_CDC_GET_ENCAPSULATED_RESPONSE, 0, u->iface_comm,
        (unsigned char *)buf, (uint16_t)cap, CTRL_TIMEOUT_MS);
    return r;  /* bytes read, or <0 */
}

/*
 * RNDIS InformationBuffer offsets are all measured from the start of the
 * RequestId field, i.e. 8 bytes into the message (past msg_type + msg_len).
 */
#define RNDIS_INFOBUF_BASE 8

/*
 * Send a command, then read its response. The device signals response
 * readiness on the interrupt endpoint; rather than depend on that timing we
 * poll GET_ENCAPSULATED_RESPONSE with a short backoff until a well-formed
 * reply (at least a common header) arrives. Returns bytes read, or <0.
 */
static int rndis_transact(usb_ctx *u, const void *cmd, int cmdlen,
                          uint8_t *resp, int respcap) {
    if (ctrl_send(u, cmd, cmdlen) < 0) return -1;
    for (int tries = 0; tries < 25; tries++) {
        int n = ctrl_recv(u, resp, respcap);
        if (n >= (int)sizeof(rndis_hdr)) return n;
        usleep(2000);  /* 2 ms */
    }
    return -1;
}

static int rndis_init(usb_ctx *u) {
    rndis_init_msg m;
    memset(&m, 0, sizeof(m));
    m.msg_type = RNDIS_MSG_INIT;
    m.msg_len = sizeof(m);
    m.request_id = ++u->request_id;
    m.major_version = RNDIS_MAJOR_VERSION;
    m.minor_version = RNDIS_MINOR_VERSION;
    m.max_transfer_size = 16384;

    uint8_t resp[CTRL_BUF_MAX];
    int n = rndis_transact(u, &m, sizeof(m), resp, sizeof(resp));
    if (n < (int)sizeof(rndis_init_cmplt)) {
        fprintf(stderr, "[usb] INIT_CMPLT short (%d)\n", n);
        return -1;
    }
    rndis_init_cmplt *c = (rndis_init_cmplt *)resp;
    if (c->msg_type != RNDIS_MSG_INIT_CMPLT || c->status != RNDIS_STATUS_SUCCESS) {
        fprintf(stderr, "[usb] INIT failed: type=0x%x status=0x%x\n",
                c->msg_type, c->status);
        return -1;
    }
    fprintf(stderr, "[usb] RNDIS init ok (dev max_xfer=%u)\n", c->max_transfer_size);
    return 0;
}

static int rndis_set_filter(usb_ctx *u, uint32_t filter) {
    uint8_t buf[sizeof(rndis_set_msg) + sizeof(uint32_t)];
    memset(buf, 0, sizeof(buf));
    rndis_set_msg *m = (rndis_set_msg *)buf;
    m->msg_type = RNDIS_MSG_SET;
    m->msg_len = sizeof(buf);
    m->request_id = ++u->request_id;
    m->oid = OID_GEN_CURRENT_PACKET_FILTER;
    m->info_buf_len = sizeof(uint32_t);
    /* Offset from the RequestId field (byte 8) to the info buffer, which sits
     * immediately after this header: sizeof(header) - 8. Getting this wrong
     * makes the device read the filter from the wrong bytes (e.g. 0 = receive
     * nothing). */
    m->info_buf_offset = sizeof(rndis_set_msg) - RNDIS_INFOBUF_BASE;
    memcpy(buf + sizeof(rndis_set_msg), &filter, sizeof(filter));

    uint8_t resp[CTRL_BUF_MAX];
    int n = rndis_transact(u, buf, sizeof(buf), resp, sizeof(resp));
    if (n < (int)sizeof(rndis_set_cmplt)) return -1;
    rndis_set_cmplt *c = (rndis_set_cmplt *)resp;
    if (c->status != RNDIS_STATUS_SUCCESS) {
        fprintf(stderr, "[usb] SET filter failed: 0x%x\n", c->status);
        return -1;
    }
    return 0;
}

static int rndis_query_mac(usb_ctx *u, uint8_t mac[ETHER_ADDR_LEN]) {
    rndis_query_msg m;
    memset(&m, 0, sizeof(m));
    m.msg_type = RNDIS_MSG_QUERY;
    m.msg_len = sizeof(m);
    m.request_id = ++u->request_id;
    m.oid = OID_802_3_CURRENT_ADDRESS;
    m.info_buf_len = 0;
    /* No input buffer, but point offset past the header as Linux does. */
    m.info_buf_offset = sizeof(rndis_query_msg) - RNDIS_INFOBUF_BASE;

    uint8_t resp[CTRL_BUF_MAX];
    int n = rndis_transact(u, &m, sizeof(m), resp, sizeof(resp));
    if (n < (int)sizeof(rndis_query_cmplt)) return -1;
    rndis_query_cmplt *c = (rndis_query_cmplt *)resp;
    if (c->status != RNDIS_STATUS_SUCCESS || c->info_buf_len < ETHER_ADDR_LEN)
        return -1;
    /* Reply buffer is at (RequestId + info_buf_offset) = 8 + info_buf_offset. */
    uint32_t off = RNDIS_INFOBUF_BASE + c->info_buf_offset;
    if (off + ETHER_ADDR_LEN > (uint32_t)n) return -1;
    memcpy(mac, resp + off, ETHER_ADDR_LEN);
    return 0;
}

/* Pull the two bulk endpoints out of an interface descriptor. */
static int iface_bulk_eps(const struct libusb_interface_descriptor *id,
                          uint8_t *in, uint8_t *out) {
    *in = *out = 0;
    for (int e = 0; e < id->bNumEndpoints; e++) {
        const struct libusb_endpoint_descriptor *ep = &id->endpoint[e];
        if ((ep->bmAttributes & 0x03) != LIBUSB_TRANSFER_TYPE_BULK) continue;
        if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) *in = ep->bEndpointAddress;
        else *out = ep->bEndpointAddress;
    }
    return (*in && *out) ? 0 : -1;
}

/*
 * Find the RNDIS function's control + data interfaces.
 *
 * This device is a 6-interface composite (RNDIS + mass storage + vendor + CDC
 * ACM serial), and BOTH the RNDIS data interface and the ACM data interface are
 * CDC-data class (0x0A). So we must not just grab "a 0x0A interface" - we must
 * pick the RNDIS control interface specifically, then take the CDC-data
 * interface that follows it (per the CDC composition, the data interface
 * immediately follows its control interface).
 *
 *   iface 0: class 0xE0 sub 0x01 proto 0x03  <- Microsoft RNDIS control
 *   iface 1: class 0x0A                        <- its data interface
 *   ...
 *   iface 4: class 0x02 sub 0x02 proto 0x01   <- CDC-ACM control (NOT ours)
 *   iface 5: class 0x0A                        <- ACM data (NOT ours)
 */
static int detect_endpoints(usb_ctx *u, libusb_device *dev) {
    struct libusb_config_descriptor *cfg;
    if (libusb_get_active_config_descriptor(dev, &cfg) != 0) return -1;

    int comm_num = -1;
    /* 1) RNDIS control interface: wireless-controller class with the
     *    RNDIS-over-Ethernet subclass/protocol (0xE0 / 0x01 / 0x03). */
    for (int i = 0; i < cfg->bNumInterfaces && comm_num < 0; i++) {
        const struct libusb_interface_descriptor *id = &cfg->interface[i].altsetting[0];
        if (id->bInterfaceClass == 0xE0 &&
            id->bInterfaceSubClass == 0x01 &&
            id->bInterfaceProtocol == 0x03) {
            comm_num = id->bInterfaceNumber;
        }
    }
    if (comm_num < 0) {
        fprintf(stderr, "[usb] no RNDIS control interface (0xE0/01/03) found\n");
        libusb_free_config_descriptor(cfg);
        return -1;
    }

    /* 2) Its data interface: the first CDC-data (0x0A) interface whose number
     *    is greater than the control interface's, with two bulk endpoints. */
    int data_num = -1;
    uint8_t in = 0, out = 0;
    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const struct libusb_interface_descriptor *id = &cfg->interface[i].altsetting[0];
        if (id->bInterfaceClass == 0x0A &&
            id->bInterfaceNumber > comm_num &&
            iface_bulk_eps(id, &in, &out) == 0) {
            data_num = id->bInterfaceNumber;
            break;   /* take the FIRST following data iface, not the last */
        }
    }
    libusb_free_config_descriptor(cfg);
    if (data_num < 0) {
        fprintf(stderr, "[usb] no CDC-data interface after control iface %d\n", comm_num);
        return -1;
    }

    u->iface_comm = comm_num;
    u->iface_data = data_num;
    u->ep_in = in;
    u->ep_out = out;
    fprintf(stderr, "[usb] RNDIS comm iface=%d data iface=%d ep_in=0x%02x ep_out=0x%02x\n",
            u->iface_comm, u->iface_data, u->ep_in, u->ep_out);
    return 0;
}

usb_ctx *usb_open(uint16_t vid, uint16_t pid, uint8_t out_drone_mac[ETHER_ADDR_LEN]) {
    usb_ctx *u = calloc(1, sizeof(*u));
    if (!u) return NULL;

    if (libusb_init(&u->ctx) != 0) { free(u); return NULL; }

    u->dev = libusb_open_device_with_vid_pid(u->ctx, vid, pid);
    if (!u->dev) {
        /* Expected while waiting for the aircraft to be plugged in; the caller
         * polls, so stay quiet here rather than spamming the log. */
        libusb_exit(u->ctx);
        free(u);
        return NULL;
    }

    libusb_device *dev = libusb_get_device(u->dev);
    if (detect_endpoints(u, dev) != 0) goto fail;

    libusb_set_auto_detach_kernel_driver(u->dev, 1);
    if (libusb_claim_interface(u->dev, u->iface_comm) != 0 ||
        libusb_claim_interface(u->dev, u->iface_data) != 0) {
        fprintf(stderr, "[usb] claim_interface failed\n");
        goto fail;
    }

    if (rndis_init(u) != 0) goto fail;
    if (rndis_set_filter(u, RNDIS_PACKET_TYPE_DIRECTED | RNDIS_PACKET_TYPE_BROADCAST) != 0)
        goto fail;
    if (rndis_query_mac(u, u->drone_mac) != 0) {
        fprintf(stderr, "[usb] MAC query failed; using zero MAC\n");
        memset(u->drone_mac, 0, ETHER_ADDR_LEN);
    }
    memcpy(out_drone_mac, u->drone_mac, ETHER_ADDR_LEN);
    fprintf(stderr, "[usb] drone MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
            u->drone_mac[0], u->drone_mac[1], u->drone_mac[2],
            u->drone_mac[3], u->drone_mac[4], u->drone_mac[5]);
    return u;

fail:
    usb_close(u);
    return NULL;
}

int usb_send_frame(usb_ctx *u, const uint8_t *frame, size_t len) {
    uint8_t buf[RNDIS_PACKET_MSG_HDR_LEN + ETHER_FRAME_MAX];
    if (len > ETHER_FRAME_MAX) return -1;
    rndis_packet_msg *p = (rndis_packet_msg *)buf;
    memset(p, 0, sizeof(*p));
    p->msg_type = RNDIS_MSG_PACKET;
    p->msg_len = (uint32_t)(RNDIS_PACKET_MSG_HDR_LEN + len);
    p->data_offset = RNDIS_PACKET_MSG_HDR_LEN - offsetof(rndis_packet_msg, data_offset);
    p->data_len = (uint32_t)len;
    memcpy(buf + RNDIS_PACKET_MSG_HDR_LEN, frame, len);

    int transferred = 0;
    int r = libusb_bulk_transfer(u->dev, u->ep_out, buf,
                                 (int)(RNDIS_PACKET_MSG_HDR_LEN + len),
                                 &transferred, BULK_TIMEOUT_MS);
    if (r != 0) {
        fprintf(stderr, "[usb] bulk OUT error: %s\n", libusb_strerror(r));
        return -1;
    }
    return transferred;
}

int usb_recv_frame(usb_ctx *u, uint8_t *frame, size_t cap) {
    uint8_t buf[RNDIS_PACKET_MSG_HDR_LEN + ETHER_FRAME_MAX + 64];
    int transferred = 0;
    int r = libusb_bulk_transfer(u->dev, u->ep_in, buf, sizeof(buf),
                                 &transferred, BULK_TIMEOUT_MS);
    if (r == LIBUSB_ERROR_TIMEOUT) return 0;
    if (r != 0) {
        fprintf(stderr, "[usb] bulk IN error: %s\n", libusb_strerror(r));
        return -1;
    }
    if (transferred < (int)RNDIS_PACKET_MSG_HDR_LEN) return 0;
    rndis_packet_msg *p = (rndis_packet_msg *)buf;
    if (p->msg_type != RNDIS_MSG_PACKET) return 0;  /* ignore non-data on bulk */
    uint32_t off = offsetof(rndis_packet_msg, data_offset) + p->data_offset;
    if (off + p->data_len > (uint32_t)transferred) return 0;
    size_t len = p->data_len;
    if (len > cap) len = cap;
    memcpy(frame, buf + off, len);
    return (int)len;
}

int usb_keepalive(usb_ctx *u) {
    rndis_keepalive_msg m;
    m.msg_type = RNDIS_MSG_KEEPALIVE;
    m.msg_len = sizeof(m);
    m.request_id = ++u->request_id;
    if (ctrl_send(u, &m, sizeof(m)) < 0) return -1;
    uint8_t resp[CTRL_BUF_MAX];
    (void)ctrl_recv(u, resp, sizeof(resp));  /* completion is best-effort */
    return 0;
}

void usb_close(usb_ctx *u) {
    if (!u) return;
    if (u->dev) {
        libusb_release_interface(u->dev, u->iface_data);
        libusb_release_interface(u->dev, u->iface_comm);
        libusb_close(u->dev);
    }
    if (u->ctx) libusb_exit(u->ctx);
    free(u);
}
