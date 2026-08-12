/*
 * rndis.h - Minimal host-side RNDIS (Remote NDIS) definitions.
 *
 * RNDIS runs a small control protocol over the USB control endpoint (or an
 * interrupt notification + control transfers) and carries Ethernet frames,
 * each wrapped in an RNDIS_PACKET_MSG, over the bulk IN/OUT endpoints.
 *
 * References: Microsoft [MS-RNDIS] and the Linux drivers/net/usb/rndis_host.c
 * host implementation. Only the subset the DJI RNDIS gadget needs is declared.
 */
#ifndef DJI_USBNET_RNDIS_H
#define DJI_USBNET_RNDIS_H

#include <stdint.h>

/* Control message types (host <-> device). */
#define RNDIS_MSG_PACKET      0x00000001u  /* data path: wraps one Ethernet frame */
#define RNDIS_MSG_INIT        0x00000002u
#define RNDIS_MSG_INIT_CMPLT  0x80000002u
#define RNDIS_MSG_HALT        0x00000003u
#define RNDIS_MSG_QUERY       0x00000004u
#define RNDIS_MSG_QUERY_CMPLT 0x80000004u
#define RNDIS_MSG_SET         0x00000005u
#define RNDIS_MSG_SET_CMPLT   0x80000005u
#define RNDIS_MSG_RESET       0x00000006u
#define RNDIS_MSG_RESET_CMPLT 0x80000006u
#define RNDIS_MSG_INDICATE    0x00000007u  /* device-initiated status */
#define RNDIS_MSG_KEEPALIVE   0x00000008u
#define RNDIS_MSG_KEEPALIVE_CMPLT 0x80000008u

/* Status codes. */
#define RNDIS_STATUS_SUCCESS  0x00000000u

/* RNDIS versions we advertise. */
#define RNDIS_MAJOR_VERSION   1u
#define RNDIS_MINOR_VERSION   0u

/* Selected OIDs (used by QUERY/SET). */
#define OID_GEN_CURRENT_PACKET_FILTER 0x0001010Eu
#define OID_802_3_PERMANENT_ADDRESS   0x01010101u
#define OID_802_3_CURRENT_ADDRESS     0x01010102u
#define OID_GEN_MAXIMUM_FRAME_SIZE    0x00010106u

/* Packet-filter bits for OID_GEN_CURRENT_PACKET_FILTER (standard NDIS values).
 * NOTE: BROADCAST is 0x08, not 0x10 (0x10 is SOURCE_ROUTING). Getting this
 * wrong drops the BROADCAST bit, so the device never forwards broadcast frames
 * (DHCP offers, ARP requests) up to the host. */
#define RNDIS_PACKET_TYPE_DIRECTED       0x00000001u
#define RNDIS_PACKET_TYPE_MULTICAST      0x00000002u
#define RNDIS_PACKET_TYPE_ALL_MULTICAST  0x00000004u
#define RNDIS_PACKET_TYPE_BROADCAST      0x00000008u
#define RNDIS_PACKET_TYPE_SOURCE_ROUTING 0x00000010u
#define RNDIS_PACKET_TYPE_PROMISCUOUS    0x00000020u

/* USB class-specific control requests carrying the RNDIS command channel. */
#define USB_CDC_SEND_ENCAPSULATED_COMMAND 0x00
#define USB_CDC_GET_ENCAPSULATED_RESPONSE 0x01

#pragma pack(push, 1)

/* Common header shared by all control messages. */
typedef struct {
    uint32_t msg_type;
    uint32_t msg_len;
    uint32_t request_id;
} rndis_hdr;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_INIT */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t major_version;
    uint32_t minor_version;
    uint32_t max_transfer_size;
} rndis_init_msg;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_INIT_CMPLT */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t status;
    uint32_t major_version;
    uint32_t minor_version;
    uint32_t device_flags;
    uint32_t medium;
    uint32_t max_packets_per_transfer;
    uint32_t max_transfer_size;
    uint32_t packet_alignment_factor;
    uint32_t reserved[2];
} rndis_init_cmplt;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_SET */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t oid;
    uint32_t info_buf_len;
    uint32_t info_buf_offset; /* from &oid */
    uint32_t reserved;
    /* info buffer follows */
} rndis_set_msg;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_SET_CMPLT */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t status;
} rndis_set_cmplt;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_QUERY */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t oid;
    uint32_t info_buf_len;
    uint32_t info_buf_offset; /* from &oid */
    uint32_t reserved;
} rndis_query_msg;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_QUERY_CMPLT */
    uint32_t msg_len;
    uint32_t request_id;
    uint32_t status;
    uint32_t info_buf_len;
    uint32_t info_buf_offset;
    /* info buffer follows */
} rndis_query_cmplt;

typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_KEEPALIVE */
    uint32_t msg_len;
    uint32_t request_id;
} rndis_keepalive_msg;

/*
 * Data-path wrapper (REMOTE_NDIS_PACKET_MSG). One message carries exactly one
 * Ethernet frame here (no per-transfer batching). data_offset is measured from
 * &data_offset. This MUST be the full 44-byte / 11-field standard header - the
 * Linux/Android RNDIS gadget most DJI aircraft derive from expects it, and with
 * this layout data_offset auto-computes to 36. Dropping the vc_handle field
 * (40-byte header, data_offset 32) makes strict gadgets reject the frame.
 */
typedef struct {
    uint32_t msg_type;      /* RNDIS_MSG_PACKET */
    uint32_t msg_len;       /* header + payload */
    uint32_t data_offset;   /* to the Ethernet frame, from &data_offset (=> 36) */
    uint32_t data_len;      /* length of the Ethernet frame */
    uint32_t oob_data_offset;
    uint32_t oob_data_len;
    uint32_t num_oob;
    uint32_t per_packet_offset;
    uint32_t per_packet_len;
    uint32_t vc_handle;     /* DeviceVcHandle; zero */
    uint32_t reserved;      /* zero */
} rndis_packet_msg;

#pragma pack(pop)

#define RNDIS_PACKET_MSG_HDR_LEN ((uint32_t)sizeof(rndis_packet_msg))

#endif /* DJI_USBNET_RNDIS_H */
