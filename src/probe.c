/*
 * probe.c - Dump the DJI device's USB descriptor (config, interfaces,
 * endpoints) so we can confirm detect_endpoints() picks the right RNDIS data
 * interface and bulk IN/OUT endpoints. Reading descriptors needs no claim, so
 * this is safe to run without disturbing anything.
 *
 * Build:  make probe
 * Run:    ./dji-probe            (no root needed just to read descriptors)
 */
#include <stdio.h>
#include <libusb.h>

#define VID 0x2ca3
#define PID 0x001f

static const char *xfer(int a) {
    switch (a & 0x03) {
        case LIBUSB_TRANSFER_TYPE_CONTROL:     return "control";
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: return "isoc";
        case LIBUSB_TRANSFER_TYPE_BULK:        return "bulk";
        case LIBUSB_TRANSFER_TYPE_INTERRUPT:   return "interrupt";
    }
    return "?";
}

int main(void) {
    libusb_context *ctx = NULL;
    if (libusb_init(&ctx) != 0) { fprintf(stderr, "libusb_init failed\n"); return 1; }

    libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h) {
        fprintf(stderr, "device %04x:%04x not found - is the drone on and connected?\n", VID, PID);
        libusb_exit(ctx);
        return 2;
    }
    libusb_device *dev = libusb_get_device(h);

    struct libusb_device_descriptor dd;
    libusb_get_device_descriptor(dev, &dd);
    printf("Device %04x:%04x  class=%d subclass=%d proto=%d numConfigs=%d\n",
           dd.idVendor, dd.idProduct, dd.bDeviceClass, dd.bDeviceSubClass,
           dd.bDeviceProtocol, dd.bNumConfigurations);

    struct libusb_config_descriptor *cfg;
    if (libusb_get_active_config_descriptor(dev, &cfg) != 0) {
        fprintf(stderr, "no active config\n");
        libusb_close(h); libusb_exit(ctx); return 3;
    }
    printf("Active config #%d, %d interfaces\n", cfg->bConfigurationValue, cfg->bNumInterfaces);

    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const struct libusb_interface *iface = &cfg->interface[i];
        for (int a = 0; a < iface->num_altsetting; a++) {
            const struct libusb_interface_descriptor *id = &iface->altsetting[a];
            printf("  iface %d alt %d: class=0x%02x subclass=0x%02x proto=0x%02x endpoints=%d\n",
                   id->bInterfaceNumber, id->bAlternateSetting,
                   id->bInterfaceClass, id->bInterfaceSubClass,
                   id->bInterfaceProtocol, id->bNumEndpoints);
            for (int e = 0; e < id->bNumEndpoints; e++) {
                const struct libusb_endpoint_descriptor *ep = &id->endpoint[e];
                printf("      ep 0x%02x  %-9s %s  maxpkt=%d\n",
                       ep->bEndpointAddress, xfer(ep->bmAttributes),
                       (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN " : "OUT",
                       ep->wMaxPacketSize);
            }
        }
    }

    libusb_free_config_descriptor(cfg);
    libusb_close(h);
    libusb_exit(ctx);
    return 0;
}
