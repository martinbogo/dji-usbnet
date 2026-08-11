/*
 * main.c - dji-usbnet: a userspace replacement for the HoRNDIS kext.
 *
 * Presents a DJI aircraft's USB RNDIS link as a routable utun interface so that
 * DJI Assistant (which reaches some products at 192.168.42.2 over IP) works
 * unmodified, with no kernel extension, no SIP change, and no Apple entitlement.
 * Root is required only to create/configure the utun interface.
 *
 * Data flow:
 *
 *   App --connect(192.168.42.2)--> utun --IP--> [bridge +Eth/+ARP] --> RNDIS/USB --> drone
 *   drone --> RNDIS/USB --> [bridge -Eth, answer ARP] --IP--> utun --> App
 *
 * The process waits for the aircraft, runs until it is unplugged (or a signal
 * arrives), then goes back to waiting - so it is safe to leave running and to
 * manage with launchd.
 */
#include "usbnet.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig) { (void)sig; g_stop = 1; }

static int g_static = 0;   /* --static: skip DHCP, use the static host address */

/*
 * Move packets between the utun interface and the drone until the device
 * disappears (usb hard error) or a stop signal arrives. Returns 0 on a clean
 * stop request, -1 if the session ended because of a USB error (reconnect).
 */
static int run_session(usb_ctx *usb, utun_ctx *tun, bridge_ctx *bridge) {
    int tfd = utun_fd(tun);
    time_t last_ka = time(NULL);
    time_t last_stat = last_ka;
    int debug = (getenv("DJI_DEBUG") != NULL);
    unsigned long n_tun_in = 0, n_usb_out = 0, n_usb_in = 0, n_arp = 0, n_ip_up = 0;
    uint8_t ip[ETHER_MTU + 64];
    uint8_t eth[ETHER_FRAME_MAX + 64];
    uint8_t scratch[ETHER_FRAME_MAX + 64];

    while (!g_stop) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(tfd, &rfds);
        struct timeval tv = {0, 2000}; /* 2 ms: keep the USB IN poll responsive */

        int s = select(tfd + 1, &rfds, NULL, NULL, &tv);
        if (s < 0 && errno != EINTR) { perror("select"); return -1; }

        /* App -> drone: IP from utun, wrap to Ethernet, send over RNDIS. */
        if (s > 0 && FD_ISSET(tfd, &rfds)) {
            int iplen = utun_read_ip(tun, ip, sizeof(ip));
            if (iplen > 0) {
                n_tun_in++;
                int ethlen = bridge_ip_to_eth(bridge, ip, iplen, eth, sizeof(eth));
                if (ethlen > 0) {
                    if (usb_send_frame(usb, eth, ethlen) < 0) return -1;
                    n_usb_out++;
                }
                if (debug)
                    fprintf(stderr, "[dbg] utun->usb ip=%dB eth=%dB dst=%s\n",
                            iplen, ethlen, bridge->mac_known ? "drone" : "bcast");
            }
        }

        /* Drone -> app: pull frames from RNDIS, answer ARP locally, forward IP. */
        for (;;) {
            int ethlen = usb_recv_frame(usb, eth, sizeof(eth));
            if (ethlen == 0) break;      /* timeout / no data this round */
            if (ethlen < 0) return -1;   /* device error -> reconnect */
            n_usb_in++;
            size_t reply_len = 0;
            int act = bridge_eth_from_drone(bridge, eth, ethlen,
                                            scratch, sizeof(scratch), &reply_len);
            if (act == BRIDGE_IP) {
                if (reply_len > 0) { utun_write_ip(tun, scratch, reply_len); n_ip_up++; }
            } else if (act == BRIDGE_ARP_REPLY) {
                if (usb_send_frame(usb, scratch, reply_len) < 0) return -1;
                n_arp++;
            }
            if (debug)
                fprintf(stderr, "[dbg] usb->? eth=%dB act=%d\n", ethlen, act);
        }

        /* RNDIS keepalive ~every 5s so the device does not drop the link. */
        time_t now = time(NULL);
        if (now - last_ka >= 5) {
            if (usb_keepalive(usb) < 0) return -1;
            last_ka = now;
        }
        if (debug && now - last_stat >= 2) {
            fprintf(stderr, "[stat] tun_in=%lu usb_out=%lu usb_in=%lu arp=%lu ip_up=%lu\n",
                    n_tun_in, n_usb_out, n_usb_in, n_arp, n_ip_up);
            last_stat = now;
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    uint16_t vid = DJI_USB_VID, pid = DJI_USB_PID;
    /* Args: [--static] [VID PID]
     *   --static        skip DHCP, use the static host address
     *   VID PID          override USB IDs for other DJI models (hex ok) */
    int a = 1;
    if (a < argc && strcmp(argv[a], "--static") == 0) { g_static = 1; a++; }
    if (a + 1 < argc) {
        vid = (uint16_t)strtol(argv[a], NULL, 0);
        pid = (uint16_t)strtol(argv[a + 1], NULL, 0);
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    fprintf(stderr, "dji-usbnet: waiting for DJI aircraft %04x:%04x (Ctrl-C to stop)\n",
            vid, pid);

    /* Outer loop: wait for the aircraft, serve it, and on unplug go back to
     * waiting. This makes reconnects and launchd management seamless. */
    while (!g_stop) {
        uint8_t drone_mac[ETHER_ADDR_LEN];
        usb_ctx *usb = usb_open(vid, pid, drone_mac);
        if (!usb) {
            if (g_stop) break;
            sleep(2);            /* device not present yet; poll */
            continue;
        }

        /* Try DHCP from the drone first; fall back to a static host address.
         * (--static skips DHCP entirely.) */
        const char *host_ip = HOST_IP, *netmask = NET_MASK;
        dhcp_lease lease;
        if (!g_static) {
            uint8_t host_mac[ETHER_ADDR_LEN];
            bridge_host_mac(host_mac);
            if (dhcp_acquire(usb, host_mac, &lease) == 0) {
                host_ip = lease.ip;
                netmask = lease.netmask;
                fprintf(stderr, "dji-usbnet: using DHCP lease %s/%s\n", host_ip, netmask);
            } else {
                fprintf(stderr, "dji-usbnet: no DHCP response; using static %s "
                        "(drone RNDIS IP stack may be inactive)\n", HOST_IP);
            }
        }

        char ifname[16];
        utun_ctx *tun = utun_open(host_ip, DRONE_IP, netmask, ifname);
        if (!tun) {
            fprintf(stderr, "fatal: could not create utun (are you root?)\n");
            usb_close(usb);
            return 1;            /* config error, not transient - give up */
        }

        bridge_ctx bridge;
        bridge_init(&bridge, drone_mac, host_ip, DRONE_IP);
        fprintf(stderr, "dji-usbnet up: drone reachable at %s via %s (host %s)\n",
                DRONE_IP, ifname, host_ip);

        int rc = run_session(usb, tun, &bridge);

        utun_close(tun);
        usb_close(usb);
        if (rc < 0 && !g_stop) {
            fprintf(stderr, "dji-usbnet: aircraft disconnected; waiting for reconnect\n");
            sleep(1);
        }
    }

    fprintf(stderr, "\ndji-usbnet: shutting down\n");
    return 0;
}
