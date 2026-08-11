/*
 * utun.c - macOS userspace tunnel interface.
 *
 * This is the piece that replaces the HoRNDIS kext: instead of a kernel network
 * driver publishing an enX interface, we open a utun via the SYSPROTO_CONTROL /
 * com.apple.net.utun_control API. No kext, no SIP change, no entitlement; only
 * root (needed to create the interface and set its address/route).
 *
 * utun is a point-to-point IP (layer 3) interface. Each packet on the fd is
 * prefixed with a 4-byte address family (AF_INET / AF_INET6) in network order.
 */
#include "usbnet.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/sys_domain.h>
#include <sys/kern_control.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_utun.h>
#include <netinet/in.h>
#include <arpa/inet.h>

struct utun_ctx {
    int fd;
    char name[16];
};

static int run(const char *fmt, ...) {
    char cmd[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cmd, sizeof(cmd), fmt, ap);
    va_end(ap);
    int rc = system(cmd);
    if (rc != 0)
        fprintf(stderr, "[utun] command failed (%d): %s\n", rc, cmd);
    return rc;
}

utun_ctx *utun_open(const char *host_ip, const char *drone_ip,
                    const char *netmask, char out_name[16]) {
    int fd = socket(PF_SYSTEM, SOCK_DGRAM, SYSPROTO_CONTROL);
    if (fd < 0) {
        perror("[utun] socket(PF_SYSTEM)");
        return NULL;
    }

    struct ctl_info info;
    memset(&info, 0, sizeof(info));
    strncpy(info.ctl_name, UTUN_CONTROL_NAME, sizeof(info.ctl_name));
    if (ioctl(fd, CTLIOCGINFO, &info) < 0) {
        perror("[utun] ioctl(CTLIOCGINFO)");
        close(fd);
        return NULL;
    }

    /* sc_unit == 0 lets the kernel pick the first free utunN. */
    struct sockaddr_ctl sc;
    memset(&sc, 0, sizeof(sc));
    sc.sc_len = sizeof(sc);
    sc.sc_family = AF_SYSTEM;
    sc.ss_sysaddr = AF_SYS_CONTROL;
    sc.sc_id = info.ctl_id;
    sc.sc_unit = 0;

    if (connect(fd, (struct sockaddr *)&sc, sizeof(sc)) < 0) {
        perror("[utun] connect(utun_control) - are we root?");
        close(fd);
        return NULL;
    }

    utun_ctx *t = calloc(1, sizeof(*t));
    if (!t) {
        close(fd);
        return NULL;
    }
    t->fd = fd;

    socklen_t nlen = sizeof(t->name);
    if (getsockopt(fd, SYSPROTO_CONTROL, UTUN_OPT_IFNAME, t->name, &nlen) < 0) {
        perror("[utun] getsockopt(UTUN_OPT_IFNAME)");
        close(fd);
        free(t);
        return NULL;
    }
    snprintf(out_name, 16, "%s", t->name);

    /* Bring the interface up with host_ip as a point-to-point peer to drone_ip.
     * The P2P config auto-installs a host route to the peer, so the app's
     * connect(192.168.42.2) already lands here; ifconfig failing is fatal. */
    if (run("/sbin/ifconfig %s inet %s %s netmask %s up",
            t->name, host_ip, drone_ip, netmask) != 0) {
        fprintf(stderr, "[utun] failed to configure %s\n", t->name);
        close(fd);
        free(t);
        return NULL;
    }
    /* Belt-and-suspenders explicit host route; harmless if it already exists. */
    run("/sbin/route -q -n add -host %s -interface %s 2>/dev/null",
        drone_ip, t->name);

    fprintf(stderr, "[utun] %s up: host %s peer %s\n", t->name, host_ip, drone_ip);
    return t;
}

int utun_read_ip(utun_ctx *t, uint8_t *ip, size_t cap) {
    uint8_t buf[4 + ETHER_MTU + 128];
    ssize_t n = read(t->fd, buf, sizeof(buf));
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN) return 0;
        perror("[utun] read");
        return -1;
    }
    if (n <= 4) return 0;              /* just the AF header, nothing to carry */
    size_t iplen = (size_t)n - 4;      /* strip the 4-byte AF_INET prefix */
    if (iplen > cap) iplen = cap;
    memcpy(ip, buf + 4, iplen);
    return (int)iplen;
}

int utun_write_ip(utun_ctx *t, const uint8_t *ip, size_t len) {
    uint8_t buf[4 + ETHER_MTU + 128];
    if (len > sizeof(buf) - 4) return -1;
    /* Prepend AF header. IPv4 only in this scaffold. */
    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = AF_INET;
    memcpy(buf + 4, ip, len);
    ssize_t n = write(t->fd, buf, len + 4);
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN) return 0;
        perror("[utun] write");
        return -1;
    }
    return (int)(n > 4 ? n - 4 : 0);
}

int utun_fd(utun_ctx *t) { return t->fd; }

void utun_close(utun_ctx *t) {
    if (!t) return;
    if (t->fd >= 0) close(t->fd);
    free(t);
}
