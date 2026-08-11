# dji-usbnet

A **userspace** replacement for the HoRNDIS kernel extension, for using
**DJI Assistant 2** on modern macOS (Apple Silicon, macOS 11+) **without a kext,
without disabling SIP, and without an Apple kernel-extension entitlement.**

Some DJI aircraft present themselves to the host as a **USB RNDIS Ethernet
device**, and DJI Assistant talks to them over IP at `192.168.42.2`. On old
macOS you installed the HoRNDIS kext to turn that USB device into a network
interface. On Apple Silicon that path is painful: the kext must be signed with a
special Apple entitlement, and loading a third-party kext at all requires
lowering the machine's security (Reduced Security + third-party kexts, and in
practice SIP off).

`dji-usbnet` does the same job from userspace instead:

- **libusb** claims the aircraft and speaks the RNDIS host protocol (no kext).
- **utun** (`SYSPROTO_CONTROL` / `com.apple.net.utun_control`) publishes a
  routable IP interface (no kext, root only, no entitlement).
- A small **layer-2/3 bridge** terminates Ethernet + ARP on the USB side so the
  utun side only ever sees IP.

DJI Assistant is completely unmodified: it connects to `192.168.42.2` exactly as
before.

```
App --connect(192.168.42.2)--> utun --IP--> [bridge +Eth/+ARP] --> RNDIS/USB --> drone
drone --> RNDIS/USB --> [bridge -Eth, answer ARP] --IP--> utun --> App
```

## Do you even need this?

**Maybe not.** Not every DJI product uses the RNDIS/IP path. Newer aircraft (for
example the **Mavic Pro**) talk to DJI Assistant entirely over **raw USB** and
never touch `192.168.42.2` - for those, Assistant works with no kext and no
`dji-usbnet` at all. DJI's installer bundles HoRNDIS because *some* products
(older **Phantom**-class models) need the IP link.

Quick check: with the aircraft connected and DJI Assistant open, run

```sh
lsof -nP -iTCP@192.168.42.2
```

If that shows a connection, your product uses the IP path and this tool is
relevant. If it is empty but Assistant still sees the drone, you do not need
this tool.

## Status / what is verified

Developed and tested against a **DJI Mavic Pro** (USB `2ca3:001f`) on macOS 26.

| Piece | State |
|-------|-------|
| Build (clang + libusb) | Working |
| USB descriptor / endpoint auto-detection | Verified (RNDIS control iface 0, data iface 1) |
| RNDIS control handshake (INIT / SET filter / QUERY MAC / KEEPALIVE) | Verified |
| utun create + IP config + packet delivery | Verified |
| Host -> drone path (utun -> RNDIS bulk) | Verified |
| DHCP client over RNDIS (frame construction) | Verified structurally; awaits a drone that answers |
| Drone -> host path (inbound) | **Implemented but not yet exercised** |

**Honest caveat:** the inbound half has not been confirmed end-to-end, because
the only device on hand (Mavic Pro) does not use the RNDIS/IP path, so it never
sends IP frames back. The host side is proven; the code that handles drone->host
traffic is written and correct by construction but wants testing on a
Phantom-class aircraft that actually uses the link. **If you try it on such a
product, please open an issue with the result** (see Contributing).

## Requirements

- macOS 11 or later (Apple Silicon or Intel)
- [libusb](https://libusb.info): `brew install libusb`
- Xcode command line tools: `xcode-select --install`

## Build

```sh
make
```

Produces `./dji-usbnet` (the bridge) and, via `make probe`, `./dji-probe`
(a descriptor dumper for identifying a new device).

## Run

```sh
sudo ./dji-usbnet
```

Root is required only to create and configure the utun interface. The process
waits for the aircraft, brings the link up, and keeps serving across
unplug/replug until you Ctrl-C it. Then launch DJI Assistant as usual.

For a different DJI model, pass its USB IDs (find them with `./dji-probe` or
`system_profiler SPUSBDataType`):

```sh
sudo ./dji-usbnet 0xVID 0xPID
```

Verbose per-packet/statistics logging:

```sh
sudo DJI_DEBUG=1 ./dji-usbnet
```

### Addressing (DHCP vs static)

By default the bridge first tries **DHCP** over the RNDIS link: DJI aircraft that
use the IP path run a DHCP server at `192.168.42.2` and expect the host to lease
its address. If the drone answers, its lease is used (and that also proves the
drone's RNDIS IP stack is alive). If there is no response within a few seconds,
the bridge falls back to a static `192.168.42.1`. To skip DHCP entirely:

```sh
sudo ./dji-usbnet --static
```

## Install (optional, run at boot)

```sh
./packaging/install.sh            # build + install /usr/local/bin/dji-usbnet
./packaging/install.sh --daemon   # also install & load the LaunchDaemon
./packaging/uninstall.sh          # remove everything
```

The LaunchDaemon runs as root, starts at boot, and restarts on exit. Install it
only if your product needs the IP path; while loaded it holds RNDIS interfaces
0/1 on the device.

## Troubleshooting

- **No traffic reaches the drone / `tun_in` stays 0** - check for an active
  **VPN first**. VPNs with a kill switch or "route all traffic" (NordVPN /
  NordLynx, WireGuard, Tailscale exit nodes, enterprise clients) install a
  global route that swallows every utun-bound packet before the routing table
  sees it. Disconnect the VPN and retry. This was the single biggest gotcha
  during development.
- **`could not create utun (are you root?)`** - run with `sudo`.
- **`device ... not found`** - the aircraft is off, asleep, or in a different
  USB mode; power-cycle it and confirm with `./dji-probe`.
- **Firmware flashing** - the aircraft re-enumerates (an extra vendor interface
  appears) during a flash. Let the flash finish before starting the bridge.
- **Assistant hiccups while the bridge runs** - stop the bridge; it and
  Assistant share the USB device (different interfaces, but worth ruling out).

## How it works (internals)

| File | Role |
|------|------|
| `src/usb.c` | libusb RNDIS host: handshake + bulk RNDIS_PACKET_MSG framing |
| `src/utun.c` | macOS userspace tunnel interface (create, configure, read/write) |
| `src/bridge.c` | Ethernet/ARP <-> IP shim, drone-MAC learning |
| `src/dhcp.c` | minimal DHCP client (DISCOVER/REQUEST) over the RNDIS link |
| `src/rndis.h` | host-side RNDIS message and OID definitions |
| `src/main.c` | wait-for-device + reconnect loop and the packet pump |
| `src/probe.c` | standalone USB descriptor dumper (`make probe`) |

Notes for hackers:

- RNDIS `InformationBufferOffset` fields are measured from the **RequestId**
  field (8 bytes into the message). Getting the SET-filter offset wrong silently
  programs a packet filter of 0 ("receive nothing").
- utun is strictly point-to-point / layer 3; it rejects a plain `/24`. The
  bridge answers the drone's ARP for the host IP with a synthetic locally
  administered MAC, and learns the drone's MAC from its first inbound frame.
- The packet pump is single-threaded (select on utun, short-timeout bulk poll on
  USB). Fine for control-plane traffic; a latency-sensitive build would move the
  USB IN read to its own thread or use libusb async transfers.

## Contributing

The most useful contribution right now is **a test report from a DJI product
that uses the RNDIS/IP path** (Phantom 3/4, Inspire, etc.): does DJI Assistant
connect through `dji-usbnet`? Please include the output of `./dji-probe` and,
if it works, `DJI_DEBUG=1` stats. New VID/PIDs and per-model notes are welcome.

## License

MIT - see [LICENSE](LICENSE). This is an independent userspace reimplementation;
it does not incorporate HoRNDIS source (which is GPL). "HoRNDIS" and "DJI" are
referenced only for interoperability.
