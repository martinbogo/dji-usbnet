# dji-usbnet

A userspace RNDIS host for using **DJI Assistant 2** on modern macOS (Apple
Silicon, macOS 11+) without a kernel extension, without disabling SIP, and
without an Apple kernel-extension entitlement. It is a drop-in replacement for
the HoRNDIS kext.

## Overview

DJI aircraft in the RNDIS family present themselves to the host as a USB RNDIS
Ethernet device, and DJI Assistant reaches the aircraft over IP at
`192.168.42.2`. Historically that required the HoRNDIS kernel extension to turn
the USB device into a network interface. On Apple Silicon that approach is
impractical: a kext must be signed with a special Apple entitlement, and loading
any third-party kext requires lowering the machine's security (Reduced Security
plus third-party kext approval, and in practice SIP disabled).

`dji-usbnet` performs the same function entirely in userspace:

- **libusb** claims the aircraft and implements the RNDIS host protocol.
- **utun** (`SYSPROTO_CONTROL` / `com.apple.net.utun_control`) publishes a
  routable IP interface. Root is required to create it, but no kext and no
  entitlement.
- A layer-2/3 bridge terminates Ethernet and ARP on the USB side so the utun
  side only ever sees IP.

DJI Assistant is unmodified; it connects to `192.168.42.2` as before.

```
App --connect(192.168.42.2)--> utun --IP--> [bridge +Eth/+ARP] --> RNDIS/USB --> drone
drone --> RNDIS/USB --> [bridge -Eth, answer ARP] --IP--> utun --> App
```

## Compatibility

Developed and verified against a **DJI Mavic Pro** (USB `2ca3:001f`) on macOS 26.
Other RNDIS-family aircraft (Phantom 3/4, Inspire, and similar) are expected to
work; reports and USB IDs for additional models are welcome.

To check whether a given aircraft uses the IP path, connect it, open DJI
Assistant, and run:

```sh
lsof -nP -iTCP@192.168.42.2
```

Note that DJI Assistant also communicates with the flight controller over a
serial (`/dev/cu.usbmodem`) channel that macOS supports natively, so an aircraft
can be detected and partially operated without an RNDIS driver. Operations that
run over IP require this bridge (or HoRNDIS).

## Verification

With a Mavic Pro in normal operating mode:

| Capability | Result |
|------------|--------|
| USB descriptor / endpoint auto-detection | RNDIS control iface 0, data iface 1, interrupt IN, bulk IN/OUT |
| RNDIS control handshake (INIT / SET filter / QUERY MAC / KEEPALIVE) | Completes |
| DHCP over RNDIS | Lease obtained from the aircraft's DHCP server (`192.168.42.2` → host `192.168.42.3`) |
| Host → drone (bulk OUT) | 100% delivery, no drops, verified under load |
| Drone → host (bulk IN) | Inbound frames received and forwarded to utun |

The aircraft firewalls unsolicited TCP and does not advertise services (no
mDNS/SSDP); DJI Assistant connects to it on a fixed port.

## Requirements

- macOS 11 or later (Apple Silicon or Intel)
- [libusb](https://libusb.info): `brew install libusb`
- Xcode command line tools: `xcode-select --install`

## Build

```sh
make
```

This produces `./dji-usbnet` (the bridge). `make probe` additionally builds
`./dji-probe`, a USB descriptor dumper for identifying a device.

## Usage

```sh
sudo ./dji-usbnet
```

Root is required only to create and configure the utun interface. The process
waits for the aircraft, brings the link up, and continues to serve across
unplug/replug until interrupted. Launch DJI Assistant as usual once it is
running.

For a different DJI model, pass its USB IDs (from `./dji-probe` or
`system_profiler SPUSBDataType`):

```sh
sudo ./dji-usbnet 0xVID 0xPID
```

Verbose per-frame and statistics logging:

```sh
sudo DJI_DEBUG=1 ./dji-usbnet
```

### Addressing

By default the bridge requests an address by DHCP over the RNDIS link; DJI
aircraft that use the IP path run a DHCP server at `192.168.42.2`. If there is no
response within a few seconds, the bridge falls back to a static
`192.168.42.1`. To skip DHCP:

```sh
sudo ./dji-usbnet --static
```

## Install (optional)

```sh
./packaging/install.sh            # build and install /usr/local/bin/dji-usbnet
./packaging/install.sh --daemon   # also install and load the LaunchDaemon
./packaging/uninstall.sh          # remove
```

The LaunchDaemon runs as root, starts at boot, and restarts on exit. While
loaded it holds the RNDIS interfaces (0 and 1) on the device.

## Troubleshooting

- **No traffic reaches the drone (`tun_in` stays 0):** check for an active VPN.
  VPNs with a kill switch or full-tunnel routing (NordVPN/NordLynx, WireGuard,
  Tailscale exit nodes, enterprise clients) install a global route that captures
  utun-bound packets before the routing table is consulted. Disconnect the VPN
  and retry.
- **`could not create utun (are you root?)`:** run with `sudo`.
- **`device ... not found`:** the aircraft is off, asleep, or in a different USB
  mode. Power-cycle it and confirm with `./dji-probe`.
- **During a firmware update** the aircraft re-enumerates with an additional
  vendor interface and its RNDIS IP stack may be inactive. Run the bridge with
  the aircraft in normal operating mode.
- **`bulk OUT error` / "drone is not accepting RNDIS data":** the aircraft is not
  draining its OUT endpoint, typically because its RNDIS IP stack is not active
  (for example while in firmware-update mode). The bridge drops outbound frames
  and continues rather than reconnecting.
- **DJI Assistant misbehaves while the bridge runs:** stop the bridge to rule out
  USB contention. It and Assistant share the device on different interfaces.

## Architecture

| File | Role |
|------|------|
| `src/usb.c` | libusb RNDIS host: control handshake and bulk `REMOTE_NDIS_PACKET_MSG` framing |
| `src/utun.c` | macOS userspace tunnel interface (create, configure, read/write) |
| `src/bridge.c` | Ethernet/ARP ↔ IP shim and drone-MAC learning |
| `src/dhcp.c` | DHCP client (DISCOVER/REQUEST) over the RNDIS link |
| `src/rndis.h` | RNDIS message and OID definitions |
| `src/main.c` | wait-for-device and reconnect loop, packet pump, frame decoding |
| `src/probe.c` | standalone USB descriptor dumper (`make probe`) |

Implementation notes:

- RNDIS `InformationBufferOffset` fields are measured from the RequestId field
  (8 bytes into the message).
- The `REMOTE_NDIS_PACKET_MSG` header is 44 bytes (`DataOffset` = 36); messages
  are padded to a 4-byte boundary, and a zero-length packet follows any bulk OUT
  whose length is an exact multiple of the endpoint's max packet size.
- Control transactions wait for `RESPONSE_AVAILABLE` on the interrupt IN pipe
  before issuing `GET_ENCAPSULATED_RESPONSE`, falling back to polling.
- The receive path parses bundled `REMOTE_NDIS_PACKET_MSG` records, so one bulk
  IN transfer may yield multiple Ethernet frames.
- utun is point-to-point layer 3. The bridge answers the drone's ARP for the
  host address with a synthetic locally administered MAC and learns the drone's
  MAC from inbound frames.
- The packet pump is single-threaded (select on utun, short-timeout bulk poll on
  USB). A latency-sensitive build would move the bulk IN read to its own thread
  or use libusb asynchronous transfers.

## Contributing

Test reports from other RNDIS-family DJI aircraft are especially useful: whether
DJI Assistant connects through `dji-usbnet`, along with `./dji-probe` output and,
where applicable, `DJI_DEBUG=1` logs. New USB IDs and per-model notes are welcome.

## License

MIT; see [LICENSE](LICENSE). This is an independent userspace implementation and
does not incorporate HoRNDIS source. "HoRNDIS" and "DJI" are referenced only for
interoperability.
