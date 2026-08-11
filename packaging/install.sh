#!/bin/sh
# Install dji-usbnet: build, copy the binary to /usr/local/bin, and (optionally)
# install the LaunchDaemon so it runs at boot.
#
# Usage:
#   ./packaging/install.sh            # build + install binary only
#   ./packaging/install.sh --daemon   # also install & load the LaunchDaemon
set -eu

REPO_DIR=$(cd "$(dirname "$0")/.." && pwd)
BIN_DST=/usr/local/bin/dji-usbnet
PLIST_SRC="$REPO_DIR/packaging/com.martinbogomolni.dji-usbnet.plist"
PLIST_DST=/Library/LaunchDaemons/com.martinbogomolni.dji-usbnet.plist

echo "==> Building"
make -C "$REPO_DIR"

echo "==> Installing binary to $BIN_DST (needs sudo)"
sudo mkdir -p /usr/local/bin
sudo install -m 0755 "$REPO_DIR/dji-usbnet" "$BIN_DST"

if [ "${1:-}" = "--daemon" ]; then
    echo "==> Installing LaunchDaemon to $PLIST_DST"
    sudo install -m 0644 "$PLIST_SRC" "$PLIST_DST"
    sudo launchctl unload "$PLIST_DST" 2>/dev/null || true
    sudo launchctl load "$PLIST_DST"
    echo "==> Loaded. Logs: /var/log/dji-usbnet.log"
else
    echo "==> Binary installed. Run it manually with:  sudo $BIN_DST"
    echo "    (add --daemon to this script to run it at boot instead)"
fi
