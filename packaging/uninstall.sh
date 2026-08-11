#!/bin/sh
# Remove dji-usbnet: unload/remove the LaunchDaemon and delete the binary.
set -eu

BIN_DST=/usr/local/bin/dji-usbnet
PLIST_DST=/Library/LaunchDaemons/com.martinbogomolni.dji-usbnet.plist

if [ -f "$PLIST_DST" ]; then
    echo "==> Unloading and removing LaunchDaemon"
    sudo launchctl unload "$PLIST_DST" 2>/dev/null || true
    sudo rm -f "$PLIST_DST"
fi

echo "==> Removing binary"
sudo rm -f "$BIN_DST"
echo "==> Done."
