#!/bin/bash
# Reset FTDI USB-serial adapters (GELLO U2D2) without physically unplugging.
# Use after a hard kill (Ctrl+\) leaves the adapter stuck (Dynamixel -3001 timeouts).
#
# Usage:
#   sudo bash scripts/reset_serial.sh          # reset all ftdi_sio ports
#   sudo bash scripts/reset_serial.sh ttyUSB0  # reset a single port

set -u

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash scripts/reset_serial.sh"
    exit 1
fi

DRIVER=/sys/bus/usb/drivers/usb

reset_one() {
    local tty="$1"
    local link="/sys/bus/usb-serial/devices/${tty}"
    if [ ! -e "$link" ]; then
        echo "  ${tty}: not found, skipping"
        return
    fi
    # Walk up from the interface (e.g. 3-2.3:1.0) to the USB device node (3-2.3)
    local intf
    intf=$(basename "$(readlink -f "$link" | sed 's|/ttyUSB.*||')")
    local dev="${intf%:*}"
    echo "  ${tty}: unbinding USB device ${dev}"
    echo "$dev" > "$DRIVER/unbind" 2>/dev/null
    sleep 1
    echo "  ${tty}: rebinding USB device ${dev}"
    echo "$dev" > "$DRIVER/bind" 2>/dev/null
    sleep 1
}

if [ "$#" -ge 1 ]; then
    reset_one "$1"
else
    echo "=== Resetting all FTDI serial adapters ==="
    found=0
    for link in /sys/bus/usb-serial/devices/ttyUSB*; do
        [ -e "$link" ] || continue
        found=1
        reset_one "$(basename "$link")"
    done
    [ "$found" -eq 0 ] && echo "No ttyUSB devices found."
fi

echo ""
echo "Current serial devices:"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none)"
