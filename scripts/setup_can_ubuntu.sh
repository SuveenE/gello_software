#!/bin/bash
# Bring up YAM CAN interfaces on native Ubuntu and verify connectivity.
# Run: sudo bash scripts/setup_can_ubuntu.sh

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash scripts/setup_can_ubuntu.sh"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/90-can.rules.template"
RULES_DST="/etc/udev/rules.d/90-can.rules"

echo "=== Detected CAN adapters ==="
if ! ls /sys/class/net/can* >/dev/null 2>&1; then
    echo "No CAN interfaces found. Plug in your USB-CAN dongles and retry."
    exit 1
fi

for iface in /sys/class/net/can*; do
    name="$(basename "$iface")"
    serial="$(udevadm info -a -p "$iface" 2>/dev/null | grep 'ATTRS{serial}==' | head -1 | sed 's/.*=="\(.*\)"/\1/')"
    echo "  ${name}: serial=${serial}"
done

echo ""
echo "=== Installing udev rules (can_left / can_right) ==="
cp "$RULES_SRC" "$RULES_DST"
udevadm control --reload-rules
systemctl restart systemd-udevd
udevadm trigger

echo ""
echo "=== Bringing interfaces up at 1 Mbps ==="
modprobe gs_usb 2>/dev/null || true
for iface in /sys/class/net/can*; do
    name="$(basename "$iface")"
    ip link set "$name" down 2>/dev/null || true
    ip link set "$name" up type can bitrate 1000000
    echo "  ${name}: UP"
done

echo ""
echo "=== Current CAN state ==="
ip -details link show type can

echo ""
echo "If can_left/can_right do not appear yet, unplug and replug both CAN dongles."
echo ""
echo "Verify YAM motors respond (run without sudo after interfaces are UP):"
echo "  python -m i2rt.motor_config_tool.ping_motors --channel can_left"
echo "  python -m i2rt.motor_config_tool.ping_motors --channel can_right"
echo ""
echo "If motors appear on the wrong arm, swap the two serial lines in:"
echo "  ${RULES_DST}"
