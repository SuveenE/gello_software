#!/bin/bash
# Print CAN interface info for writing /etc/udev/rules.d/90-can.rules

set -e

can_ifaces=$(ls -d /sys/class/net/can* 2>/dev/null || true)

if [[ -z "$can_ifaces" ]]; then
    echo "No CAN interfaces found in WSL."
    echo ""
    echo "Attach dongles from Windows (Admin PowerShell), then re-run:"
    echo '  usbipd attach --wsl --busid 6-1'
    echo '  usbipd attach --wsl --busid 6-2'
    echo ""
    echo "Then: sh scripts/reset_all_can.sh"
    exit 1
fi

echo "Found CAN interfaces:"
echo ""

for iface_path in $can_ifaces; do
    iface=$(basename "$iface_path")
    echo "========== $iface =========="
    ip link show "$iface" 2>/dev/null | sed 's/^/  /'
    echo ""
    echo "  udev attributes (use ATTRS{serial} in rules):"
    udevadm info -a -p "/sys/class/net/$iface" | grep -E 'ATTRS\{serial\}' | sed 's/^/  /' | sort -u
    udevadm info -a -p "/sys/class/net/$iface" | grep -E 'ATTRS\{product\}|ATTRS\{manufacturer\}' | sed 's/^/  /' | sort -u
    echo ""
done

echo "Suggested /etc/udev/rules.d/90-can.rules (edit serials and left/right assignment):"
echo ""
idx=0
for iface_path in $can_ifaces; do
    iface=$(basename "$iface_path")
    serial=$(udevadm info -a -p "/sys/class/net/$iface" | grep -oP 'ATTRS\{serial\}=="\K[^"]+' | head -1)
    if [[ $idx -eq 0 ]]; then name="can_left"; else name="can_right"; fi
    if [[ -n "$serial" ]]; then
        echo "SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{serial}==\"$serial\", NAME=\"$name\"  # was $iface"
    else
        echo "# $iface: no serial found — may need ATTRS{idVendor}==\"1d50\", ATTRS{idProduct}==\"606f\" plus another unique ATTR"
    fi
    idx=$((idx + 1))
done
echo ""
echo "After saving rules:"
echo "  sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger"
echo "  Detach + re-attach CAN dongles from Windows, then: sh scripts/reset_all_can.sh"
