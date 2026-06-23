#!/bin/bash

if [ "$(id -u)" != "0" ]; then
    SUDO="sudo"
else
    SUDO=""
fi

# Function to reset a CAN interface
reset_can_interface() {
    local iface=$1
    echo "Resetting CAN interface: $iface"
    $SUDO ip link set "$iface" down
    $SUDO ip link set "$iface" up type can bitrate 1000000
}

# Get all CAN interfaces
can_interfaces=$(ip link show | grep -oP '(?<=: )(can\w+)')

# Check if any CAN interfaces were found
if [ -z "$can_interfaces" ]; then
    echo "No CAN interfaces found."
    echo "Attach CAN dongles to WSL first (Windows Admin PowerShell):"
    echo '  usbipd attach --wsl --busid 6-1'
    echo '  usbipd attach --wsl --busid 6-2'
    exit 1
fi

# Reset each CAN interface
echo "Detected CAN interfaces: $can_interfaces"
for iface in $can_interfaces; do
    reset_can_interface "$iface"
done

echo "All CAN interfaces have been reset with bitrate 1000000."
