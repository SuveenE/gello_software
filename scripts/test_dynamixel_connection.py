#!/usr/bin/env python3
"""Diagnostic tool for testing Dynamixel motor connections.

This script helps diagnose connection issues with Dynamixel motors by testing
various aspects of the communication setup.
"""

import glob
import os
import subprocess
import sys
import time
from typing import List, Optional

import tyro
from dataclasses import dataclass

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


@dataclass
class Args:
    port: Optional[str] = None
    """The port to test. If not provided, will auto-detect FTDI ports."""
    
    baudrate: int = 57600
    """Baudrate to test (default: 57600). Common values: 57600, 1000000, 115200"""
    
    scan_ids: bool = True
    """Whether to scan for motor IDs on the bus."""
    
    max_id: int = 20
    """Maximum motor ID to scan for (default: 20)."""


def find_ftdi_ports() -> List[str]:
    """Find all FTDI USB-Serial converter ports."""
    return glob.glob("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_*")


def check_port_permissions(port: str) -> bool:
    """Check if we have read/write permissions on the port."""
    if not os.path.exists(port):
        print(f"✗ Port {port} does not exist")
        return False
    
    if os.access(port, os.R_OK | os.W_OK):
        print(f"✓ Port {port} has read/write permissions")
        return True
    else:
        print(f"✗ Port {port} lacks read/write permissions")
        print(f"  Try: sudo chmod 666 {port}")
        return False


def check_port_in_use(port: str) -> bool:
    """Check if the port is being used by another process."""
    try:
        result = subprocess.run(
            ["lsof", port], 
            capture_output=True, 
            text=True,
            timeout=2
        )
        
        if result.returncode == 0:
            lines = result.stdout.strip().split("\n")
            if len(lines) > 1:
                print(f"✗ Port {port} is in use by:")
                for line in lines[1:]:
                    print(f"    {line}")
                return True
        
        print(f"✓ Port {port} is not in use by other processes")
        return False
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print(f"⚠ Could not check if port is in use (lsof not available or timeout)")
        return False


def check_latency_timer(port: str):
    """Check USB latency timer setting."""
    try:
        # Extract the ttyUSB device name
        result = subprocess.run(
            ["readlink", "-f", port],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        if result.returncode == 0:
            real_port = result.stdout.strip()
            tty_device = os.path.basename(real_port)
            
            latency_path = f"/sys/bus/usb-serial/devices/{tty_device}/latency_timer"
            
            if os.path.exists(latency_path):
                with open(latency_path, 'r') as f:
                    latency = int(f.read().strip())
                
                if latency == 1:
                    print(f"✓ Latency timer is optimal (1 ms)")
                else:
                    print(f"⚠ Latency timer is {latency} ms (recommended: 1 ms)")
                    print(f"  To fix: echo 1 | sudo tee {latency_path}")
            else:
                print(f"⚠ Could not find latency timer setting")
    except Exception as e:
        print(f"⚠ Could not check latency timer: {e}")


def test_port_open(port: str, baudrate: int) -> Optional[PortHandler]:
    """Test if we can open the port."""
    print(f"\nTesting port connection...")
    print(f"  Port: {port}")
    print(f"  Baudrate: {baudrate}")
    
    port_handler = PortHandler(port)
    
    try:
        if port_handler.openPort():
            print(f"✓ Successfully opened port")
        else:
            print(f"✗ Failed to open port")
            return None
        
        if port_handler.setBaudRate(baudrate):
            print(f"✓ Successfully set baudrate to {baudrate}")
        else:
            print(f"✗ Failed to set baudrate to {baudrate}")
            port_handler.closePort()
            return None
        
        return port_handler
    
    except Exception as e:
        print(f"✗ Exception while opening port: {e}")
        return None


def scan_for_motors(port_handler: PortHandler, max_id: int = 20) -> List[int]:
    """Scan for Dynamixel motors on the bus."""
    print(f"\nScanning for motors (IDs 1-{max_id})...")
    print("This may take a moment...")
    
    packet_handler = PacketHandler(2.0)  # Protocol 2.0
    found_ids = []
    
    # Try to ping each ID
    for motor_id in range(1, max_id + 1):
        # Try to read model number (address 0, length 2)
        model_number, result, error = packet_handler.ping(port_handler, motor_id)
        
        if result == COMM_SUCCESS:
            print(f"  ✓ Found motor at ID {motor_id} (Model: {model_number})")
            found_ids.append(motor_id)
        
        # Small delay to avoid overwhelming the bus
        time.sleep(0.01)
    
    if not found_ids:
        print(f"  ✗ No motors found")
        print(f"\nPossible reasons:")
        print(f"  1. Motors are not powered")
        print(f"  2. Wrong baudrate (try --baudrate 1000000 or --baudrate 115200)")
        print(f"  3. Motor IDs are outside the scanned range (use --max-id to increase)")
        print(f"  4. Cable connection issues")
        print(f"  5. Motors are configured for Protocol 1.0 (this tool uses Protocol 2.0)")
    else:
        print(f"\n✓ Found {len(found_ids)} motor(s): {found_ids}")
    
    return found_ids


def read_motor_info(port_handler: PortHandler, motor_id: int):
    """Read detailed information from a motor."""
    print(f"\nReading information from motor ID {motor_id}...")
    
    packet_handler = PacketHandler(2.0)
    
    # Read various control table values
    info_to_read = [
        ("Model Number", 0, 2),
        ("Firmware Version", 6, 1),
        ("Baud Rate", 8, 1),
        ("Operating Mode", 11, 1),
        ("Torque Enable", 64, 1),
        ("Present Position", 132, 4),
        ("Present Temperature", 146, 1),
    ]
    
    for name, addr, length in info_to_read:
        if length == 1:
            value, result, error = packet_handler.read1ByteTxRx(port_handler, motor_id, addr)
        elif length == 2:
            value, result, error = packet_handler.read2ByteTxRx(port_handler, motor_id, addr)
        elif length == 4:
            value, result, error = packet_handler.read4ByteTxRx(port_handler, motor_id, addr)
        else:
            continue
        
        if result == COMM_SUCCESS:
            # Convert position to degrees if applicable
            if name == "Present Position":
                degrees = (value - 2048) * 360 / 4096
                print(f"  {name}: {value} (≈ {degrees:.1f}°)")
            else:
                print(f"  {name}: {value}")
        else:
            print(f"  {name}: Failed to read (error: {result})")


def main(args: Args):
    print("=" * 60)
    print("Dynamixel Connection Diagnostic Tool")
    print("=" * 60)
    
    # Step 1: Find port
    if args.port is None:
        print("\n[1] Detecting FTDI ports...")
        ports = find_ftdi_ports()
        
        if not ports:
            print("✗ No FTDI ports found")
            print("  Make sure your GELLO device is connected via USB")
            return 1
        elif len(ports) == 1:
            port = ports[0]
            print(f"✓ Found FTDI port: {port}")
        else:
            print(f"✓ Found {len(ports)} FTDI ports:")
            for i, p in enumerate(ports):
                print(f"  {i+1}: {p}")
            
            try:
                choice = int(input("\nSelect port number: ")) - 1
                if 0 <= choice < len(ports):
                    port = ports[choice]
                else:
                    print("Invalid choice")
                    return 1
            except (ValueError, KeyboardInterrupt):
                print("\nCancelled")
                return 1
    else:
        port = args.port
        print(f"\n[1] Using specified port: {port}")
    
    # Step 2: Check port status
    print(f"\n[2] Checking port status...")
    if not check_port_permissions(port):
        return 1
    
    in_use = check_port_in_use(port)
    if in_use:
        print("\n⚠ Warning: Port is in use. Close other programs using this port.")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            return 1
    
    check_latency_timer(port)
    
    # Step 3: Test port connection
    print(f"\n[3] Testing port connection...")
    port_handler = test_port_open(port, args.baudrate)
    
    if port_handler is None:
        print("\n✗ Failed to open port")
        return 1
    
    # Step 4: Scan for motors
    if args.scan_ids:
        print(f"\n[4] Scanning for motors...")
        found_ids = scan_for_motors(port_handler, args.max_id)
        
        # Read detailed info for found motors
        if found_ids and len(found_ids) <= 5:
            for motor_id in found_ids[:5]:
                read_motor_info(port_handler, motor_id)
    
    # Cleanup
    port_handler.closePort()
    
    print("\n" + "=" * 60)
    print("Diagnostic complete!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main(tyro.cli(Args)))

