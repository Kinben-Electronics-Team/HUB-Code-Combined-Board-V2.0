#!/usr/bin/env python3
"""
Force BOOTSEL mode upload helper
Automatically puts device into BOOTSEL mode and uploads firmware
"""

import subprocess
import sys
import time
import serial.tools.list_ports

def find_rp2040_devices():
    """Find all RP2040 devices"""
    devices = []
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if (port.vid == 0x2E8A or  # Raspberry Pi Foundation VID
            'pico' in port.description.lower() or 
            'rp2040' in port.description.lower() or
            'uart' in port.description.lower()):
            devices.append(port.device)
    
    return devices

def force_bootsel_and_upload(environment):
    """Force device into BOOTSEL mode and upload"""
    print(f"🔄 Forcing device into BOOTSEL mode for {environment}...")
    
    # Find devices
    devices = find_rp2040_devices()
    if not devices:
        print("❌ No RP2040 devices found!")
        return False
    
    print(f"📱 Found devices: {devices}")
    
    # Try to force each device into BOOTSEL mode
    for device in devices:
        try:
            print(f"🔧 Attempting to force {device} into BOOTSEL mode...")
            result = subprocess.run(['picotool', 'reboot', '-f', '-u', device], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print(f"✅ Successfully forced {device} into BOOTSEL mode")
                break
        except subprocess.TimeoutExpired:
            print(f"⏰ Timeout forcing {device} - trying next device")
            continue
        except Exception as e:
            print(f"⚠️  Error with {device}: {e}")
            continue
    
    # Wait for device to enter BOOTSEL mode
    print("⏳ Waiting for BOOTSEL mode...")
    time.sleep(3)
    
    # Now try the upload
    print(f"🚀 Uploading {environment}...")
    try:
        result = subprocess.run(['pio', 'run', '-e', environment, '-t', 'upload'], 
                              check=False, timeout=120)
        if result.returncode == 0:
            print(f"✅ {environment} uploaded successfully!")
            return True
        else:
            print(f"❌ {environment} upload failed!")
            return False
    except Exception as e:
        print(f"❌ Upload error: {e}")
        return False

def main():
    """Main function"""
    if len(sys.argv) != 2:
        print("Usage: python force_bootsel_upload.py <environment>")
        print("Example: python force_bootsel_upload.py hub_master_deploy")
        sys.exit(1)
    
    environment = sys.argv[1]
    
    # Check if picotool is available
    try:
        subprocess.run(['picotool', 'version'], capture_output=True, check=True)
    except:
        print("❌ picotool not found! Please install picotool")
        sys.exit(1)
    
    success = force_bootsel_and_upload(environment)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()