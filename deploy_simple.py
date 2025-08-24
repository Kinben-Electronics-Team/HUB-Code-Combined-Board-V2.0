#!/usr/bin/env python3
"""
Simple HUB deployment automation script
"""

import serial
import time
import subprocess
import sys

# Configuration
MASTER_PORT = 'COM13'
MASTER_BAUD = 152000

def send_command_to_master(command):
    """Send a command to the master controller"""
    try:
        with serial.Serial(MASTER_PORT, MASTER_BAUD, timeout=2) as ser:
            print(f"Connected to master on {MASTER_PORT}")
            time.sleep(1)
            
            cmd_str = f"{command}\n"
            ser.write(cmd_str.encode())
            print(f"Sent command: {command}")
            
            time.sleep(2)
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"Master response: {response}")
                    
            return True
    except Exception as e:
        print(f"Error communicating with master: {e}")
        return False

def upload_slot_firmware(slot_num):
    """Upload firmware to a specific slot"""
    env_name = f"hub_slot{slot_num}_deploy"
    
    print(f"Uploading slot {slot_num} firmware...")
    
    cmd = ["pio", "run", "-e", env_name, "-t", "upload"]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        
        if result.returncode == 0:
            print(f"Slot {slot_num} uploaded successfully!")
            return True
        else:
            print(f"Slot {slot_num} upload failed:")
            print(result.stderr[-300:])
            return False
            
    except Exception as e:
        print(f"Error uploading slot {slot_num}: {e}")
        return False

def main():
    print("Starting HUB Deployment Process")
    print("=" * 40)
    
    # Slot configuration: (command, slot_number)
    slots = [(17, 1), (18, 2), (19, 3), (20, 4), (21, 5)]
    
    successful_slots = []
    
    for select_cmd, slot_num in slots:
        print(f"\n--- SLOT {slot_num} ---")
        
        # Send selection command to master
        print(f"Selecting slot {slot_num} (command {select_cmd})...")
        if not send_command_to_master(select_cmd):
            print(f"Failed to communicate with master for slot {slot_num}")
            continue
        
        # Wait for slot to initialize
        print("Waiting for slot to initialize...")
        time.sleep(5)
        
        # Upload slot firmware
        if upload_slot_firmware(slot_num):
            successful_slots.append(slot_num)
        
        time.sleep(2)
    
    # Summary
    print(f"\n" + "=" * 40)
    print("DEPLOYMENT SUMMARY")
    print(f"Successful slots: {successful_slots}")
    print(f"Success rate: {len(successful_slots)}/5 slots")
    
    if len(successful_slots) == 5:
        print("ALL SLOTS DEPLOYED SUCCESSFULLY!")
        
        # Send test commands
        print("Running test commands...")
        send_command_to_master('?')
        time.sleep(1)
        send_command_to_master('status')
        
    return len(successful_slots) == 5

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)