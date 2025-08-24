#!/usr/bin/env python3
"""
Updated HUB deployment script for new menu system
"""

import serial
import time
import subprocess
import sys

# Configuration
MASTER_PORT = 'COM13'
MASTER_BAUD = 152000

def send_menu_command(ser, command, wait_time=2):
    """Send a command and wait for response"""
    cmd_str = f"{command}\n"
    ser.write(cmd_str.encode('ascii'))
    print(f"Sent: {command}")
    time.sleep(wait_time)
    
    responses = []
    while ser.in_waiting > 0:
        try:
            response = ser.readline().decode('ascii', errors='ignore').strip()
            if response:
                print(f"Master: {response}")
                responses.append(response)
        except:
            pass
    return responses

def select_slot_via_menu(slot_num):
    """Navigate menu to select a slot"""
    try:
        with serial.Serial(MASTER_PORT, MASTER_BAUD, timeout=5) as ser:
            print(f"Connected to master on {MASTER_PORT}")
            time.sleep(1)
            
            # Step 1: Enter Slot Management (option 1)
            print("Entering Slot Management menu...")
            send_menu_command(ser, "1", 2)
            
            # Step 2: Select specific slot (6=Slot1, 7=Slot2, 8=Slot3, 9=Slot4, 10=Slot5)
            slot_command = 5 + slot_num  # Convert slot 1->6, slot 2->7, etc.
            print(f"Selecting Slot {slot_num} (menu option {slot_command})...")
            responses = send_menu_command(ser, str(slot_command), 3)
            
            # Check for success response
            success = any("Command executed successfully" in r for r in responses)
            
            # Exit back to main menu
            send_menu_command(ser, "0", 1)
            send_menu_command(ser, "0", 1)
            
            return success
            
    except Exception as e:
        print(f"Error: {e}")
        return False

def upload_slot_firmware(slot_num):
    """Upload firmware to a specific slot"""
    env_name = f"hub_slot{slot_num}_deploy"
    
    print(f"Uploading slot {slot_num} firmware...")
    
    cmd = ["pio", "run", "-e", env_name, "-t", "upload"]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        
        if result.returncode == 0:
            print(f"SUCCESS: Slot {slot_num} uploaded!")
            return True
        else:
            print(f"FAILED: Slot {slot_num} upload failed:")
            print(result.stderr[-500:])
            return False
            
    except Exception as e:
        print(f"Error uploading slot {slot_num}: {e}")
        return False

def main():
    print("Updated HUB Deployment Process")
    print("=" * 40)
    
    successful_slots = []
    
    for slot_num in range(1, 6):  # Slots 1-5
        print(f"\n--- SLOT {slot_num} ---")
        
        # Step 1: Navigate menu to select slot
        print(f"Step 1: Selecting slot {slot_num} via menu...")
        if not select_slot_via_menu(slot_num):
            print(f"Failed to select slot {slot_num}")
            continue
        
        # Step 2: Wait for slot to initialize and appear as COM port
        print("Step 2: Waiting for slot to initialize...")
        time.sleep(5)
        
        # Step 3: Upload slot firmware
        print(f"Step 3: Uploading firmware to slot {slot_num}...")
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
    
    return len(successful_slots) == 5

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)