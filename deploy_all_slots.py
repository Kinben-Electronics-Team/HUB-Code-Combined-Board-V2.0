#!/usr/bin/env python3
"""
Complete HUB deployment automation script
Handles master communication and slot firmware uploads
"""

import serial
import time
import subprocess
import sys
import os

# Configuration
MASTER_PORT = 'COM13'
MASTER_BAUD = 152000
UPLOAD_TIMEOUT = 30

def send_command_to_master(command):
    """Send a command to the master controller"""
    try:
        with serial.Serial(MASTER_PORT, MASTER_BAUD, timeout=2) as ser:
            print(f"Connected to master on {MASTER_PORT}")
            time.sleep(1)  # Let connection stabilize
            
            # Send command
            cmd_str = f"{command}\n"
            ser.write(cmd_str.encode())
            print(f"Sent command: {command}")
            
            # Read response
            time.sleep(2)
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"Master response: {response}")
                    
            return True
    except Exception as e:
        print(f"✗ Error communicating with master: {e}")
        return False

def upload_slot_firmware(slot_num, port_expected):
    """Upload firmware to a specific slot"""
    env_name = f"hub_slot{slot_num}_deploy"
    
    print(f"\n🔄 Uploading slot {slot_num} firmware...")
    print(f"   Environment: {env_name}")
    print(f"   Expected port: {port_expected}")
    
    # Build upload command
    cmd = ["pio", "run", "-e", env_name, "-t", "upload"]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=UPLOAD_TIMEOUT)
        
        if result.returncode == 0:
            print(f"✅ Slot {slot_num} uploaded successfully!")
            return True
        else:
            print(f"❌ Slot {slot_num} upload failed:")
            print(result.stderr[-500:])  # Show last 500 chars of error
            return False
            
    except subprocess.TimeoutExpired:
        print(f"⏱️ Slot {slot_num} upload timed out")
        return False
    except Exception as e:
        print(f"❌ Error uploading slot {slot_num}: {e}")
        return False

def wait_for_port_ready(port, timeout=10):
    """Wait for a COM port to become available"""
    print(f"⏳ Waiting for {port} to become ready...")
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        try:
            # Try to open the port briefly to check if it's ready
            with serial.Serial(port, 9600, timeout=0.1):
                pass
            print(f"✓ {port} is ready")
            return True
        except:
            time.sleep(0.5)
    
    print(f"⏱️ Timeout waiting for {port}")
    return False

def main():
    """Main deployment function"""
    print("Starting Complete HUB Deployment Process")
    print("=" * 50)
    
    # Slot configuration: (command, expected_port, slot_number)
    slots = [
        (17, 'COM14', 1),
        (18, 'COM15', 2), 
        (19, 'COM16', 3),
        (20, 'COM17', 4),
        (21, 'COM18', 5)
    ]
    
    successful_slots = []
    failed_slots = []
    
    print(f"📡 Master controller should be running on {MASTER_PORT}")
    
    for select_cmd, expected_port, slot_num in slots:
        print(f"\n{'='*20} SLOT {slot_num} {'='*20}")
        
        # Step 1: Send selection command to master
        print(f"1️⃣ Selecting slot {slot_num} (command {select_cmd})...")
        if not send_command_to_master(select_cmd):
            print(f"❌ Failed to communicate with master for slot {slot_num}")
            failed_slots.append(slot_num)
            continue
        
        # Step 2: Wait for the COM port to appear/become ready
        print(f"2️⃣ Waiting for {expected_port} to become available...")
        time.sleep(3)  # Give time for slot to initialize
        
        if not wait_for_port_ready(expected_port, timeout=15):
            print(f"❌ {expected_port} not ready for slot {slot_num}")
            failed_slots.append(slot_num)
            continue
        
        # Step 3: Upload slot firmware
        print(f"3️⃣ Uploading firmware to slot {slot_num}...")
        if upload_slot_firmware(slot_num, expected_port):
            successful_slots.append(slot_num)
            print(f"✅ Slot {slot_num} deployment complete!")
        else:
            failed_slots.append(slot_num)
        
        # Brief pause between slots
        time.sleep(2)
    
    # Summary
    print(f"\n{'='*50}")
    print("📊 DEPLOYMENT SUMMARY")
    print(f"✅ Successful slots: {successful_slots}")
    print(f"❌ Failed slots: {failed_slots}")
    print(f"📈 Success rate: {len(successful_slots)}/5 slots")
    
    if len(successful_slots) == 5:
        print("\n🎉 ALL SLOTS DEPLOYED SUCCESSFULLY!")
        
        # Send a test command sequence
        print("\n🧪 Running quick test sequence...")
        test_commands = ['?', 'status']
        for cmd in test_commands:
            send_command_to_master(cmd)
            time.sleep(1)
            
    else:
        print(f"\n⚠️ Some slots failed. You may need to manually retry failed slots.")
        
    return len(successful_slots) == 5

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)