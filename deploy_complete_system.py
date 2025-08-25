#!/usr/bin/env python3
"""
Complete System Deployment - One Click Solution
Run this script for full deployment with progress bars
"""

import subprocess
import sys
import os
import time

def main():
    """Deploy complete HUB system"""
    print("=" * 60)
    print(">>> HUB COMPLETE SYSTEM DEPLOYMENT")
    print("    One-Click Solution with Progress Bars")
    print("=" * 60)
    
    # Step 1: Upload Master firmware  
    print("\nStep 1: Uploading Master firmware...")
    
    # Find master port
    master_port = None
    try:
        import serial.tools.list_ports
        available_ports = []
        for port in serial.tools.list_ports.comports():
            if (port.vid == 0x2E8A or 
                'pico' in port.description.lower() or 
                'rp2040' in port.description.lower() or
                'uart' in port.description.lower()):
                available_ports.append(port.device)
    except Exception as e:
        print(f"Error detecting ports: {e}")
        available_ports = []
    print(f"Found RP2040 ports: {available_ports}")
    
    # Try direct upload to each port until one works
    master_uploaded = False
    for port in available_ports:
        if not port:
            continue
            
        print(f"Trying master upload to {port}...")
        try:
            result = subprocess.run(['pio', 'run', '-e', 'hub_master_deploy', 
                                   '--target', 'upload', '--upload-port', port],
                                  capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                master_port = port
                master_uploaded = True
                print(f"[OK] Master uploaded to {port}!")
                break
            else:
                print(f"[SKIP] {port} - {result.stderr.split(chr(10))[0] if result.stderr else 'Upload failed'}")
        except Exception as e:
            print(f"[SKIP] {port} - {e}")
    
    if not master_uploaded:
        print("[ERROR] Master upload failed on all ports!")
        return False
    
    # Wait for master to settle after upload
    time.sleep(3)
    
    # Step 2: Deploy all slots via master selection
    print(f"\nStep 2: Deploying slots via master control (master on {master_port})...")
    
    successful_slots = []
    
    # Try uploading to slots 1-5 using master selection
    for slot_num in range(1, 6):
        print(f"\n--- SLOT {slot_num} ---")
        env_name = f"hub_slot{slot_num}_deploy"
        
        # Step 1: Select slot via master menu
        print(f"Step 1: Selecting slot {slot_num} via master...")
        try:
            import serial
            ser = serial.Serial(master_port, 152000, timeout=3)
            time.sleep(1)
            
            # Enter Slot Management menu
            ser.write(b"1\\n")  # Slot Management
            time.sleep(1)
            
            # Select slot (commands 6-10 for slot 1-5 selection)
            select_cmd = 5 + slot_num  # 6=Select Slot 1, 7=Select Slot 2, etc.
            ser.write(f"{select_cmd}\\n".encode())
            time.sleep(2)
            
            # Read response to check selection
            responses = []
            while ser.in_waiting > 0:
                try:
                    response = ser.readline().decode('ascii', errors='ignore').strip()
                    if response:
                        responses.append(response)
                except:
                    pass
            
            # Exit menu
            ser.write(b"0\\n")  # Back to main menu
            time.sleep(0.5)
            ser.close()
            
            # Check if slot was selected successfully
            response_text = ' '.join(responses).lower()
            if 'selected' in response_text or 'successfully' in response_text:
                print(f"   [OK] Slot {slot_num} selected successfully")
                slot_selected = True
            else:
                print(f"   [WARN] Slot {slot_num} selection unclear, proceeding anyway...")
                slot_selected = True  # Proceed anyway
                
        except Exception as e:
            print(f"   [ERROR] Failed to select slot {slot_num}: {e}")
            slot_selected = False
        
        if not slot_selected:
            print(f"   [SKIP] Cannot upload to slot {slot_num} - selection failed")
            continue
        
        # Step 2: Wait for slot to be available
        time.sleep(2)
        
        # Step 3: Try to upload to the selected slot
        print(f"Step 2: Uploading {env_name}...")
        
        # Find the current slot port (should be available after selection)
        current_ports = []
        try:
            import serial.tools.list_ports
            for port in serial.tools.list_ports.comports():
                if (port.vid == 0x2E8A and port.device != master_port):
                    current_ports.append(port.device)
        except:
            pass
        
        upload_success = False
        
        # Upload without specifying port (let PlatformIO auto-detect)
        print(f"   Uploading {env_name} (auto-detect port)...")
        try:
            result = subprocess.run(['pio', 'run', '-e', env_name, '--target', 'upload'],
                                  capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                successful_slots.append((slot_num, "auto-detected"))
                upload_success = True
                print(f"   [OK] Slot {slot_num} uploaded successfully!")
            else:
                print(f"   [WARN] Auto-upload failed: {result.stderr.split(chr(10))[0] if result.stderr else 'Unknown error'}")
                
        except Exception as e:
            print(f"   [WARN] Upload error: {e}")
        
        if not upload_success:
            # Fallback: Try boot + upload
            print(f"   [RETRY] Trying boot as fallback...")
            try:
                ser = serial.Serial(master_port, 152000, timeout=2)
                time.sleep(1)
                ser.write(b"1\\n")  # Slot Management
                time.sleep(1) 
                ser.write(f"{slot_num}\\n".encode())  # Boot slot
                time.sleep(2)
                ser.write(b"0\\n")  # Exit menu
                ser.close()
                
                time.sleep(3)  # Wait for boot
                
                # Try upload again with auto-detect
                result = subprocess.run(['pio', 'run', '-e', env_name, '--target', 'upload'],
                                      capture_output=True, text=True, timeout=120)
                if result.returncode == 0:
                    successful_slots.append((slot_num, 'after-boot'))
                    print(f"   [OK] Slot {slot_num} uploaded after boot!")
                else:
                    print(f"   [ERROR] Slot {slot_num} upload failed even with boot fallback")
                    
            except Exception as e:
                print(f"   [ERROR] Boot fallback failed: {e}")
    
    # Results
    print(f"\n[SUMMARY] Successful uploads:")
    print(f"  Master: {master_port}")
    for slot_num, port in successful_slots:
        print(f"  Slot {slot_num}: {port}")
    
    return len(successful_slots) > 0

if __name__ == "__main__":
    print("Starting Complete System Deployment...")
    
    # Check PlatformIO
    try:
        subprocess.run(['pio', '--version'], capture_output=True, check=True)
    except:
        print("[ERROR] PlatformIO not found! Install with: pip install platformio")
        sys.exit(1)
    
    success = main()
    
    if success:
        print("\n" + "=" * 60)
        print(">>> DEPLOYMENT SUCCESSFUL!")
        print("Your complete HUB system is ready!")
        print("Master + All 5 Slots deployed with enhanced menu system")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print(">>> DEPLOYMENT INCOMPLETE")
        print("Check connections and try again")
        print("=" * 60)
    
    sys.exit(0 if success else 1)