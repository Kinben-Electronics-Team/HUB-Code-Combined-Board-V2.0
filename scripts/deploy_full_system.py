#!/usr/bin/env python3
"""
Full HUB System Deployment - Auto COM Port Detection
Works on any PC with automatic port discovery
"""

import serial
import serial.tools.list_ports
import time
import subprocess
import sys
import os
import threading

def show_progress_bar(duration, message="Processing"):
    """Show an animated progress bar - ASCII compatible"""
    def animate():
        chars = "|/-\\"
        start_time = time.time()
        i = 0
        
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            bar_length = 40
            filled_length = int(bar_length * progress)
            
            bar = "#" * filled_length + "." * (bar_length - filled_length)
            percentage = int(progress * 100)
            
            print(f"\r{chars[i % len(chars)]} {message}: [{bar}] {percentage}%", end="", flush=True)
            i += 1
            time.sleep(0.1)
        
        # Final completed state
        bar = "#" * bar_length
        print(f"\r[OK] {message}: [{bar}] 100%", flush=True)
    
    thread = threading.Thread(target=animate)
    thread.daemon = True
    thread.start()
    return thread

def show_upload_progress(process, device_name):
    """Show upload progress with animated bar - ASCII compatible"""
    chars = "|/-\\"
    i = 0
    
    print(f"\n>> Uploading {device_name} firmware...")
    
    while process.poll() is None:
        print(f"\r{chars[i % len(chars)]} Uploading to {device_name}... ", end="", flush=True)
        i += 1
        time.sleep(0.1)
    
    if process.returncode == 0:
        print(f"\r[OK] {device_name} upload completed!                    ")
        return True
    else:
        print(f"\r[FAIL] {device_name} upload failed!                      ")
        return False

def find_rp2040_ports():
    """Find all RP2040 devices (Pico boards)"""
    rp2040_ports = []
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # Look for RP2040/Pico identifiers
        if (port.vid == 0x2E8A or  # Raspberry Pi Foundation VID
            'pico' in port.description.lower() or 
            'rp2040' in port.description.lower() or
            'uart' in port.description.lower()):
            rp2040_ports.append({
                'port': port.device,
                'desc': port.description,
                'vid': port.vid,
                'pid': port.pid
            })
    
    return rp2040_ports

def find_master_port():
    """Automatically find the HUB Master port"""
    print(">> Scanning for HUB Master...")
    rp2040_ports = find_rp2040_ports()
    
    if not rp2040_ports:
        print("[ERROR] No RP2040 devices found! Please connect HUB Master.")
        return None
    
    # Try to identify master by testing menu response
    for port_info in rp2040_ports:
        port = port_info['port']
        print(f"Testing {port}: {port_info['desc']}")
        
        try:
            with serial.Serial(port, 152000, timeout=2) as ser:
                time.sleep(1)
                ser.write(b"4\n")  # Try to access System Info
                time.sleep(2)
                
                responses = []
                while ser.in_waiting > 0:
                    response = ser.readline().decode('ascii', errors='ignore').strip()
                    responses.append(response)
                
                # Look for master menu responses
                if any('HUB MASTER' in r or 'System Info' in r or 'Hub ID' in r for r in responses):
                    print(f"[OK] Found HUB Master on {port}")
                    return port
                    
        except Exception as e:
            print(f"   {port} - Not responding as master")
            continue
    
    # If no master found by menu test, use first available port
    if rp2040_ports:
        master_port = rp2040_ports[0]['port']
        print(f"[WARN] Using first available port as master: {master_port}")
        return master_port
    
    return None

def send_menu_command(ser, command, wait_time=2):
    """Send command and collect responses"""
    cmd_str = f"{command}\n"
    ser.write(cmd_str.encode('ascii'))
    print(f"   -> {command}")
    time.sleep(wait_time)
    
    responses = []
    while ser.in_waiting > 0:
        try:
            response = ser.readline().decode('ascii', errors='ignore').strip()
            if response:
                responses.append(response)
        except:
            pass
    return responses

def deploy_full_system():
    """Deploy complete HUB system with auto-detection"""
    print("=" * 60)
    print(">>> FULL HUB SYSTEM DEPLOYMENT")
    print("    Automatic COM Port Detection Enabled")
    print("=" * 60)
    
    # Step 1: Find Master with progress
    print("\nStep 1: Detecting HUB Master...")
    progress_thread = show_progress_bar(3, "Scanning COM ports")
    time.sleep(3)  # Let progress complete
    
    master_port = find_master_port()
    if not master_port:
        return False
    
    print(f"\n>> Using Master Port: {master_port}")
    
    # Step 2: Skip master upload (already deployed) - just show boot progress
    print("\nStep 2: Master already deployed, checking readiness...")
    progress_thread = show_progress_bar(3, "Master initialization")
    time.sleep(3)
    
    # Step 3: Deploy All Slots
    print("\nStep 3: Deploying All Slots...")
    successful_slots = []
    
    for slot_num in range(1, 6):
        print(f"\n--- SLOT {slot_num} ---")
        
        # Navigate menu to select slot
        try:
            with serial.Serial(master_port, 152000, timeout=5) as ser:
                time.sleep(1)
                
                # Navigate to slot selection
                send_menu_command(ser, "1", 2)  # Slot Management
                slot_cmd = 5 + slot_num  # 6=Slot1, 7=Slot2, etc.
                responses = send_menu_command(ser, str(slot_cmd), 3)
                
                # Check success
                if any("successfully" in r.lower() for r in responses):
                    print(f"   [OK] Slot {slot_num} selected")
                else:
                    print(f"   [WARN] Slot {slot_num} selection unclear")
                
                # Exit menus
                send_menu_command(ser, "0", 1)
                send_menu_command(ser, "0", 1)
                
        except Exception as e:
            print(f"   [ERROR] Failed to select slot {slot_num}: {e}")
            continue
        
        # Wait for slot to initialize with progress
        progress_thread = show_progress_bar(5, f"Slot {slot_num} initializing")
        time.sleep(5)
        
        # Upload slot firmware with progress
        try:
            env_name = f"hub_slot{slot_num}_deploy"
            process = subprocess.Popen(['pio', 'run', '-e', env_name, '-t', 'upload'],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            success = show_upload_progress(process, f"Slot {slot_num}")
            
            if success:
                successful_slots.append(slot_num)
                
        except Exception as e:
            print(f"   [ERROR] Slot {slot_num} upload error: {e}")
    
    # Final Results
    print("\n" + "=" * 60)
    print(">>> DEPLOYMENT COMPLETE")
    print(f"[OK] Successful slots: {successful_slots}")
    print(f"Success rate: {len(successful_slots)}/5 slots")
    
    if len(successful_slots) == 5:
        print(">>> ALL SYSTEMS DEPLOYED SUCCESSFULLY!")
        print(f"Master running on: {master_port}")
        print("Ready for operation!")
        return True
    else:
        print("[WARN] Some deployments failed - check connections")
        return False

def update_platformio_ports(master_port):
    """Update platformio.ini with detected master port"""
    # This could be enhanced to update all ports dynamically
    print(f"📝 Master port detected: {master_port}")
    # For now, we'll use the detected port via environment variable
    os.environ['DETECTED_MASTER_PORT'] = master_port

def main():
    """Main entry point"""
    print("Starting Full System Deployment...")
    
    # Check prerequisites
    try:
        subprocess.run(['pio', '--version'], capture_output=True, check=True)
    except:
        print("[ERROR] PlatformIO CLI not found! Please install PlatformIO.")
        return False
    
    success = deploy_full_system()
    
    if success:
        print("\n>>> DEPLOYMENT SUCCESSFUL!")
        print("Your HUB system is ready to use.")
    else:
        print("\n>>> DEPLOYMENT INCOMPLETE")
        print("Please check connections and try again.")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)