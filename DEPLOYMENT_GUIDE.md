# HUB Deployment Guide

## Quick Start

### Prerequisites
- Python 3.x installed
- PlatformIO CLI installed
- pyserial library: `pip install pyserial`
- Hardware connected and powered

### One-Command Deployment

```bash
python deploy_simple.py
```

This single command will:
1. Upload master firmware to COM13
2. Communicate with master to select each slot
3. Upload slot firmware to dynamically created ports (COM14-COM18)
4. Test the system
5. Report results

## Step-by-Step Manual Process

If you prefer manual control or need to troubleshoot:

### 1. Upload Master
```bash
pio run -e hub_master_deploy -t upload
```

### 2. Manual Slot Deployment
For each slot, repeat this process:

```bash
# Connect to master (use your preferred serial terminal)
# Send command to select slot (17 for slot 1, 18 for slot 2, etc.)
17

# Wait for new COM port to appear, then upload
pio run -e hub_slot1_deploy -t upload
```

### 3. Verify Deployment
Connect to master and test:
```
? → Show menu
status → Show system status
0-4 → Boot slots
17-21 → Select slots
```

## Build Commands Reference

### Individual Environment Builds
```bash
# Master only
pio run -e hub_master_deploy

# Specific slot
pio run -e hub_slot1_deploy
pio run -e hub_slot2_deploy
pio run -e hub_slot3_deploy
pio run -e hub_slot4_deploy
pio run -e hub_slot5_deploy

# All environments
pio run
```

### Upload Commands
```bash
# Master
pio run -e hub_master_deploy -t upload

# Slots (after selection via master commands)
pio run -e hub_slot1_deploy -t upload  # COM14
pio run -e hub_slot2_deploy -t upload  # COM15
pio run -e hub_slot3_deploy -t upload  # COM16
pio run -e hub_slot4_deploy -t upload  # COM17
pio run -e hub_slot5_deploy -t upload  # COM18
```

## Port Mapping

| Device | COM Port | Environment | Selection Command |
|--------|----------|-------------|-------------------|
| Master | COM13 | hub_master_deploy | N/A |
| Slot 1 | COM14 | hub_slot1_deploy | 17 |
| Slot 2 | COM15 | hub_slot2_deploy | 18 |
| Slot 3 | COM16 | hub_slot3_deploy | 19 |
| Slot 4 | COM17 | hub_slot4_deploy | 20 |
| Slot 5 | COM18 | hub_slot5_deploy | 21 |

## Serial Communication Settings

- **Master (COM13)**: 152000 baud, 8N1
- **Slots (COM14-18)**: 152000 baud, 8N1

## Success Indicators

### Master Upload Success
```
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK
The device was rebooted to start the application.
========================= [SUCCESS] Took X.XX seconds =========================
```

### Slot Selection Success  
Master should respond with:
```
cmd = 17  (or 18, 19, 20, 21)
```

### Slot Upload Success
```
Slot X uploaded successfully!
```

### Full Deployment Success
```
DEPLOYMENT SUMMARY
Successful slots: [1, 2, 3, 4, 5]
Success rate: 5/5 slots
ALL SLOTS DEPLOYED SUCCESSFULLY!
```

## Troubleshooting Quick Fixes

### Device Not in BOOTSEL Mode
1. Hold BOOTSEL button while plugging USB
2. Or reset device while holding BOOTSEL

### No Response from Master
1. Check COM13 is correct port
2. Verify baud rate (152000)
3. Press reset button on master

### Slot Not Appearing
1. Send selection command first (17-21)
2. Wait 5 seconds for port to stabilize
3. Check Device Manager for new ports

### Upload Timeouts
1. Ensure device is in BOOTSEL mode
2. Try manual reset before upload
3. Check USB cable connection

## Environment Variables (Optional)

You can customize the deployment script by setting these environment variables:

```bash
# Custom master port
set MASTER_PORT=COM15

# Custom baud rate  
set MASTER_BAUD=115200

# Run deployment
python deploy_simple.py
```

## Production Deployment Checklist

- [ ] All 6 environments build without errors
- [ ] Master firmware uploads successfully
- [ ] Master responds to serial commands
- [ ] All 5 slot selection commands work (17-21)  
- [ ] All 5 slot firmwares upload successfully
- [ ] System responds to test commands (?, status)
- [ ] Hardware connections verified
- [ ] Power supply stable

## Emergency Recovery

If deployment fails partially:

### Re-deploy Individual Slots
```bash
# Connect to master, send selection command
17  # Select slot 1

# Upload just that slot
pio run -e hub_slot1_deploy -t upload
```

### Full System Reset
```bash
# Re-upload master
pio run -e hub_master_deploy -t upload

# Re-run full deployment
python deploy_simple.py
```

### Manual Recovery Mode
If automated script fails, use manual step-by-step process described above.

---

**Quick Reference Card**

```
Deploy All:     python deploy_simple.py
Build All:      pio run  
Upload Master:  pio run -e hub_master_deploy -t upload
Monitor:        pio device monitor -p COM13 -b 152000
Test Commands:  ?, status, 17-21, 0-4
```