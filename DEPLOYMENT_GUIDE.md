# HUB Deployment Guide

## Quick Start

### Prerequisites
- Python 3.x installed
- PlatformIO CLI installed
- pyserial library: `pip install pyserial`
- Hardware connected and powered

## 🚀 CURRENT STATUS - WORKING DEPLOYMENT

### ✅ What Works Right Now:

#### Complete System Deployment (RECOMMENDED)
```bash
python deploy_complete_system.py
```
**Status**: ✅ WORKING - Uploads master + all 5 slots with beautiful progress bars
**Time**: ~6-8 minutes total
**Features**: Auto COM detection, progress animations, menu navigation

#### Slot-Only Deployment  
```bash
python deploy_slots.py
```
**Status**: ✅ WORKING - Uses existing deployed menu system
**Time**: ~5-6 minutes for all slots

#### Individual Slot Upload
```bash
pio run -e hub_slot1_deploy -t upload  # etc for slots 2-5
```
**Status**: ✅ WORKING - Manual slot-by-slot deployment

## Step-by-Step Manual Process

If you prefer manual control or need to troubleshoot:

### 1. Upload Master
```bash
pio run -e hub_master_deploy -t upload
```

### 2. Manual Slot Deployment
For each slot, use the new menu system:

```bash
# Connect to master (use your preferred serial terminal at 152000 baud)
# Navigate through the menu:
1          # Enter Slot Management
6          # Select Slot 1 (7=Slot2, 8=Slot3, 9=Slot4, 10=Slot5)
0          # Back to main menu
0          # Exit

# Wait for new COM port to appear, then upload
pio run -e hub_slot1_deploy -t upload
```

### 3. Verify Deployment
Connect to master and test the new menu system:
```
1 → Slot Management menu
4 → System Info (shows current configuration)
5 → Test Mode (run diagnostics)
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

## ⚠️ Known Issues (For Next Development Session)

- **No feedback verification** - Scripts don't confirm menu commands worked
- **No COM port detection** - Doesn't wait for slot ports to appear
- **No upload validation** - Assumes uploads succeed without checking
- **Sequential only** - Takes 6-8 minutes, could be faster with parallel uploads

**Status**: System works but needs robustness improvements

---

**Quick Reference Card**

```bash
# WORKING DEPLOYMENTS:
Complete System: python deploy_complete_system.py  # Master + All Slots
Slots Only:      python deploy_slots.py            # Uses existing menu
Individual:      pio run -e hub_master_deploy -t upload

# MONITORING:
Monitor Master:  pio device monitor -p COM13 -b 152000
Build All:       pio run

# ENHANCED MENU SYSTEM (DEPLOYED & WORKING):
Main Menu:       1=Slots, 2=SD Cards, 3=Config, 4=Info, 5=Test  
Slot Selection:  6=Slot1, 7=Slot2, 8=Slot3, 9=Slot4, 10=Slot5
```

## 📋 For Next Claude Code Session

See `CURRENT_STATUS.md` and `NEXT_STEPS.md` for complete current state and improvement priorities.