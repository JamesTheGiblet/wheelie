# 🚀 Quick Start: Autonomous Calibration System

## First Time Setup

### 1. Physical Preparation

- Place robot on flat surface with 1+ meter clearance
- Ensure a wall or obstacle is 20cm-2m in front of the robot for ToF (VL53L0X) targeting
   (Rear ultrasonic sensor is used for backup and safety, not for calibration)
- Verify all connections (especially encoders: Right on GPIO 5, Left on GPIO 33)
- Fully charge batteries

### 2. Initial Power-On

```txt
🤖 WHEELIE AUTONOMOUS ROBOT - Refactored Main File with Auto-Calibration
════════════════════════════════════════════════════════════════════════

🔍 CHECKING CALIBRATION STATUS...
📏 Setting up encoder interrupts...
✅ Encoders initialized successfully
❌ Robot needs calibration

🤖 ROBOT REQUIRES CALIBRATION
Please ensure:
• Robot has at least 1 meter of clear space
• A wall or obstacle is within 2 meters
• Robot is on a flat, stable surface

Starting calibration in 5 seconds...
(Power off now if conditions are not suitable)
⏰ Starting in 5...
⏰ Starting in 4...
⏰ Starting in 3...
⏰ Starting in 2...
⏰ Starting in 1...

🚀 STARTING AUTONOMOUS CALIBRATION SEQUENCE
════════════════════════════════════════════════
```

### 3. Calibration Process (Automatic)

The robot will automatically:

- **Phase 1:** Learn which motors create left/right turns (30 seconds)
- **Phase 2:** Measure exact ticks needed for 90° turns (45 seconds)  
- **Phase 3:** Determine forward/backward commands (60 seconds)
- **Phase 4:** Calibrate distance measurements (45 seconds)

Total time: **3-5 minutes**

### 4. Completion and Reboot

```txt
🎉 CALIBRATION SUCCESS!
   Total time: 187.3 seconds
   Robot is now ready for autonomous operation
   Calibration data saved permanently to EEPROM

🔄 Rebooting in 3 seconds for normal operation...
```

## Subsequent Power-Ons

```txt
🔍 CHECKING CALIBRATION STATUS...
   Magic number: 0x7B (expected: 0x7B)
   Version: 1 (expected: 1)
✅ Robot is already calibrated!

📖 Loading calibration data from EEPROM...
📊 CALIBRATION DATA SUMMARY
════════════════════════════
Magic: 0x7B, Version: 1
Ticks per 90°: 1446
Ticks per mm: 10.52
ToF offset: 12.3 mm
Motor directions:
  Left: M1=REV, M2=FWD
  Forward: M1=FWD, M2=FWD
════════════════════════════

✅ Calibration data loaded successfully!
🤖 Robot is ready for precise autonomous operation

Boot time after calibration: <1 second

## Force Recalibration
```

## Using Calibrated Movements

### In your code

```cpp
// Basic movements (use calibrated directions)
1. **Hold BOOT button while powering on**
2. Release when you see: `🔄 FORCE RECALIBRATION detected` (serial message or LED indicator)
3. Follow normal calibration process

## Using Calibrated Movements

### In your code

```cpp
// Basic movements (use calibrated directions)
calibratedTurnLeft();
calibratedTurnRight(); 
calibratedMoveForward();
calibratedMoveBackward();

// Precise movements (use calibrated measurements)
calibratedTurn90Left();        // Exactly 90° left
calibratedTurn90Right();       // Exactly 90° right
calibratedMoveDistance(200);   // Move exactly 20cm forward
calibratedMoveDistance(-100);  // Move exactly 10cm backward
```

### Navigation improvements

- **Perfect 90° turns** for grid-based navigation
- **Accurate distance movements** for precise positioning
- **Consistent behavior** across different robots
- **No manual tuning required**

## Troubleshooting

### Calibration Fails

**Most common causes:**

1. **Insufficient clearance** - Need 1m+ clear space
2. **No ToF target** - Need wall/obstacle 20cm-2m away
3. **Encoder issues** - Check connections: Right encoder to GPIO 5, Left encoder to GPIO 33
4. **Low battery** - Ensure full charge before calibration

### Robot Recalibrates Every Boot

- EEPROM not saving properly
- Power instability during calibration
- Calibration sequence didn't complete fully

### Movements Not Precise

- Run force recalibration (see instructions above: hold BOOT button while powering on)
- Check wheel attachments
- Verify encoder wheel installation

## Success Indicators

✅ **Calibration successful when you see:**

- All 4 phases complete without errors
- Reasonable calibration values (shown in summary)
- "CALIBRATION SUCCESS!" message
- Automatic reboot to normal operation

✅ **Robot working properly when:**

- Turns exactly 90° with `calibratedTurn90Left()`
- Moves precise distances with `calibratedMoveDistance()`
- Consistent behavior every time
- Fast boot (<1 second) on subsequent power-ons

---

**🎉 Congratulations! Your robot now has professional-grade autonomous capabilities with zero manual tuning!**
