# S17/S18 Lateral Motion Fix Summary

## Issue Identified

### Problem
When testing 360° lateral motion with S17/S18 buttons on 0x143 motor:
- Motor moved **270° forward, then 90° backward** instead of 360° forward
- Motor **vibrated at goal position** with squeaking noise
- Position tracking was fundamentally flawed

### Root Cause Analysis
Reading `/home/koceti/ros2_ws/rmd_can_protocol/250516_CAN BUS Motor Motion Protocol V4.3.md` revealed:

**0xA2 (Speed Control) Reply Format:**
```
DATA[6:7] = int16_t type, 1degree/LSB, maximum range ±32767degree
```

**Critical Finding:** This is **SINGLE-TURN angle** that wraps around at ±32767°, NOT multi-turn position!

**Previous Flawed Approach:**
```python
# ❌ WRONG: Trying to track multi-turn position from single-turn encoder
encoder_deg = msg.current_position * 360.0 / 227.0  # CPR conversion
self.motor_positions[motor_id] = encoder_deg
```

**Why It Failed:**
- Encoder value wraps: 0→227→0 (single revolution)
- Multi-turn tracking requires either:
  1. Command 0x60 (Read Multi-Turn Encoder Position)
  2. **Speed integration method** ← Chosen solution

---

## Solution: Speed Integration Method

### New Implementation

**Motor Feedback Callback** (drive_controller.py:285-313):
```python
def motor_feedback_callback(self, msg):
    """모터 피드백 처리 (0x243 → 0x43) - 속도 적분 방식"""
    motor_id = msg.motor_id
    if motor_id == self.lateral_motor_id:
        current_time = self.get_clock().now()

        # 속도 피드백을 적분하여 위치 계산 (더 정확)
        if self.last_feedback_time is not None:
            dt = (current_time - self.last_feedback_time).nanoseconds / 1e9
            # msg.current_speed는 dps 단위
            delta_deg = msg.current_speed * dt
            old_pos = self.motor_positions.get(motor_id, 0.0)
            new_pos = old_pos + delta_deg
            self.motor_positions[motor_id] = new_pos
        else:
            # 첫 피드백: 엔코더 기준으로 초기화
            new_pos = msg.current_position * 360.0 / 227.0
            self.motor_positions[motor_id] = new_pos

        self.last_feedback_time = current_time
```

**Key Points:**
1. **First feedback**: Initialize position from encoder (single-turn reference)
2. **Subsequent updates**: Integrate velocity over time: `position += speed * dt`
3. **No wrap handling needed**: Position accumulates continuously
4. **Increased tolerance**: 0.5° → **5.0°** to prevent vibration

---

## Configuration Changes

### can_devices.yaml
```yaml
drive_controller:
  ros__parameters:
    lateral_max_speed: 150.0  # 0x143 등속 최대 속도 (dps) - 정격: 498 dps
    lateral_position_tolerance: 5.0  # 도달 판정 오차 (deg) - 진동 방지
    lateral_encoder_cpr: 227.0  # counts per revolution (출력축, 실측값)
```

### velocity_profiler.py
```python
def __init__(self, v_max):
    self.acc_time = 0.5  # 가속 시간 (초) - 0x143: 150dps 도달 시 0.5초
    self.dec_time = 0.5  # 감속 시간 (초) - 0x143: 150dps→0 감속 시 0.5초
    self.v_min = 0.1
```

---

## Expected Behavior After Fix

### S17 Button (+360°)
1. Press S17
2. Motor accelerates: 0 → 150 dps (0.5s)
3. Constant velocity: 150 dps
4. Motor decelerates: 150 → 0 dps (0.5s)
5. Stops at +360° (within ±5° tolerance)

### S18 Button (-360°)
1. Press S18
2. Motor accelerates: 0 → -150 dps (0.5s)
3. Constant velocity: -150 dps
4. Motor decelerates: -150 → 0 dps (0.5s)
5. Stops at -360° (within ±5° tolerance)

### Expected Logs (Tire Roller style)
```
[INFO] [drive_controller]: 🔘 S17 버튼 눌림 감지 (+360.0° 횡이동 시작)
[INFO] [drive_controller]: 횡이동 시작: 0.0° → 360.0° (+360.0°)
[INFO] [drive_controller]: [Lateral 0x143] target:360.0° current:7.5° dist_moved:7.5° dist_togo:352.5° vel_cmd:45.0dps elapsed:0.10s
[INFO] [can_sender]: [CAN TX 0x143] lateral_speed:75.0dps → cmd:7500 (0.01dps unit)
[INFO] [can_parser]: [CAN RX 0x243] encoder:23 speed:78dps current:1.23A temp:32°C
[INFO] [drive_controller]: [Lateral 0x143] target:360.0° current:89.3° dist_moved:89.3° dist_togo:270.7° vel_cmd:150.0dps elapsed:0.65s
[INFO] [drive_controller]: [Lateral 0x143] target:360.0° current:342.1° dist_moved:342.1° dist_togo:17.9° vel_cmd:82.3dps elapsed:2.45s
[INFO] [drive_controller]: [Lateral 0x143] target:360.0° current:357.8° dist_moved:357.8° dist_togo:2.2° vel_cmd:21.1dps elapsed:2.75s
[INFO] [drive_controller]: 횡이동 완료: 358.3° (오차 1.70°)
```

---

## Testing Checklist

### 1. Hardware Setup
- [ ] CAN2 connected to 0x143 motor at 1Mbps
- [ ] CAN3 connected to Iron-MD remote at 250kbps
- [ ] S19 switch set to Manual mode

### 2. Launch System
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rebar_base_control base_system.launch.py
```

### 3. Test S17 (+360°)
- [ ] Press S17 button once
- [ ] Motor accelerates smoothly (no jerky motion)
- [ ] Motor reaches constant velocity (~150 dps)
- [ ] Motor decelerates smoothly
- [ ] Motor stops near +360° (±5°)
- [ ] **NO vibration or squeaking at goal**
- [ ] **NO overshoot/undershoot (e.g., 270° + 90° back)**

### 4. Test S18 (-360°)
- [ ] Press S18 button once
- [ ] Motor accelerates smoothly in reverse
- [ ] Motor reaches constant velocity (~-150 dps)
- [ ] Motor decelerates smoothly
- [ ] Motor stops near -360° (±5°)
- [ ] **NO vibration or squeaking at goal**
- [ ] **NO overshoot/undershoot**

### 5. Check Logs
- [ ] Log shows `🔘 S17 버튼 눌림 감지` when pressing S17
- [ ] Log shows `[Lateral 0x143]` with detailed position/velocity info
- [ ] Log shows `[CAN TX 0x143]` with lateral_speed commands
- [ ] Log shows `[CAN RX 0x243]` with motor feedback
- [ ] Log shows `횡이동 완료` with final position and error
- [ ] Final error is within ±5°

### 6. Multi-Step Test
- [ ] Press S17 → +360° → Complete
- [ ] Press S17 again → +720° total → Complete
- [ ] Press S18 → +360° total (back to 360°) → Complete
- [ ] Position tracking remains accurate across multiple moves

---

## Technical Details

### Why Speed Integration Works

**Physics:**
```
position(t) = position(t₀) + ∫ velocity(τ) dτ  (from t₀ to t)
```

**Discrete Implementation:**
```
position[n] = position[n-1] + velocity[n-1] × Δt
```

**Advantages over Encoder Counting:**
1. No wrap-around issues (continuous accumulation)
2. Smooth multi-turn tracking
3. No need for CPR conversion (uses actual dps from motor)
4. More accurate for long movements (>360°)

**Disadvantages:**
1. Accumulates integration error over time (acceptable for short motions)
2. Requires consistent feedback (handled with 1s timeout check)
3. Initial position from encoder (but this is fine for relative moves)

---

## Files Modified

### Created
- `src/rebar_base_control/rebar_base_control/velocity_profiler.py` - Trapezoidal velocity profiler

### Modified
- `src/rebar_base_control/rebar_base_control/drive_controller.py`
  - Lines 285-313: Speed integration method
  - Line 37: Increased tolerance to 5.0°
  - Lines 315-348: S17/S18 button handling
  - Lines 350-418: Lateral motion profile generation

- `src/rebar_base_control/rebar_base_control/can_sender.py`
  - Lines 129-143: Lateral speed transmission with logging

- `src/rebar_base_control/rebar_base_control/can_parser.py`
  - Lines 159-164: 0x243 feedback logging

- `src/rebar_base_control/config/can_devices.yaml`
  - Added `lateral_position_tolerance: 5.0`

- `src/rebar_base_interfaces/msg/DriveControl.msg`
  - Added `float32 lateral_speed` field

---

## Next Steps

1. **Test with actual hardware** using the checklist above
2. **Monitor logs** for position tracking accuracy
3. **Verify no vibration** at goal position with 5° tolerance
4. **Check multi-step movements** (multiple S17/S18 presses)
5. **Report any issues** if behavior doesn't match expected

---

## Rollback Plan (If Needed)

If speed integration doesn't work as expected:

**Alternative Approach:** Use Command 0x60 (Read Multi-Turn Encoder Position)
- Periodically query actual multi-turn position from motor
- More accurate but requires additional CAN traffic
- See manual section "Multi-turn encoder data is read" for implementation

**Current Git Branch:** `refactoring/phase2-base-control`

To revert changes:
```bash
git diff HEAD -- src/rebar_base_control/rebar_base_control/drive_controller.py
git checkout HEAD -- src/rebar_base_control/rebar_base_control/drive_controller.py
```

---

**Build Status:** ✅ Successfully built
**Ready for Testing:** Yes
**Estimated Test Duration:** 5-10 minutes per test scenario
