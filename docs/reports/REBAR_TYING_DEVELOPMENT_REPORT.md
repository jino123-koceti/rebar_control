# Rebar Tying Autonomous System - Development Report

## 1. Overview

Encoder-based autonomous driving + 12-point tying cycle system for rebar tying robot.
Development period: 2026-03-15 ~ 2026-03-17

### System Architecture
```
[UI (External PC)]
    ↓ Zenoh
[zenoh_client] → /mission/command
    ↓
[navigator]          → Area setup, Path generation, Mission management
[rebar_controller]   → Waypoint tracking (encoder odometry), Tying coordination
[tying_orchestrator] → Detection, Pose change, Tying execution
[joint_controller]   → Homing, Stage XYZ + Yaw control
[encoder_odom]       → Differential drive odometry (motor encoder)
[drive_controller]   → Wheel motor speed control
```

## 2. Encoder-Based Autonomous Driving

### 2.1 Encoder Odometry (`encoder_odom`)
- Differential drive integration from wheel motors (0x141 left, 0x142 right)
- Publishes `/encoder_odom` (PoseStamped)
- Front/back inversion compensated (v_linear, v_angular negated)
- VSLAM is auxiliary (log only), encoder is primary for control

### 2.2 Waypoint Tracking (`rebar_controller`)
- `_execute_differential_motion()`: Pure pursuit-style controller
- Position: encoder_pose (x, y, theta)
- Waypoint arrival: distance < 50mm threshold
- Along-path remaining: projected distance for direction-aware arrival

### 2.3 Navigation (`navigator`)
- Area setup: encoder-based position recording
- Path generation: RL-based waypoint generation from area data
- Waypoints converted to relative coordinates (encoder resets at mission start removed)
- `mission_origin` offset approach: UI position continuous, no jump to (0,0)

## 3. 12-Point Tying Cycle

### 3.1 Detection Pipeline
```
Multi-detection (5 tries, right + left camera alternating)
  → Accumulate raw points
  → Per-camera clustering (40mm distance threshold)
  → Range filter (margin 30mm)
  → Interpolation (if < 3 clusters, add at ~195mm intervals)
  → Trendline fitting + Y-correction
  → Outlier removal (err > 100mm) + refit
  → Final range filter + clamping (margin 30mm)
  → P1~P6 labeling
```

### 3.2 Tying Sequence (Forward: Right → Left)
```
P1(~400mm) → P2(~200mm) → P3(~0mm)     [Right cam, yaw ~-3 deg]
  ↓ Pose change: Y→near → Yaw→173 deg → Y→far → Yaw→390 deg → Yaw→393 deg
P4(~0mm) → P5(~190mm) → P6(~370mm)      [Left cam, yaw ~393 deg]
```

### 3.3 Tying Sequence (Reverse: Left → Right)
```
P6(~370mm) → P5(~190mm) → P4(~0mm)      [Left cam, yaw ~393 deg]
  ↓ Pose change: Y→far → Yaw→173 deg → Y→near → Yaw→-3 deg
P3(~0mm) → P2(~200mm) → P1(~400mm)      [Right cam, yaw ~-3 deg]
```

### 3.4 Per-Point Action Sequence
```
XY move → Settle (1.5s) → Z down (10mm) → Trigger (0.3s) → Z up (10mm)
```

### 3.5 Pose Change Timing
| Step | Action | Duration |
|------|--------|----------|
| 1/5 | XY → home, Y=near | ~2s |
| 2/5 | Yaw → 173 deg (mid) | ~3s |
| 3/5 | XY → home, Y=far | ~1.5s |
| 4/5 | Yaw → 390 deg (approach) | ~3s |
| 5/5 | Yaw → 393 deg (final) | ~0.5s |
| **Total** | | **~10s** |

## 4. Repeat Mode (Ping-Pong)

### 4.1 Standalone Tying (TYING_START, repeat:ON)
```json
{"command": "TYING_START", "speed": 30, "repeat": "ON"}
```
- Forward pass: Right→PoseChange→Left → COMPLETE → auto re-detect
- Return pass: Left→PoseChange→Right → TYING_COMPLETE
- Direction auto-toggle: forward → reverse

### 4.2 Autonomous Mission (START_MISSION, repeat:ON)
```json
{"command": "START_MISSION", "repeat": "ON"}
```
- Waypoint ping-pong: WP0→WP1→WP2→WP1→WP0→...
- Direction alternation per WP: even=forward, odd=reverse
- Termination: `total_tying_points >= max_tying_points` (config, default 100)
- Emergency stop: CANCEL command at any time

### 4.3 100-Point Autonomous Tying Results (3 Trials, 2026-03-16)

| | Trial 1 | Trial 2 | Trial 3 | Average |
|---|:---:|:---:|:---:|:---:|
| Total Points | 100 | 106 | 101 | 102 |
| Total Time | 17.4 min | 22.8 min | 17.1 min | **19.1 min** |
| Laps | 11 | 26 | 13 | 17 |
| Original WPs | 4 | 3 | 4 | - |
| Points/WP | 2.9 | 2.7 | 2.5 | 2.7 |
| Sec/Point | 10.4 | 12.9 | 10.1 | **11.2** |
| Zero-detect % | 5.9% | 10.3% | 0% | 5.4% |

### 4.4 Time Breakdown (per cycle, from tying_cycle_report.py)

| Phase | Cycle 1 | Cycle 2 | Cycle 3 | Average |
|-------|:-------:|:-------:|:-------:|:-------:|
| Initial Detection | 1.8 | 0.9 | 0.8 | 1.2 |
| Forward Right P1~P3 | 11.4 | 11.2 | 11.3 | 11.3 |
| Pose Change 1 (R→L) | 6.1 | 6.1 | 6.2 | 6.1 |
| Forward Left P4~P6 | 10.3 | 10.3 | 10.3 | 10.3 |
| Re-detection | 0.9 | 0.9 | 0.9 | 0.9 |
| Return Left P6'~P4' | 9.8 | 9.9 | 9.8 | 9.8 |
| Pose Change 2 (L→R) | 5.8 | 5.9 | 6.1 | 5.9 |
| Return Right P3'~P1' | 12.8 | 12.6 | 12.6 | 12.7 |
| **Total** | **58.9** | **57.8** | **58.0** | **58.2** |

Time distribution: Tying 75.7% / Pose Change 20.7% / Detection 3.5%

## 5. UI Commands

| Command | Description |
|---------|-------------|
| `{"command": "GO_HOME"}` | Homing sequence |
| `{"command": "TYING_START", "speed": 50}` | Single tying cycle |
| `{"command": "TYING_START", "speed": 50, "repeat": "ON"}` | Repeat tying (forward+return) |
| `{"command": "START_MISSION", "repeat": "ON"}` | Autonomous mission with ping-pong |
| `{"command": "CHN_POS=left"}` | Pose change to left cam |
| `{"command": "CHN_POS=right"}` | Pose change to right cam |
| `{"command": "CANCEL"}` | Emergency stop |
| `{"command": "PAUSE"}` / `{"command": "RESUME"}` | Pause / Resume |

## 6. Key Improvements (2026-03-17)

### 6.1 Left Camera X-Inversion Fix
- **Before**: Left camera raw coords sign-inverted → range filter removed valid points
- **After**: Calibration model already outputs robot coordinates → inversion removed
- **Result**: Left detection 2pt → 3pt (full detection)

### 6.2 Trendline Outlier Removal
- **Before**: Single outlier corrupted trendline (avg error 80mm)
- **After**: err > 100mm points removed + refit (avg error 9.8mm)

### 6.3 Clamp Margin Increase
- **Before**: clamp_margin=15mm → X=-24mm point removed
- **After**: clamp_margin=30mm → X=-24mm clamped to 0mm, point preserved

### 6.4 Return Pass Direction Fix
- **Before**: Return pass used forward direction → 3 pose changes, ended at LEFT
- **After**: `tying_direction = 'reverse'` set at return entry → 2 pose changes, ends at RIGHT

### 6.5 12-Point Full Detection Result (2026-03-17)
```
Forward: P1(395,28) P2(187,35) P3(0,42) P4(0,239) P5(177,249) P6(366,259)  → 6pt
Return:  P1(395,29) P2(194,38) P3(0,47) P4(0,238) P5(170,245) P6(364,252)  → 6pt
Total:   12pt / 63 sec
```

## 7. File Logs

| Node | Log Path |
|------|----------|
| rebar_controller | `/tmp/rebar_controller_YYYYMMDD_HHMMSS.log` |
| navigator | `/tmp/navigator_YYYYMMDD_HHMMSS.log` |
| tying_orchestrator | `/tmp/tying_orchestrator_YYYYMMDD_HHMMSS.log` |
| joint_controller | `/tmp/joint_controller_YYYYMMDD_HHMMSS.log` |

## 8. Known Issues

1. **X-axis MOVE_TO_READY timeout on first homing**: Motor doesn't respond to large position command after power cycle. Workaround: run homing twice.
2. **Yaw 0x90 encoder drift**: Gear backlash/slip causes 0x90 value to change at same output position. Cannot use 0x90 for absolute position check. Pending: hardware limit sensor addition.
3. **Return pass detection accuracy**: Tool may partially block camera view at certain positions. Improvement: add XY home return before re-detection.
