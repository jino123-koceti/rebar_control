# 100-Point Autonomous Tying System - Development Report

## 1. Overview

Encoder-based waypoint tracking + ping-pong repeat tying for 100-point autonomous rebar tying.
Test date: 2026-03-16

### System Flow
```
[UI] START_MISSION (repeat:ON)
  ↓
[navigator] → Area setup (encoder) → Path generation → Waypoint publish
  ↓
[rebar_controller] → Encoder odometry tracking → Waypoint arrival → TYING_START
  ↓
[tying_orchestrator] → Detection → Tying → TYING_COMPLETE:N
  ↓
[rebar_controller] → Next WP → ... → All WP done → Ping-pong reverse → ...
  ↓
total_tying_points >= 100 → MISSION_COMPLETE
```

## 2. Autonomous Mission Process

### 2.1 Area Setup (Encoder-Based)
```
1. AREA_SETUP_START → Record encoder_odom start position
2. Manual drive (forward/backward via remote)
3. AREA_SETUP_COMPLETE → Record encoder_odom end position
4. PATH_GEN → Generate waypoints from area data
```
- Previously VSLAM-based, changed to encoder odometry for consistency
- Area length calculated from encoder displacement

### 2.2 Waypoint Generation
- RL-based path generator creates waypoints within the area
- Waypoints sorted closest-to-current first
- Converted to relative coordinates (subtracted from current encoder pose)
- Typical spacing: 600mm (configurable)

### 2.3 Mission Start
```
START_MISSION (repeat:ON)
  → navigator publishes WaypointArray (relative coords)
  → navigator sends SET_REPEAT:ON to rebar_controller
  → rebar_controller saves mission_origin (current encoder_pose)
  → Control loop starts: (encoder_pose - mission_origin) vs WP target
```
- No encoder reset: UI position stays continuous
- mission_origin offset approach preserves UI display

### 2.4 Waypoint Tracking
- Controller: `_execute_differential_motion()`
- Feedback: encoder odometry (20Hz control loop)
- Speed: 0.25 m/s max linear, proportional deceleration near target
- Arrival: distance < 50mm + along-path remaining check
- Direction: determined by first WP sign (forward/backward)

### 2.5 Tying at Each Waypoint
```
WP arrival → TYING_START (direction=forward/reverse, alternating)
  → tying_orchestrator executes detection + tying cycle
  → TYING_COMPLETE:N (N = points tied)
  → rebar_controller accumulates total_tying_points
  → Move to next WP
```

## 3. Ping-Pong Repeat Mode

### 3.1 Mechanism
```
1st lap (forward):  WP[0] → WP[1] → WP[2] → WP[3]
2nd lap (reverse):  WP[2] → WP[1] → WP[0]
3rd lap (forward):  WP[1] → WP[2] → WP[3]
4th lap (reverse):  WP[2] → WP[1] → WP[0]
...until total_tying_points >= max_tying_points
```

### 3.2 Direction Alternation
- `total_wp_count % 2`: even=forward, odd=reverse
- Continuous counter across laps (not reset per lap)
- Tying tool pose naturally alternates: forward ends LEFT, reverse ends RIGHT

### 3.3 Termination Conditions
- **repeat OFF**: All WPs visited once → mission complete
- **repeat ON**: `total_tying_points >= max_tying_points` → mission complete
- **CANCEL**: Immediate stop at any time
- `max_tying_points`: configurable (default 100)

## 4. Tracking Performance

### 4.1 Path Following (from first test, 2026-03-15)
| Metric | Value |
|--------|-------|
| Cross-track error (Y) avg | -12.5mm |
| Cross-track error (Y) std | 2.5mm |
| Cross-track error range | -16.0 ~ -9.0mm |
| Heading stability avg | +0.16 deg |
| Heading stability std | 0.39 deg |
| Heading range | -0.7 ~ +1.3 deg |

### 4.2 Waypoint Arrival Accuracy
- Arrival threshold: 50mm
- Typical arrival distance: 49.3 ~ 49.9mm
- Along-path remaining at arrival: < 50mm

## 5. 100-Point Test Results (3 Trials)

### 5.1 Summary

| | Trial 1 | Trial 2 | Trial 3 | Average |
|---|:---:|:---:|:---:|:---:|
| Total Points | 100 | 106 | 101 | 102 |
| Total Time | 17.4 min | 22.8 min | 17.1 min | **19.1 min** |
| Laps | 11 | 26 | 13 | 17 |
| Original WPs | 4 | 3 | 4 | - |
| Total WP Visits | 34 | 39 | 40 | 38 |
| Points/WP | 2.9 | 2.7 | 2.5 | **2.7** |
| Sec/Point | 10.4 | 12.9 | 10.1 | **11.2** |
| Zero-detect WPs | 2 (5.9%) | 4 (10.3%) | 0 (0%) | 5.4% |

### 5.2 Trial 1 Detail (17.4 min, 100pt, 11 laps)

| Lap | Direction | Points | Cumulative | Duration |
|:---:|-----------|-------:|:----------:|--------:|
| 1 | Forward | 7pt | 7 | 88s |
| 2 | Reverse | 10pt | 17 | 106s |
| 3 | Forward | 8pt | 25 | 96s |
| 4 | Reverse | 11pt | 36 | 101s |
| 5 | Forward | 8pt | 44 | 88s |
| 6 | Reverse | 8pt | 52 | 90s |
| 7 | Forward | 8pt | 60 | 91s |
| 8 | Reverse | 10pt | 70 | 96s |
| 9 | Forward | 11pt | 81 | 99s |
| 10 | Reverse | 8pt | 89 | 90s |
| 11 | Forward | 11pt | **100** | 98s |

### 5.3 Trial 3 Detail (17.1 min, 101pt, 13 laps)

| Lap | Direction | Points | Cumulative | Duration |
|:---:|-----------|-------:|:----------:|--------:|
| 1 | Forward | 9pt | 9 | 88s |
| 2 | Reverse | 10pt | 19 | 90s |
| 3 | Forward | 7pt | 26 | 74s |
| 4 | Reverse | 10pt | 36 | 101s |
| 5 | Forward | 6pt | 42 | 55s |
| 6 | Reverse | 8pt | 50 | 84s |
| 7 | Forward | 7pt | 57 | 73s |
| 8 | Reverse | 9pt | 66 | 87s |
| 9 | Forward | 7pt | 73 | 71s |
| 10 | Reverse | 7pt | 80 | 79s |
| 11 | Forward | 7pt | 87 | 74s |
| 12 | Reverse | 8pt | 95 | 82s |
| 13 | Forward | 6pt | **101** | 68s |

### 5.4 Tying Rate
- Average: **5.5 points/min** (rolling 5-WP window)
- Peak: ~7.8 points/min
- Consistent across trials (no degradation over time)

## 6. Time Composition Analysis

### 6.1 Per 100-Point Breakdown (estimated from Trial 1)
| Component | Time | Percentage |
|-----------|-----:|:----------:|
| Tying (XY+Z+Trigger) | ~760s | 73% |
| Driving (WP to WP) | ~180s | 17% |
| Pose Change | ~80s | 8% |
| Detection | ~23s | 2% |
| **Total** | **~1043s** | 100% |

### 6.2 Speed Improvement Potential
| Speed Setting | Estimated 100pt Time |
|:---:|:---:|
| 50% (current) | ~19 min |
| 80% | ~14 min |
| 100% | ~12 min |

## 7. Key Implementation Details

### 7.1 Encoder Odometry (no reset approach)
```python
# Mission start: save origin, don't reset encoder
self.mission_origin_x = curr_x
self.mission_origin_y = curr_y
# Control: relative position
rel_x = curr_x - self.mission_origin_x
rel_y = curr_y - self.mission_origin_y
```
- UI sees continuous position (no jump to 0,0)
- Controller uses relative position internally

### 7.2 Ping-Pong WP Generation
```python
def _start_pingpong_next_lap(self):
    # Reverse WP order, exclude current endpoint
    new_wps = original_wps[:-1]  # remove last (already visited)
    new_wps.reverse()
    self.waypoint_array_x = [wp.x for wp in new_wps]
    self.current_waypoint_index = 0
    self.ping_pong_forward = not self.ping_pong_forward
```

### 7.3 Emergency Stop Integration
- CANCEL command: stops drive + tying at any point
- drive_controller: manual mode override stops wheel motors
- tying_orchestrator: CANCEL → immediate IDLE
- rebar_controller: CANCEL → reset mission state

## 8. Analysis Plots

| File | Description |
|------|-------------|
| `tying_100pt_analysis.png` | 4-panel analysis (cumulative, distribution, per-WP time, summary) |
| `tying_100pt_rate.png` | Tying rate over time (points/min) |
| `tying_100pt_analysis.py` | Analysis script (generates above plots) |

## 9. Configuration

### Key Parameters (`can_devices.yaml`)
| Parameter | Value | Description |
|-----------|-------|-------------|
| `max_tying_points` | 100 | Ping-pong termination threshold |
| `ready_x_mm` | -300.0 | Ready position X offset |
| `ready_yaw_deg` | 0.0 | Ready position Yaw offset |

### Tying Parameters (`tying_orchestrator.yaml`)
| Parameter | Value | Description |
|-----------|-------|-------------|
| `multi_detect_tries` | 5 | Detection attempts per cycle |
| `cluster_distance_mm` | 40.0 | Clustering threshold |
| `expected_points_per_cam` | 3 | Expected points per camera |
| `detection_confidence` | 0.3 | YOLO confidence threshold |
| `stage_max_speed` | 1000 dps | XY stage max speed |
| `z_tying_depth_mm` | 10.0 | Z descent for tying |
| `trigger_duration` | 0.3s | Trigger activation time |
