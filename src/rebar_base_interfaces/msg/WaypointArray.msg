# Array of waypoints for batch transfer (tire_roller style)
# Entire mission path is sent at once, controller manages index internally

# Waypoint arrays (parallel arrays for efficiency)
float32[] x              # X positions (meters)
float32[] y              # Y positions (meters)
uint8[] motion_type      # Motion type per waypoint (0=DIFFERENTIAL, 1=LATERAL)
float32[] max_speed      # Max speed per segment (m/s for diff, dps for lateral)

# Motion type constants (same as Waypoint.msg)
uint8 MOTION_DIFFERENTIAL = 0
uint8 MOTION_LATERAL = 1
