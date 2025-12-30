# Enhanced waypoint with motion type for hybrid navigation
# Used by navigator.py and hybrid_controller.py

# Target position (in meters)
float32 x              # Target X position
float32 y              # Target Y position

# Motion control parameters
uint8 motion_type      # Motion type (see constants below)
float32 max_speed      # Maximum speed for this segment (m/s for differential, dps for lateral)

# Motion type constants
uint8 MOTION_DIFFERENTIAL = 0    # Forward/backward using differential drive
uint8 MOTION_LATERAL = 1         # Pure lateral motion (no rotation)
uint8 MOTION_HYBRID = 2          # Combined motion (future use)
