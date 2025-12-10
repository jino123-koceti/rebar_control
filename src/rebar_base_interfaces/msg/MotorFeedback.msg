# Motor feedback message
# Received from CAN2 bus (motors 0x141, 0x142)

uint8 motor_id              # Motor CAN ID (0x141 or 0x142)
float32 current_speed       # Current motor speed (rpm or m/s)
float32 current_position    # Current motor position (encoder counts)
int16 current_current       # Motor current (mA)
uint8 temperature           # Motor temperature (°C)
uint8 error_code            # Error code (0 = no error)
uint8 status                # Motor status flags
