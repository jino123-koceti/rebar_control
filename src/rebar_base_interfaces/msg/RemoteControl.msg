# Remote control message
# Parsed from CAN3 remote control signals

bool switch_s10        # Manual mode switch
bool switch_s20        # Auto mode switch
bool emergency_stop    # Emergency stop button

uint8[] buttons        # Button states (array of button values)
float32[] joysticks    # Joystick values (array of analog values)

# AN commands (from remote control)
uint8 an_command       # AN1, AN2, AN3, etc.
