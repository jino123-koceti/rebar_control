# Gripper control message
# Commands for Seengrip gripper via Modbus RTU

uint8 COMMAND_STOP = 0
uint8 COMMAND_OPEN = 1
uint8 COMMAND_CLOSE = 2
uint8 COMMAND_GRIP = 3

uint8 command         # Command type (use constants above)
uint8 speed           # Gripper speed (0-100%)
uint8 force           # Gripping force (0-100%)
