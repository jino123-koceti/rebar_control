# I/O status message
# From EZI-IO Modbus TCP device

bool[] digital_inputs      # Digital input states
float32[] analog_inputs    # Analog input values (voltage, current, etc.)

# Sensor-specific values
float32 battery_voltage    # Battery voltage (V)
bool limit_switch_front    # Front limit switch
bool limit_switch_rear     # Rear limit switch
