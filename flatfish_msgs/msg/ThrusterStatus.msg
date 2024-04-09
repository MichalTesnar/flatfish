# A command to move multiple joints with a timestamp
std_msgs/Header header

# Thruster Name
string name

# Rotation speed
float64 speed

# Thruster current
float64 current

# Reported overtemperature on the electronics
bool overtemp_bg149

# Reported overtemperature on the motor
bool overtemp_motor

#Whether the extra gain for motor start is being used or not
bool start_gain

# Whether the control parameters for air (true) or water (false) are being used
bool air_parameters

# If the control mode either "current" or "rotation speed"
string control_mode
