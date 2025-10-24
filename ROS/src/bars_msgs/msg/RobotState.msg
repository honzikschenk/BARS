# Overall robot state information
# Combines multiple subsystem states

std_msgs/Header header

# State machine status
string current_state      # Current state (idle, walking, balancing, etc.)
string previous_state     # Previous state
float64 state_duration    # Time in current state (seconds)

# Physical status
geometry_msgs/Pose pose          # Robot pose in world frame
geometry_msgs/Twist velocity     # Linear and angular velocity
bool is_stable                   # Whether robot is in stable position
float32 battery_voltage          # Battery voltage
uint8 emergency_stop             # Emergency stop status (0=normal, 1=estop)

# Subsystem status
bool servos_enabled              # Whether servo control is active
bool sensors_active              # Whether sensors are working
bool communication_ok            # Communication with Arduino status