# Current state of a servo
# Reports position, speed, load, and status

uint8 servo_id          # Servo identifier
float32 current_angle   # Current angle in radians
float32 current_speed   # Current speed
float32 load           # Current load/torque
bool is_moving         # Whether servo is currently moving
bool error             # Error status
uint8 temperature      # Temperature in Celsius