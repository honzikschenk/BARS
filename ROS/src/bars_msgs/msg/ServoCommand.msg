# Command for controlling a single servo
# Used to send position and speed commands to servos

uint8 servo_id          # Servo identifier (0-255)
float32 target_angle    # Target angle in radians
float32 speed           # Speed of movement (0.0-1.0)
bool enable            # Enable/disable servo