# Joint angles for the entire robot
# Defines the position of all joints

std_msgs/Header header

# Left leg joints (hip, knee, ankle)
float32 left_hip_pitch
float32 left_hip_roll
float32 left_hip_yaw
float32 left_knee_pitch
float32 left_ankle_pitch
float32 left_ankle_roll

# Right leg joints (hip, knee, ankle)
float32 right_hip_pitch
float32 right_hip_roll
float32 right_hip_yaw
float32 right_knee_pitch
float32 right_ankle_pitch
float32 right_ankle_roll

# Optional: torso/arm joints for future expansion
float32[] additional_joints