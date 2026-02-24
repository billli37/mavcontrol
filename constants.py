# Drone connection
CONNECTION_STRING = "serial:///dev/ttyAMA0:57600"

# Main control loop
LOOP_HZ = 50.0

# Takeoff parameters
TAKEOFF_ALT_M = 1.0
TAKEOFF_ALT_TOL_M = 0.15
TAKEOFF_TIMEOUT_S = 10.0

# Alignment parameters
ALIGNMENT_TIMEOUT_S = 5.0
CUTOFF_ALT_M = 0.15

# PID landing parameters
LANDING_SWITCH_ALT_M = 0.25
LANDING_DESCENT_SPEED_MPS = 0.30
LANDING_DESCENT_TIMEOUT_S = 20.0

# Color detection parameters (HSV, OpenCV ranges)
# Keep this aligned with real camera output on your setup. Even with BGR888
# configuration, testing showed RGB->HSV gives correct target detection.
FRAME_COLOR_ORDER = "RGB"
RED_LOWER_1 = (0, 70, 0)
RED_UPPER_1 = (5, 255, 255)
RED_LOWER_2 = (165, 70, 0)
RED_UPPER_2 = (180, 255, 255)
RED_MIN_AREA_PX = 300

# Alignment PID parameters (pixel error -> body velocity)
ALIGN_PID_KP = (0.002, 0.002)
ALIGN_PID_KI = (0.0, 0.0)
ALIGN_PID_KD = (0.0, 0.0)

# Horizontal velocity limits (body-frame)
MAX_SPEED_MPS = 1.0

# Servo hardware parameters
PWM_CHANNEL = 2
PWM_CHIP = 0
STEPS_0_DEG = 2.8
STEPS_180_DEG = 11.6
ANGLE_LANDING = 105
