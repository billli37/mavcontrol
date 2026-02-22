# Drone connection
CONNECTION_STRING = "serial:///dev/ttyAMA0:57600"

# TakeOff Parameters
TAKEOFF_ALT_M = 1.0
TAKEOFF_ALT_TOL_M = 0.15
TAKEOFF_HOLD_TIME_S = 0.5
TAKEOFF_TIMEOUT_S = 20.0

# Alignment parameters
ALIGNMENT_TIMEOUT_S = 15.0

# Controller parameters
LOOP_HZ = 50.0

# Color detection parameters (HSV, OpenCV ranges)
FRAME_COLOR_ORDER = "RGB"
RED_LOWER_1 = (0, 70, 0)
RED_UPPER_1 = (5, 255, 255)
RED_LOWER_2 = (165, 70, 0)
RED_UPPER_2 = (180, 255, 255)
RED_MIN_AREA_PX = 300

# Alignment PID parameters (pixel error -> body velocity)
ALIGN_PID_KP = (0.0025, 0.0025)
ALIGN_PID_KI = (0.0, 0.0)
ALIGN_PID_KD = (0.0008, 0.0008)

# Horizontal velocity limits (body-frame)
MAX_SPEED_MPS = 1.0

# Vertical altitude PID parameters (meters -> m/s command)
ALT_PID_KP = (0.8,)
ALT_PID_KI = (0.0,)
ALT_PID_KD = (0.2,)

# Vertical velocity limits (body-frame: +vz is down)
MAX_CLIMB_MPS = 2.0
MAX_DESCENT_MPS = 2.0

# Landing transition / descent parameters
LANDING_SWITCH_ALT_M = 0.25
LANDING_DESCENT_TARGET_M = 0.25
LANDING_DESCENT_TIMEOUT_S = 20.0

# ArUco + camera calibration parameters
MARKER_ID = 0
MARKER_SIZE_METERS = 0.1
CALIB_MTX_FILE = "camera_matrix.npy"
CALIB_DIST_FILE = "dist_coeffs.npy"

# Servo hardware parameters
PWM_CHANNEL = 2
PWM_CHIP = 0
STEPS_0_DEG = 2.8
STEPS_180_DEG = 11.6
ANGLE_LANDING = 105
