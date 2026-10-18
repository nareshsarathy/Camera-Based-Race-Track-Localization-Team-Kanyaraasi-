"""Configuration parameters for Yas Marina Camera-Based Localization."""
from pathlib import Path

# Paths
BASE_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = BASE_DIR / "data"
OUTPUTS_DIR = BASE_DIR / "outputs"

TRACK_BOUNDS_PATH = DATA_DIR / "track_bounds.json"
CAMERA_CALIB_PATH = DATA_DIR / "camera_calibrations.json"
GT_PATH = DATA_DIR / "gt.csv"

# Topics
STATE_TOPIC = "/constructor0/state_estimation"
CAMERA_FRONT_TOPIC = "/constructor0/sensor/camera_fr/compressed_image"
CAMERA_NARROW_TOPIC = "/constructor0/sensor/camera_fl/compressed_image"
TRACK_BOUNDS_TOPIC = "/constructor0/track_bounds"
TF_STATIC_TOPIC = "/tf_static"

# Kinematic parameters
DEFAULT_GYRO_BIAS = -0.000204  # rad/s calibrated from Yas Marina training run
SPEED_SCALE_FACTOR = 1.000214  # calibrated wheel speed scaling
LAP_LENGTH_RACING_LINE = 2970.0  # metres along typical racing line

# Start / Finish line coordinates on main straight
START_FINISH_X = 5.60
START_FINISH_Y = 3.08
START_FINISH_YAW = 0.135
