# SMBUS FAN AND INTERNAL LIGHTS
#   bus arg 1
BUS_ARG_1 = 0x0d
#   bus arg 2
BUS_ARG_2_LIGHT_STATE = 0x07
BUS_ARG_2_LIGHT_MODE = 0x04
#   bus arg 3
LIGHT_STOP_CMD = 0
LIGHT_EFFECT_CMD_LIST = [0, 1, 2, 3, 4]

# robot modes
MODE_USER_CONTROLLED = 'user_controlled'
MODE_AUTONOMOUS_VISION = 'autonomous_vision'
# MODE_AUTONOMOUS_SOUND = 'autonomous_sound'
# robot sub modes
SUB_MODE_WHEELS = 'wheels'
SUB_MODE_ARM_FK = 'arm_fk'
SUB_MODE_ARM_IK = 'arm_ik'

# beep durations
SHORT_BEEP = 50  # milliseconds
MEDIUM_BEEP = 300  # milliseconds
LONG_BEEP = 1000  # milliseconds

# OS PATHS
MAIN_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/'
APP_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/'
# LIDAR_LIB_PATH = '/root/GIT/yahboom_rdk_x3_robot/library_ws_src/src/oradar_lidar/sdk/build/liboradar_sdk.so'
# LIDAR_LIB_PATH = '/root/software/library_ws/src/oradar_lidar/sdk/build/liboradar_sdk.so'
CONFIG_FOLDER_PATH = APP_FOLDER_PATH + 'configs/'
OUTPUT_FOLDER_PATH = APP_FOLDER_PATH + 'output/'
GENERIC_MODEL_FOLDER_PATH = APP_FOLDER_PATH + 'models/vision_models/'
MAGIC_MODEL_FOLDER_PATH = APP_FOLDER_PATH + 'models/magic_detector/'
SCRIPT_FOLDER_PATH = MAIN_FOLDER_PATH + 'script_launchers/'
URDF_FOLDER_PATH = APP_FOLDER_PATH + 'urdf/'

# 40 PIN INTERFACE
# with GPIO mode = BOARD
# commented cables are connected to pins with fixed outputs (like power or ground), so it is not necessary to
# initialize and manage them in the code

# two pin button (blue-black)
BLUE_CABLE_01 = 16  # GPIO CABLE
# BLACK_CABLE = 6 GROUND CABLE !!!IMPORTANT!!!
# two pin button (red-black)
RED_CABLE_02 = 18  # GPIO CABLE
# BLACK_CABLE = 20 GROUND CABLE !!!IMPORTANT!!!

# tri pin led
GREEN_CABLE_01 = 13  # GPIO CABLE
# BLACK_CABLE = 14 GROUND CABLE !!!IMPORTANT!!!
RED_CABLE_01 = 15  # GPIO CABLE

# led colors
POWER_OFF = 'off'
RED = 'red'
ORANGE = 'orange'
GREEN = 'green'

# Battery voltage
MAX_VOLTAGE = 8.4  # 100% battery
MIN_VOLTAGE = 6.6  # 0% battery

# Controller states
STATE_OK = 0
STATE_NO_OPEN = 1
STATE_DISCONNECT = 2
STATE_KEY_BREAK = 3

# COMPUTER VISION MODEL
YOLO_CLASS_DICT = {
    0: 'person',
    1: 'bicycle',
    2: 'car',
    3: 'motorcycle',
    4: 'airplane',
    5: 'bus',
    6: 'train',
    7: 'truck',
    8: 'boat',
    9: 'traffic light',
    10: 'fire hydrant',
    11: 'stop sign',
    12: 'parking meter',
    13: 'bench',
    14: 'bird',
    15: 'cat',
    16: 'dog',
    17: 'horse',
    18: 'sheep',
    19: 'cow',
    20: 'elephant',
    21: 'bear',
    22: 'zebra',
    23: 'giraffe',
    24: 'backpack',
    25: 'umbrella',
    26: 'handbag',
    27: 'tie',
    28: 'suitcase',
    29: 'frisbee',
    30: 'skis',
    31: 'snowboard',
    32: 'sports ball',
    33: 'kite',
    34: 'baseball bat',
    35: 'baseball glove',
    36: 'skateboard',
    37: 'surfboard',
    38: 'tennis racket',
    39: 'bottle',
    40: 'wine glass',
    41: 'cup',
    42: 'fork',
    43: 'knife',
    44: 'spoon',
    45: 'bowl',
    46: 'banana',
    47: 'apple',
    48: 'sandwich',
    49: 'orange',
    50: 'broccoli',
    51: 'carrot',
    52: 'hot dog',
    53: 'pizza',
    54: 'donut',
    55: 'cake',
    56: 'chair',
    57: 'couch',
    58: 'potted plant',
    59: 'bed',
    60: 'dining table',
    61: 'toilet',
    62: 'tv',
    63: 'laptop',
    64: 'mouse',
    65: 'remote',
    66: 'keyboard',
    67: 'cell phone',
    68: 'microwave',
    69: 'oven',
    70: 'toaster',
    71: 'sink',
    72: 'refrigerator',
    73: 'book',
    74: 'clock',
    75: 'vase',
    76: 'scissors',
    77: 'teddy bear',
    78: 'hair drier',
    79: 'toothbrush'
}
