# SMBUS FAN AND INTERNAL LIGHTS
#   bus arg 1
BUS_ARG_1 = 0x0d
#   bus arg 2
BUS_ARG_2_FAN_STATE = 0x08
BUS_ARG_2_LIGHT_STATE = 0x07
BUS_ARG_2_LIGHT_MODE = 0x04
#   bus arg 3
FAN_STOP_CMD = 0
FAN_START_CMD = 1
LIGHT_STOP_CMD = 0
LIGHT_EFFECT_CMD_LIST = [0, 1, 2, 3, 4]
GPIO_LED_COLOR_LIST = ['off', 'red', 'green', 'red_and_green']

# OS PATHS
MAIN_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/'
APP_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/'
CONFIG_FOLDER_PATH = APP_FOLDER_PATH + 'configs/'
OUTPUT_FOLDER_PATH = APP_FOLDER_PATH + 'output/'
GENERIC_MODEL_FOLDER_PATH = APP_FOLDER_PATH + 'models/generic_detector/'
MAGIC_MODEL_FOLDER_PATH = APP_FOLDER_PATH + 'models/magic_detector/'
SCRIPT_FOLDER_PATH = MAIN_FOLDER_PATH + 'script_launchers/'

# 40 PIN INTERFACE
# with GPIO mode = BOARD
VIOLET_CABLE = 13   # GPIO17
BLUE_CABLE = 15     # GPIO27
GREEN_CABLE = 16    # GPIO22
# respectively: GPIO5, GPIO6, PWM0, I2S0_LRCK
BLACK_CABLES = [29, 31, 33, 35]

# COMPUTER VISION MODEL
CPU_DEVICE = 'cpu'
TPU_DEVICE = 'edgetpu'
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
