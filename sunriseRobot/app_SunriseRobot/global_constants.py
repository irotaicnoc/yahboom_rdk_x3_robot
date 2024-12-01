# FAN AND INTERNAL LIGHTS
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
ALL_LIGHTS_ID = 0xff

# OS PATHS
MAIN_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/'
APP_FOLDER_PATH = '/root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/'
CONFIG_FOLDER_PATH = APP_FOLDER_PATH + 'configs/'

# ALIAS
CPU_DEVICE = 'cpu'
TPU_DEVICE = 'edgetpu'
