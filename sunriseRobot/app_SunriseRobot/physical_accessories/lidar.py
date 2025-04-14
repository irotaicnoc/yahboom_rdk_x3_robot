import ctypes

import global_constants as gc


ORADAR_MS200 = 1
ORADAR_TYPE_SERIAL = 0x0


class lidar_python():
    """
    Load the Lidar library using ctypes.
    """

    def __init__(self):
        # Load the shared library
        try:
            lidar_lib = ctypes.CDLL(gc.LIDAR_LIB_PATH)
            print("Lidar library loaded successfully.")
            # Set the argument and return types for the Init function
            self.lidar = lidar_lib.OrdlidarDriver(ORADAR_TYPE_SERIAL, ORADAR_MS200)
            self.lidar.SetSerialPort('/dev/ttyACM0', 230400)

        except OSError as e:
            print(f"Failed to load the lidar library: {e}")
            raise Exception('Failed to open lidar.')
