import ctypes

import global_constants as gc


print(f'Library path: {gc.LIDAR_LIB_PATH}')
# Load the shared library
try:
    lidar_lib = ctypes.CDLL(gc.LIDAR_LIB_PATH)
    print("Lidar library loaded successfully.")
except OSError as e:
    print(f"Failed to load the lidar library: {e}")

# Example: Define a function from the library (adjust based on the actual API)
# lidar_lib.some_function.argtypes = [ctypes.c_int, ctypes.c_double]
# lidar_lib.some_function.restype = ctypes.c_int

# Replace some_function with the actual function names from the library.
# Define the argument and return types for each function you want to use.
# Ensure the shared library is in the correct path relative to your Python script.
