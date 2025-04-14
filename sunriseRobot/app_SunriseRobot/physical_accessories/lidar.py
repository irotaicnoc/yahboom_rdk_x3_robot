import ctypes
import os

# Define the path to the shared library
project_root = os.path.dirname(os.path.abspath(__file__))  # Adjust to your project root
print(f'Project root: {project_root}')
library_path = os.path.join(project_root, "../../library_ws_src/src/oradar_lidar/sdk/build/liboradar_sdk.so")
print(f'Library path: {library_path}')

# Load the shared library
try:
    lidar_lib = ctypes.CDLL(library_path)
    print("Lidar library loaded successfully.")
except OSError as e:
    print(f"Failed to load the lidar library: {e}")

# Example: Define a function from the library (adjust based on the actual API)
# lidar_lib.some_function.argtypes = [ctypes.c_int, ctypes.c_double]
# lidar_lib.some_function.restype = ctypes.c_int

# Replace some_function with the actual function names from the library.
# Define the argument and return types for each function you want to use.
# Ensure the shared library is in the correct path relative to your Python script.