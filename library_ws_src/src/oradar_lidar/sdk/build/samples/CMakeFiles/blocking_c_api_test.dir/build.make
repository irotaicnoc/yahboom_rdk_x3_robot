# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /userdata/software/oradar_ws/src/oradar_lidar/sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /userdata/software/oradar_ws/src/oradar_lidar/sdk/build

# Include any dependencies generated for this target.
include samples/CMakeFiles/blocking_c_api_test.dir/depend.make

# Include the progress variables for this target.
include samples/CMakeFiles/blocking_c_api_test.dir/progress.make

# Include the compile flags for this target's objects.
include samples/CMakeFiles/blocking_c_api_test.dir/flags.make

samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o: samples/CMakeFiles/blocking_c_api_test.dir/flags.make
samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o: ../samples/blocking_c_api_test.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/userdata/software/oradar_ws/src/oradar_lidar/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o"
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o   -c /userdata/software/oradar_ws/src/oradar_lidar/sdk/samples/blocking_c_api_test.c

samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.i"
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /userdata/software/oradar_ws/src/oradar_lidar/sdk/samples/blocking_c_api_test.c > CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.i

samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.s"
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /userdata/software/oradar_ws/src/oradar_lidar/sdk/samples/blocking_c_api_test.c -o CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.s

# Object files for target blocking_c_api_test
blocking_c_api_test_OBJECTS = \
"CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o"

# External object files for target blocking_c_api_test
blocking_c_api_test_EXTERNAL_OBJECTS =

blocking_c_api_test: samples/CMakeFiles/blocking_c_api_test.dir/blocking_c_api_test.c.o
blocking_c_api_test: samples/CMakeFiles/blocking_c_api_test.dir/build.make
blocking_c_api_test: liboradar_sdk.a
blocking_c_api_test: samples/CMakeFiles/blocking_c_api_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/userdata/software/oradar_ws/src/oradar_lidar/sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../blocking_c_api_test"
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blocking_c_api_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/CMakeFiles/blocking_c_api_test.dir/build: blocking_c_api_test

.PHONY : samples/CMakeFiles/blocking_c_api_test.dir/build

samples/CMakeFiles/blocking_c_api_test.dir/clean:
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples && $(CMAKE_COMMAND) -P CMakeFiles/blocking_c_api_test.dir/cmake_clean.cmake
.PHONY : samples/CMakeFiles/blocking_c_api_test.dir/clean

samples/CMakeFiles/blocking_c_api_test.dir/depend:
	cd /userdata/software/oradar_ws/src/oradar_lidar/sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /userdata/software/oradar_ws/src/oradar_lidar/sdk /userdata/software/oradar_ws/src/oradar_lidar/sdk/samples /userdata/software/oradar_ws/src/oradar_lidar/sdk/build /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples /userdata/software/oradar_ws/src/oradar_lidar/sdk/build/samples/CMakeFiles/blocking_c_api_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/CMakeFiles/blocking_c_api_test.dir/depend

