# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.27.7-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.27.7-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sj/Desktop/VR_ARM/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sj/Desktop/VR_ARM/build

# Utility rule file for doxygen.

# Include any custom commands dependencies for this target.
include dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/compiler_depend.make

# Include the progress variables for this target.
include dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/progress.make

doxygen: dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/build.make
.PHONY : doxygen

# Rule to build all files generated by this target.
dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/build: doxygen
.PHONY : dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/build

dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/clean:
	cd /home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk && $(CMAKE_COMMAND) -P CMakeFiles/doxygen.dir/cmake_clean.cmake
.PHONY : dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/clean

dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/depend:
	cd /home/sj/Desktop/VR_ARM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sj/Desktop/VR_ARM/src /home/sj/Desktop/VR_ARM/src/dependencies/dynamixel_sdk /home/sj/Desktop/VR_ARM/build /home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk /home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : dependencies/dynamixel_sdk/CMakeFiles/doxygen.dir/depend

