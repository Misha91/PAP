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
CMAKE_COMMAND = /opt/cmake-3.16.2-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.16.2-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mikhail/Desktop/dev/PAP/02/particle_filter_init

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build

# Include any dependencies generated for this target.
include laserSimulator/CMakeFiles/lasersimulator.dir/depend.make

# Include the progress variables for this target.
include laserSimulator/CMakeFiles/lasersimulator.dir/progress.make

# Include the compile flags for this target's objects.
include laserSimulator/CMakeFiles/lasersimulator.dir/flags.make

laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o: laserSimulator/CMakeFiles/lasersimulator.dir/flags.make
laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o: ../laserSimulator/lasersimulator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o"
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o -c /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/laserSimulator/lasersimulator.cpp

laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lasersimulator.dir/lasersimulator.cpp.i"
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/laserSimulator/lasersimulator.cpp > CMakeFiles/lasersimulator.dir/lasersimulator.cpp.i

laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lasersimulator.dir/lasersimulator.cpp.s"
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/laserSimulator/lasersimulator.cpp -o CMakeFiles/lasersimulator.dir/lasersimulator.cpp.s

# Object files for target lasersimulator
lasersimulator_OBJECTS = \
"CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o"

# External object files for target lasersimulator
lasersimulator_EXTERNAL_OBJECTS =

laserSimulator/liblasersimulator.a: laserSimulator/CMakeFiles/lasersimulator.dir/lasersimulator.cpp.o
laserSimulator/liblasersimulator.a: laserSimulator/CMakeFiles/lasersimulator.dir/build.make
laserSimulator/liblasersimulator.a: laserSimulator/CMakeFiles/lasersimulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblasersimulator.a"
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && $(CMAKE_COMMAND) -P CMakeFiles/lasersimulator.dir/cmake_clean_target.cmake
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lasersimulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
laserSimulator/CMakeFiles/lasersimulator.dir/build: laserSimulator/liblasersimulator.a

.PHONY : laserSimulator/CMakeFiles/lasersimulator.dir/build

laserSimulator/CMakeFiles/lasersimulator.dir/clean:
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator && $(CMAKE_COMMAND) -P CMakeFiles/lasersimulator.dir/cmake_clean.cmake
.PHONY : laserSimulator/CMakeFiles/lasersimulator.dir/clean

laserSimulator/CMakeFiles/lasersimulator.dir/depend:
	cd /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mikhail/Desktop/dev/PAP/02/particle_filter_init /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/laserSimulator /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator /home/mikhail/Desktop/dev/PAP/02/particle_filter_init/build/laserSimulator/CMakeFiles/lasersimulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : laserSimulator/CMakeFiles/lasersimulator.dir/depend
