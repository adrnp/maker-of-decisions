# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/adrienp/git-uav/maker-of-decisions

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adrienp/git-uav/maker-of-decisions/build

# Include any dependencies generated for this target.
include libs/serial/CMakeFiles/SerialLib.dir/depend.make

# Include the progress variables for this target.
include libs/serial/CMakeFiles/SerialLib.dir/progress.make

# Include the compile flags for this target's objects.
include libs/serial/CMakeFiles/SerialLib.dir/flags.make

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o: libs/serial/CMakeFiles/SerialLib.dir/flags.make
libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o: ../libs/serial/mavlink_serial.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/serial/mavlink_serial.cpp

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialLib.dir/mavlink_serial.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/serial/mavlink_serial.cpp > CMakeFiles/SerialLib.dir/mavlink_serial.cpp.i

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialLib.dir/mavlink_serial.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/serial/mavlink_serial.cpp -o CMakeFiles/SerialLib.dir/mavlink_serial.cpp.s

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.requires:
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.requires

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.provides: libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.requires
	$(MAKE) -f libs/serial/CMakeFiles/SerialLib.dir/build.make libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.provides.build
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.provides

libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.provides.build: libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o: libs/serial/CMakeFiles/SerialLib.dir/flags.make
libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o: ../libs/serial/serial_port.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/SerialLib.dir/serial_port.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/serial/serial_port.cpp

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialLib.dir/serial_port.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/serial/serial_port.cpp > CMakeFiles/SerialLib.dir/serial_port.cpp.i

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialLib.dir/serial_port.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/serial/serial_port.cpp -o CMakeFiles/SerialLib.dir/serial_port.cpp.s

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.requires:
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.requires

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.provides: libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.requires
	$(MAKE) -f libs/serial/CMakeFiles/SerialLib.dir/build.make libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.provides.build
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.provides

libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.provides.build: libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o: libs/serial/CMakeFiles/SerialLib.dir/flags.make
libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o: ../libs/serial/wifly_serial.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/SerialLib.dir/wifly_serial.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/serial/wifly_serial.cpp

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialLib.dir/wifly_serial.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/serial/wifly_serial.cpp > CMakeFiles/SerialLib.dir/wifly_serial.cpp.i

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialLib.dir/wifly_serial.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/serial/wifly_serial.cpp -o CMakeFiles/SerialLib.dir/wifly_serial.cpp.s

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.requires:
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.requires

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.provides: libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.requires
	$(MAKE) -f libs/serial/CMakeFiles/SerialLib.dir/build.make libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.provides.build
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.provides

libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.provides.build: libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o

# Object files for target SerialLib
SerialLib_OBJECTS = \
"CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o" \
"CMakeFiles/SerialLib.dir/serial_port.cpp.o" \
"CMakeFiles/SerialLib.dir/wifly_serial.cpp.o"

# External object files for target SerialLib
SerialLib_EXTERNAL_OBJECTS =

libs/serial/libSerialLib.a: libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o
libs/serial/libSerialLib.a: libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o
libs/serial/libSerialLib.a: libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o
libs/serial/libSerialLib.a: libs/serial/CMakeFiles/SerialLib.dir/build.make
libs/serial/libSerialLib.a: libs/serial/CMakeFiles/SerialLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libSerialLib.a"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && $(CMAKE_COMMAND) -P CMakeFiles/SerialLib.dir/cmake_clean_target.cmake
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/serial/CMakeFiles/SerialLib.dir/build: libs/serial/libSerialLib.a
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/build

libs/serial/CMakeFiles/SerialLib.dir/requires: libs/serial/CMakeFiles/SerialLib.dir/mavlink_serial.cpp.o.requires
libs/serial/CMakeFiles/SerialLib.dir/requires: libs/serial/CMakeFiles/SerialLib.dir/serial_port.cpp.o.requires
libs/serial/CMakeFiles/SerialLib.dir/requires: libs/serial/CMakeFiles/SerialLib.dir/wifly_serial.cpp.o.requires
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/requires

libs/serial/CMakeFiles/SerialLib.dir/clean:
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/serial && $(CMAKE_COMMAND) -P CMakeFiles/SerialLib.dir/cmake_clean.cmake
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/clean

libs/serial/CMakeFiles/SerialLib.dir/depend:
	cd /home/adrienp/git-uav/maker-of-decisions/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrienp/git-uav/maker-of-decisions /home/adrienp/git-uav/maker-of-decisions/libs/serial /home/adrienp/git-uav/maker-of-decisions/build /home/adrienp/git-uav/maker-of-decisions/build/libs/serial /home/adrienp/git-uav/maker-of-decisions/build/libs/serial/CMakeFiles/SerialLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/serial/CMakeFiles/SerialLib.dir/depend

