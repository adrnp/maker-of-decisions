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
include CMakeFiles/mod.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mod.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mod.dir/flags.make

CMakeFiles/mod.dir/mod.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/mod.cpp.o: ../mod.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/mod.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/mod.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/mod.cpp

CMakeFiles/mod.dir/mod.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/mod.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/mod.cpp > CMakeFiles/mod.dir/mod.cpp.i

CMakeFiles/mod.dir/mod.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/mod.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/mod.cpp -o CMakeFiles/mod.dir/mod.cpp.s

CMakeFiles/mod.dir/mod.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/mod.cpp.o.requires

CMakeFiles/mod.dir/mod.cpp.o.provides: CMakeFiles/mod.dir/mod.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/mod.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/mod.cpp.o.provides

CMakeFiles/mod.dir/mod.cpp.o.provides.build: CMakeFiles/mod.dir/mod.cpp.o

CMakeFiles/mod.dir/wifly2_thread.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/wifly2_thread.cpp.o: ../wifly2_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/wifly2_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/wifly2_thread.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/wifly2_thread.cpp

CMakeFiles/mod.dir/wifly2_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/wifly2_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/wifly2_thread.cpp > CMakeFiles/mod.dir/wifly2_thread.cpp.i

CMakeFiles/mod.dir/wifly2_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/wifly2_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/wifly2_thread.cpp -o CMakeFiles/mod.dir/wifly2_thread.cpp.s

CMakeFiles/mod.dir/wifly2_thread.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/wifly2_thread.cpp.o.requires

CMakeFiles/mod.dir/wifly2_thread.cpp.o.provides: CMakeFiles/mod.dir/wifly2_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/wifly2_thread.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/wifly2_thread.cpp.o.provides

CMakeFiles/mod.dir/wifly2_thread.cpp.o.provides.build: CMakeFiles/mod.dir/wifly2_thread.cpp.o

CMakeFiles/mod.dir/dirk_thread.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/dirk_thread.cpp.o: ../dirk_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/dirk_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/dirk_thread.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/dirk_thread.cpp

CMakeFiles/mod.dir/dirk_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/dirk_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/dirk_thread.cpp > CMakeFiles/mod.dir/dirk_thread.cpp.i

CMakeFiles/mod.dir/dirk_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/dirk_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/dirk_thread.cpp -o CMakeFiles/mod.dir/dirk_thread.cpp.s

CMakeFiles/mod.dir/dirk_thread.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/dirk_thread.cpp.o.requires

CMakeFiles/mod.dir/dirk_thread.cpp.o.provides: CMakeFiles/mod.dir/dirk_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/dirk_thread.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/dirk_thread.cpp.o.provides

CMakeFiles/mod.dir/dirk_thread.cpp.o.provides.build: CMakeFiles/mod.dir/dirk_thread.cpp.o

CMakeFiles/mod.dir/commander.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/commander.cpp.o: ../commander.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/commander.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/commander.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/commander.cpp

CMakeFiles/mod.dir/commander.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/commander.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/commander.cpp > CMakeFiles/mod.dir/commander.cpp.i

CMakeFiles/mod.dir/commander.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/commander.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/commander.cpp -o CMakeFiles/mod.dir/commander.cpp.s

CMakeFiles/mod.dir/commander.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/commander.cpp.o.requires

CMakeFiles/mod.dir/commander.cpp.o.provides: CMakeFiles/mod.dir/commander.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/commander.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/commander.cpp.o.provides

CMakeFiles/mod.dir/commander.cpp.o.provides.build: CMakeFiles/mod.dir/commander.cpp.o

CMakeFiles/mod.dir/read_thread.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/read_thread.cpp.o: ../read_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/read_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/read_thread.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/read_thread.cpp

CMakeFiles/mod.dir/read_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/read_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/read_thread.cpp > CMakeFiles/mod.dir/read_thread.cpp.i

CMakeFiles/mod.dir/read_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/read_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/read_thread.cpp -o CMakeFiles/mod.dir/read_thread.cpp.s

CMakeFiles/mod.dir/read_thread.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/read_thread.cpp.o.requires

CMakeFiles/mod.dir/read_thread.cpp.o.provides: CMakeFiles/mod.dir/read_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/read_thread.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/read_thread.cpp.o.provides

CMakeFiles/mod.dir/read_thread.cpp.o.provides.build: CMakeFiles/mod.dir/read_thread.cpp.o

CMakeFiles/mod.dir/wifly_thread.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/wifly_thread.cpp.o: ../wifly_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/wifly_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/wifly_thread.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/wifly_thread.cpp

CMakeFiles/mod.dir/wifly_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/wifly_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/wifly_thread.cpp > CMakeFiles/mod.dir/wifly_thread.cpp.i

CMakeFiles/mod.dir/wifly_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/wifly_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/wifly_thread.cpp -o CMakeFiles/mod.dir/wifly_thread.cpp.s

CMakeFiles/mod.dir/wifly_thread.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/wifly_thread.cpp.o.requires

CMakeFiles/mod.dir/wifly_thread.cpp.o.provides: CMakeFiles/mod.dir/wifly_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/wifly_thread.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/wifly_thread.cpp.o.provides

CMakeFiles/mod.dir/wifly_thread.cpp.o.provides.build: CMakeFiles/mod.dir/wifly_thread.cpp.o

CMakeFiles/mod.dir/tracker.cpp.o: CMakeFiles/mod.dir/flags.make
CMakeFiles/mod.dir/tracker.cpp.o: ../tracker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mod.dir/tracker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mod.dir/tracker.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/tracker.cpp

CMakeFiles/mod.dir/tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mod.dir/tracker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/tracker.cpp > CMakeFiles/mod.dir/tracker.cpp.i

CMakeFiles/mod.dir/tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mod.dir/tracker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/tracker.cpp -o CMakeFiles/mod.dir/tracker.cpp.s

CMakeFiles/mod.dir/tracker.cpp.o.requires:
.PHONY : CMakeFiles/mod.dir/tracker.cpp.o.requires

CMakeFiles/mod.dir/tracker.cpp.o.provides: CMakeFiles/mod.dir/tracker.cpp.o.requires
	$(MAKE) -f CMakeFiles/mod.dir/build.make CMakeFiles/mod.dir/tracker.cpp.o.provides.build
.PHONY : CMakeFiles/mod.dir/tracker.cpp.o.provides

CMakeFiles/mod.dir/tracker.cpp.o.provides.build: CMakeFiles/mod.dir/tracker.cpp.o

# Object files for target mod
mod_OBJECTS = \
"CMakeFiles/mod.dir/mod.cpp.o" \
"CMakeFiles/mod.dir/wifly2_thread.cpp.o" \
"CMakeFiles/mod.dir/dirk_thread.cpp.o" \
"CMakeFiles/mod.dir/commander.cpp.o" \
"CMakeFiles/mod.dir/read_thread.cpp.o" \
"CMakeFiles/mod.dir/wifly_thread.cpp.o" \
"CMakeFiles/mod.dir/tracker.cpp.o"

# External object files for target mod
mod_EXTERNAL_OBJECTS =

mod: CMakeFiles/mod.dir/mod.cpp.o
mod: CMakeFiles/mod.dir/wifly2_thread.cpp.o
mod: CMakeFiles/mod.dir/dirk_thread.cpp.o
mod: CMakeFiles/mod.dir/commander.cpp.o
mod: CMakeFiles/mod.dir/read_thread.cpp.o
mod: CMakeFiles/mod.dir/wifly_thread.cpp.o
mod: CMakeFiles/mod.dir/tracker.cpp.o
mod: CMakeFiles/mod.dir/build.make
mod: libs/bearing/libBearingLib.a
mod: libs/serial/libSerialLib.a
mod: libs/udp/libUDPLib.a
mod: CMakeFiles/mod.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable mod"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mod.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mod.dir/build: mod
.PHONY : CMakeFiles/mod.dir/build

CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/mod.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/wifly2_thread.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/dirk_thread.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/commander.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/read_thread.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/wifly_thread.cpp.o.requires
CMakeFiles/mod.dir/requires: CMakeFiles/mod.dir/tracker.cpp.o.requires
.PHONY : CMakeFiles/mod.dir/requires

CMakeFiles/mod.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mod.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mod.dir/clean

CMakeFiles/mod.dir/depend:
	cd /home/adrienp/git-uav/maker-of-decisions/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrienp/git-uav/maker-of-decisions /home/adrienp/git-uav/maker-of-decisions /home/adrienp/git-uav/maker-of-decisions/build /home/adrienp/git-uav/maker-of-decisions/build /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles/mod.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mod.dir/depend

