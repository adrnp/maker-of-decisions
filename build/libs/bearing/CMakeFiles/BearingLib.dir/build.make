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
include libs/bearing/CMakeFiles/BearingLib.dir/depend.make

# Include the progress variables for this target.
include libs/bearing/CMakeFiles/BearingLib.dir/progress.make

# Include the compile flags for this target's objects.
include libs/bearing/CMakeFiles/BearingLib.dir/flags.make

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o: libs/bearing/CMakeFiles/BearingLib.dir/flags.make
libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o: ../libs/bearing/bearing_cc.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BearingLib.dir/bearing_cc.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_cc.cpp

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BearingLib.dir/bearing_cc.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_cc.cpp > CMakeFiles/BearingLib.dir/bearing_cc.cpp.i

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BearingLib.dir/bearing_cc.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_cc.cpp -o CMakeFiles/BearingLib.dir/bearing_cc.cpp.s

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.requires:
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.requires

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.provides: libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.requires
	$(MAKE) -f libs/bearing/CMakeFiles/BearingLib.dir/build.make libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.provides.build
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.provides

libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.provides.build: libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o: libs/bearing/CMakeFiles/BearingLib.dir/flags.make
libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o: ../libs/bearing/bearing_helper.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BearingLib.dir/bearing_helper.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_helper.cpp

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BearingLib.dir/bearing_helper.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_helper.cpp > CMakeFiles/BearingLib.dir/bearing_helper.cpp.i

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BearingLib.dir/bearing_helper.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_helper.cpp -o CMakeFiles/BearingLib.dir/bearing_helper.cpp.s

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.requires:
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.requires

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.provides: libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.requires
	$(MAKE) -f libs/bearing/CMakeFiles/BearingLib.dir/build.make libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.provides.build
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.provides

libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.provides.build: libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o: libs/bearing/CMakeFiles/BearingLib.dir/flags.make
libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o: ../libs/bearing/bearing_max.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BearingLib.dir/bearing_max.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_max.cpp

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BearingLib.dir/bearing_max.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_max.cpp > CMakeFiles/BearingLib.dir/bearing_max.cpp.i

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BearingLib.dir/bearing_max.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_max.cpp -o CMakeFiles/BearingLib.dir/bearing_max.cpp.s

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.requires:
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.requires

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.provides: libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.requires
	$(MAKE) -f libs/bearing/CMakeFiles/BearingLib.dir/build.make libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.provides.build
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.provides

libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.provides.build: libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o: libs/bearing/CMakeFiles/BearingLib.dir/flags.make
libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o: ../libs/bearing/bearing_mle.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/adrienp/git-uav/maker-of-decisions/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BearingLib.dir/bearing_mle.cpp.o -c /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_mle.cpp

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BearingLib.dir/bearing_mle.cpp.i"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_mle.cpp > CMakeFiles/BearingLib.dir/bearing_mle.cpp.i

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BearingLib.dir/bearing_mle.cpp.s"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/adrienp/git-uav/maker-of-decisions/libs/bearing/bearing_mle.cpp -o CMakeFiles/BearingLib.dir/bearing_mle.cpp.s

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.requires:
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.requires

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.provides: libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.requires
	$(MAKE) -f libs/bearing/CMakeFiles/BearingLib.dir/build.make libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.provides.build
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.provides

libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.provides.build: libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o

# Object files for target BearingLib
BearingLib_OBJECTS = \
"CMakeFiles/BearingLib.dir/bearing_cc.cpp.o" \
"CMakeFiles/BearingLib.dir/bearing_helper.cpp.o" \
"CMakeFiles/BearingLib.dir/bearing_max.cpp.o" \
"CMakeFiles/BearingLib.dir/bearing_mle.cpp.o"

# External object files for target BearingLib
BearingLib_EXTERNAL_OBJECTS =

libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o
libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o
libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o
libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o
libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/build.make
libs/bearing/libBearingLib.a: libs/bearing/CMakeFiles/BearingLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libBearingLib.a"
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && $(CMAKE_COMMAND) -P CMakeFiles/BearingLib.dir/cmake_clean_target.cmake
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BearingLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/bearing/CMakeFiles/BearingLib.dir/build: libs/bearing/libBearingLib.a
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/build

libs/bearing/CMakeFiles/BearingLib.dir/requires: libs/bearing/CMakeFiles/BearingLib.dir/bearing_cc.cpp.o.requires
libs/bearing/CMakeFiles/BearingLib.dir/requires: libs/bearing/CMakeFiles/BearingLib.dir/bearing_helper.cpp.o.requires
libs/bearing/CMakeFiles/BearingLib.dir/requires: libs/bearing/CMakeFiles/BearingLib.dir/bearing_max.cpp.o.requires
libs/bearing/CMakeFiles/BearingLib.dir/requires: libs/bearing/CMakeFiles/BearingLib.dir/bearing_mle.cpp.o.requires
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/requires

libs/bearing/CMakeFiles/BearingLib.dir/clean:
	cd /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing && $(CMAKE_COMMAND) -P CMakeFiles/BearingLib.dir/cmake_clean.cmake
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/clean

libs/bearing/CMakeFiles/BearingLib.dir/depend:
	cd /home/adrienp/git-uav/maker-of-decisions/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrienp/git-uav/maker-of-decisions /home/adrienp/git-uav/maker-of-decisions/libs/bearing /home/adrienp/git-uav/maker-of-decisions/build /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing /home/adrienp/git-uav/maker-of-decisions/build/libs/bearing/CMakeFiles/BearingLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/bearing/CMakeFiles/BearingLib.dir/depend

