# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/teo/git/ColorDebug/example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teo/git/ColorDebug/example/build

# Include any dependencies generated for this target.
include CMakeFiles/testColorDebug.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testColorDebug.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testColorDebug.dir/flags.make

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o: CMakeFiles/testColorDebug.dir/flags.make
CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o: ../testColorDebug.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/teo/git/ColorDebug/example/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o -c /home/teo/git/ColorDebug/example/testColorDebug.cpp

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testColorDebug.dir/testColorDebug.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/teo/git/ColorDebug/example/testColorDebug.cpp > CMakeFiles/testColorDebug.dir/testColorDebug.cpp.i

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testColorDebug.dir/testColorDebug.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/teo/git/ColorDebug/example/testColorDebug.cpp -o CMakeFiles/testColorDebug.dir/testColorDebug.cpp.s

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.requires:
.PHONY : CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.requires

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.provides: CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.requires
	$(MAKE) -f CMakeFiles/testColorDebug.dir/build.make CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.provides.build
.PHONY : CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.provides

CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.provides.build: CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o

# Object files for target testColorDebug
testColorDebug_OBJECTS = \
"CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o"

# External object files for target testColorDebug
testColorDebug_EXTERNAL_OBJECTS =

testColorDebug: CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o
testColorDebug: CMakeFiles/testColorDebug.dir/build.make
testColorDebug: CMakeFiles/testColorDebug.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable testColorDebug"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testColorDebug.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testColorDebug.dir/build: testColorDebug
.PHONY : CMakeFiles/testColorDebug.dir/build

CMakeFiles/testColorDebug.dir/requires: CMakeFiles/testColorDebug.dir/testColorDebug.cpp.o.requires
.PHONY : CMakeFiles/testColorDebug.dir/requires

CMakeFiles/testColorDebug.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testColorDebug.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testColorDebug.dir/clean

CMakeFiles/testColorDebug.dir/depend:
	cd /home/teo/git/ColorDebug/example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teo/git/ColorDebug/example /home/teo/git/ColorDebug/example /home/teo/git/ColorDebug/example/build /home/teo/git/ColorDebug/example/build /home/teo/git/ColorDebug/example/build/CMakeFiles/testColorDebug.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testColorDebug.dir/depend
