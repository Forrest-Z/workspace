# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bruno/Newstage/Stage-4.0.0-src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bruno/Newstage/Stage-4.0.0-src

# Include any dependencies generated for this target.
include examples/ctrl/CMakeFiles/fasr.dir/depend.make

# Include the progress variables for this target.
include examples/ctrl/CMakeFiles/fasr.dir/progress.make

# Include the compile flags for this target's objects.
include examples/ctrl/CMakeFiles/fasr.dir/flags.make

examples/ctrl/CMakeFiles/fasr.dir/fasr.o: examples/ctrl/CMakeFiles/fasr.dir/flags.make
examples/ctrl/CMakeFiles/fasr.dir/fasr.o: examples/ctrl/fasr.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/Newstage/Stage-4.0.0-src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object examples/ctrl/CMakeFiles/fasr.dir/fasr.o"
	cd /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/include/freetype2   -D_THREAD_SAFE -D_REENTRANT -o CMakeFiles/fasr.dir/fasr.o -c /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl/fasr.cc

examples/ctrl/CMakeFiles/fasr.dir/fasr.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fasr.dir/fasr.i"
	cd /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/include/freetype2   -D_THREAD_SAFE -D_REENTRANT -E /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl/fasr.cc > CMakeFiles/fasr.dir/fasr.i

examples/ctrl/CMakeFiles/fasr.dir/fasr.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fasr.dir/fasr.s"
	cd /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/include/freetype2   -D_THREAD_SAFE -D_REENTRANT -S /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl/fasr.cc -o CMakeFiles/fasr.dir/fasr.s

examples/ctrl/CMakeFiles/fasr.dir/fasr.o.requires:
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/fasr.o.requires

examples/ctrl/CMakeFiles/fasr.dir/fasr.o.provides: examples/ctrl/CMakeFiles/fasr.dir/fasr.o.requires
	$(MAKE) -f examples/ctrl/CMakeFiles/fasr.dir/build.make examples/ctrl/CMakeFiles/fasr.dir/fasr.o.provides.build
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/fasr.o.provides

examples/ctrl/CMakeFiles/fasr.dir/fasr.o.provides.build: examples/ctrl/CMakeFiles/fasr.dir/fasr.o

# Object files for target fasr
fasr_OBJECTS = \
"CMakeFiles/fasr.dir/fasr.o"

# External object files for target fasr
fasr_EXTERNAL_OBJECTS =

examples/ctrl/fasr.so: examples/ctrl/CMakeFiles/fasr.dir/fasr.o
examples/ctrl/fasr.so: libstage/libstage.so.4.0.0
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libGLU.so
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libGL.so
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libSM.so
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libICE.so
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libX11.so
examples/ctrl/fasr.so: /usr/lib/i386-linux-gnu/libXext.so
examples/ctrl/fasr.so: /usr/lib/libltdl.so
examples/ctrl/fasr.so: examples/ctrl/CMakeFiles/fasr.dir/build.make
examples/ctrl/fasr.so: examples/ctrl/CMakeFiles/fasr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared module fasr.so"
	cd /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fasr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/ctrl/CMakeFiles/fasr.dir/build: examples/ctrl/fasr.so
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/build

examples/ctrl/CMakeFiles/fasr.dir/requires: examples/ctrl/CMakeFiles/fasr.dir/fasr.o.requires
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/requires

examples/ctrl/CMakeFiles/fasr.dir/clean:
	cd /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl && $(CMAKE_COMMAND) -P CMakeFiles/fasr.dir/cmake_clean.cmake
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/clean

examples/ctrl/CMakeFiles/fasr.dir/depend:
	cd /home/bruno/Newstage/Stage-4.0.0-src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/Newstage/Stage-4.0.0-src /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl /home/bruno/Newstage/Stage-4.0.0-src /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl /home/bruno/Newstage/Stage-4.0.0-src/examples/ctrl/CMakeFiles/fasr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/ctrl/CMakeFiles/fasr.dir/depend

