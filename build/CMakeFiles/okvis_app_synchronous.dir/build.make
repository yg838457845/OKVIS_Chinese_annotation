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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dnc/okvis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dnc/okvis/build

# Include any dependencies generated for this target.
include CMakeFiles/okvis_app_synchronous.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/okvis_app_synchronous.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/okvis_app_synchronous.dir/flags.make

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o: CMakeFiles/okvis_app_synchronous.dir/flags.make
CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o: ../okvis_apps/src/okvis_app_synchronous.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dnc/okvis/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o -c /home/dnc/okvis/okvis_apps/src/okvis_app_synchronous.cpp

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dnc/okvis/okvis_apps/src/okvis_app_synchronous.cpp > CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.i

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dnc/okvis/okvis_apps/src/okvis_app_synchronous.cpp -o CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.s

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.requires:
.PHONY : CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.requires

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.provides: CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.requires
	$(MAKE) -f CMakeFiles/okvis_app_synchronous.dir/build.make CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.provides.build
.PHONY : CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.provides

CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.provides.build: CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o

# Object files for target okvis_app_synchronous
okvis_app_synchronous_OBJECTS = \
"CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o"

# External object files for target okvis_app_synchronous
okvis_app_synchronous_EXTERNAL_OBJECTS =

okvis_app_synchronous: CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o
okvis_app_synchronous: CMakeFiles/okvis_app_synchronous.dir/build.make
okvis_app_synchronous: okvis_util/libokvis_util.a
okvis_app_synchronous: okvis_kinematics/libokvis_kinematics.a
okvis_app_synchronous: okvis_time/libokvis_time.a
okvis_app_synchronous: okvis_cv/libokvis_cv.a
okvis_app_synchronous: okvis_common/libokvis_common.a
okvis_app_synchronous: okvis_ceres/libokvis_ceres.a
okvis_app_synchronous: okvis_timing/libokvis_timing.a
okvis_app_synchronous: okvis_matcher/libokvis_matcher.a
okvis_app_synchronous: okvis_frontend/libokvis_frontend.a
okvis_app_synchronous: okvis_multisensor_processing/libokvis_multisensor_processing.a
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libboost_system.so
okvis_app_synchronous: okvis_frontend/libokvis_frontend.a
okvis_app_synchronous: okvis_ceres/libokvis_ceres.a
okvis_app_synchronous: okvis_common/libokvis_common.a
okvis_app_synchronous: okvis_cv/libokvis_cv.a
okvis_app_synchronous: okvis_kinematics/libokvis_kinematics.a
okvis_app_synchronous: okvis_time/libokvis_time.a
okvis_app_synchronous: /usr/local/lib/libopencv_features2d.so.2.4.11
okvis_app_synchronous: /usr/local/lib/libopencv_highgui.so.2.4.11
okvis_app_synchronous: /usr/local/lib/libopencv_imgproc.so.2.4.11
okvis_app_synchronous: /usr/local/lib/libopencv_flann.so.2.4.11
okvis_app_synchronous: /usr/local/lib/libopencv_core.so.2.4.11
okvis_app_synchronous: okvis_timing/libokvis_timing.a
okvis_app_synchronous: okvis_matcher/libokvis_matcher.a
okvis_app_synchronous: okvis_util/libokvis_util.a
okvis_app_synchronous: lib/libbrisk.a
okvis_app_synchronous: lib/libagast.a
okvis_app_synchronous: lib/libceres.a
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libspqr.so
okvis_app_synchronous: /usr/lib/libtbb.so
okvis_app_synchronous: /usr/lib/libtbbmalloc.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libcholmod.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libccolamd.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libcamd.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libcolamd.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libamd.so
okvis_app_synchronous: /usr/lib/liblapack.so
okvis_app_synchronous: /usr/lib/libf77blas.so
okvis_app_synchronous: /usr/lib/libatlas.so
okvis_app_synchronous: /usr/lib/libf77blas.so
okvis_app_synchronous: /usr/lib/libatlas.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.a
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/librt.so
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libcxsparse.so
okvis_app_synchronous: lib/libopengv.a
okvis_app_synchronous: /usr/lib/x86_64-linux-gnu/libglog.so
okvis_app_synchronous: CMakeFiles/okvis_app_synchronous.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable okvis_app_synchronous"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/okvis_app_synchronous.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/okvis_app_synchronous.dir/build: okvis_app_synchronous
.PHONY : CMakeFiles/okvis_app_synchronous.dir/build

CMakeFiles/okvis_app_synchronous.dir/requires: CMakeFiles/okvis_app_synchronous.dir/okvis_apps/src/okvis_app_synchronous.cpp.o.requires
.PHONY : CMakeFiles/okvis_app_synchronous.dir/requires

CMakeFiles/okvis_app_synchronous.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/okvis_app_synchronous.dir/cmake_clean.cmake
.PHONY : CMakeFiles/okvis_app_synchronous.dir/clean

CMakeFiles/okvis_app_synchronous.dir/depend:
	cd /home/dnc/okvis/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dnc/okvis /home/dnc/okvis /home/dnc/okvis/build /home/dnc/okvis/build /home/dnc/okvis/build/CMakeFiles/okvis_app_synchronous.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/okvis_app_synchronous.dir/depend

