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
CMAKE_SOURCE_DIR = /home/sandun/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sandun/catkin_ws/build

# Include any dependencies generated for this target.
include iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/depend.make

# Include the progress variables for this target.
include iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/progress.make

# Include the compile flags for this target's objects.
include iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/flags.make

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/flags.make
iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o: /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/kinect2_registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sandun/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o -c /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/kinect2_registration.cpp

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/kinect2_registration.cpp > CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.i

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/kinect2_registration.cpp -o CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.s

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/flags.make
iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o: /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_cpu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sandun/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o -c /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_cpu.cpp

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_cpu.cpp > CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.i

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_cpu.cpp -o CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.s

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/flags.make
iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o: /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_opencl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sandun/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o -c /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_opencl.cpp

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_opencl.cpp > CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.i

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration/src/depth_registration_opencl.cpp -o CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.s

# Object files for target kinect2_registration
kinect2_registration_OBJECTS = \
"CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o" \
"CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o" \
"CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o"

# External object files for target kinect2_registration
kinect2_registration_EXTERNAL_OBJECTS =

/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/kinect2_registration.cpp.o
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_cpu.cpp.o
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/src/depth_registration_opencl.cpp.o
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/build.make
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/libroscpp.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/librosconsole.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/librostime.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /opt/ros/noetic/lib/libcpp_common.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libOpenCL.so
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/sandun/catkin_ws/devel/lib/libkinect2_registration.so: iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sandun/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/sandun/catkin_ws/devel/lib/libkinect2_registration.so"
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect2_registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/build: /home/sandun/catkin_ws/devel/lib/libkinect2_registration.so

.PHONY : iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/build

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/clean:
	cd /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration && $(CMAKE_COMMAND) -P CMakeFiles/kinect2_registration.dir/cmake_clean.cmake
.PHONY : iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/clean

iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/depend:
	cd /home/sandun/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sandun/catkin_ws/src /home/sandun/catkin_ws/src/iai_kinect2_opencv4/kinect2_registration /home/sandun/catkin_ws/build /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration /home/sandun/catkin_ws/build/iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : iai_kinect2_opencv4/kinect2_registration/CMakeFiles/kinect2_registration.dir/depend

