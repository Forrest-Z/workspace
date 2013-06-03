cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init(RBBM)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/RBBMCartesian.cpp src/RBBM_radius.cpp src/RBBMCartesian_radius.cpp src/RBBM.cpp src/RBBM1Beam.cpp src/phit.cpp src/poccl.cpp)
rosbuild_add_executable(testRBBM181 src/testRBBM181.cpp src/RBBM.cpp src/RBBM1Beam.cpp src/phit.cpp src/poccl.cpp)
rosbuild_add_executable(testMeasModel src/testMeasModel.cpp src/RBBM.cpp src/RBBM1Beam.cpp src/phit.cpp src/poccl.cpp)
rosbuild_add_executable(testMeasModelCartesian src/testMeasModelCartesian.cpp src/RBBMCartesian.cpp src/RBBM1Beam.cpp src/phit.cpp src/poccl.cpp)
rosbuild_add_executable(testMeasModelCartesian_radius src/testMeasModelCartesian_radius.cpp src/RBBMCartesian_radius.cpp src/RBBM1Beam.cpp src/phit.cpp src/poccl.cpp)
ADD_DEFINITIONS(-DOCL_DLL_EXPORT)

