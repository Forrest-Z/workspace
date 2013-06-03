cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_library(${PROJECT_NAME} src/EstimationGaussian.cpp src/EstimationMCPdf.cpp)
rosbuild_add_library(${PROJECT_NAME} src/EstimationGaussian.cpp)
rosbuild_add_library(twoLevelMTTD_MCPdf src/EstimationMCPdf.cpp)
rosbuild_add_library(twoLevelMTTD_RBBM src/EstimationRBBM.cpp)
rosbuild_add_library(twoLevelMTTD_RBBM_radius src/EstimationRBBM_radius.cpp)
rosbuild_add_library(VBClusters src/VBClusters.cpp )
rosbuild_add_library(NumberObjectsEstimator src/NumberObjectsEstimator.cpp )
rosbuild_add_library(prepareSeparatedMeasurement
src/PrepareSeparatedMeasurement.cpp src/PrepareSeparatedMeasurementGit.cpp
src/PrepareSeparatedMeasurementFile.cpp
src/PrepareSeparatedMeasurementSimulator.cpp
src/PrepareSeparatedMeasurementLaserScanner.cpp)
ADD_DEFINITIONS(-DOCL_DLL_EXPORT)

