cmake_minimum_required(VERSION 2.8)
project(libkinectwrapper)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

message("PROJECT_NAME:  " ${PROJECT_NAME})
message("PROJECT_NAME_INCLUDE_DIRS:  " ${PROJECT_NAME}_INCLUDE_DIRS)
# set(CMAKE_BUILD_TYPE Debug)
# find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

SET(LIB_KINECT_WRAPPER_SRC
    KFRDisplay.cpp
    KinectPacket.cpp
    KinectWrapperLive.cpp
    KinectWrapperPlayback.cpp
    MKConfig.cpp
    MKPPersonTracker.cpp
    MKPComputeGoal.cpp
    MKPPool.cpp
    MKPRAprilTag.cpp
    MKPRCornerDisplay.cpp
    MKPRMergeBodyTracks.cpp
    MKPRRecord.cpp
    MKPRSaveFrame.cpp
    MKPRSplitter.cpp
    AsyncMKPRSplitter.cpp
    MultiKinectPacket.cpp
    MultiKinectWrapper.cpp
    Calibration.cpp
    DepthViewer.cpp
    ThreadPool.cpp
    AsyncMKPRecipient.cpp
)

SET(LIB_KINECT_WRAPPER_INCLUDES
    BodyTrackConstants.h
    KFRDisplay.h
    LinectPacket.cpp
    KinectWrapper.h
    KinectWrapperLive.h
    KinectWrapperPlayback.h
    MKConfig.h
    MKPPersonTracker.h
    MKPComputeGoal.h
    MKPPool.h
    MKPRAprilTag.h
    MKPRCornerDisplay.h
    MKPRMergeBodyTracks.h
    MKPRRecord.h
    MKPRSaveFrame.h
    MKPRSplitter.h
    AsyncMKPRSplitter.h
    MultiKinectPacket.h
    MultiKinectWrapper.h
    Calibration.h
    DepthViewer.h
    profiling.h
    ThreadPool.h
    AsyncMKPRecipient.h
)

include_directories(
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${VISION_GEOMETRY_INCLUDE_DIRS}
    ${APRIL_TAG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(kinectwrapper ${LIB_KINECT_WRAPPER_SRC})

install(TARGETS kinectwrapper DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/kinectwrapper)