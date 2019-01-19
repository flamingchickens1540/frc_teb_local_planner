## Define our host system
#SET(CMAKE_SYSTEM_NAME Linux)
#SET(CMAKE_SYSTEM_VERSION 1)
#
## Define the cross compiler locations
#SET(CMAKE_C_COMPILER   /home/wangl/raspberrypi/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
#SET(CMAKE_CXX_COMPILER /home/wangl/raspberrypi/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)
#
## Define the sysroot path for the RaspberryPi distribution in our tools folder
#SET(CMAKE_FIND_ROOT_PATH /home/wangl/RaspberryPi/raspberrypi-tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/arm-linux-gnueabihf/sysroot/)
##
### Use our definitions for compiler tools
#SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)..
### Search for libraries and headers in the target directories only
#SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#
add_definitions(-Wall -std=c11)

cmake_minimum_required(VERSION 2.8.3)
project(frc_teb_local_planner)


set(CMAKE_VERBOSE_MAKEFILE ON)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -pthread")

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

set(EXTERNAL_INCLUDE_DIRS
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
#        /usr/include/boost/
#        /usr/include/g2o/
#        /usr/include/suitesparse/

        ${CMAKE_FIND_ROOT_PATH}/usr/include/eigen3/
#        /usr/local/include/ntcore/
        ${CMAKE_FIND_ROOT_PATH}/usr/include/pcl-1.7/
        /home/wangl/RaspberryPi/rootfs/usr/include/arm-linux-gnueabihf/

        # Iffy includes
        /opt/ros/kinetic/include/
        )

include_directories(
        include
        src
        lib
        ${EXTERNAL_INCLUDE_DIRS}
)

link_directories(lib)

set(NT_LIBRARIES
#        /usr/local/lib/libntcore.so
#        /usr/local/lib/libwpiutil.so
        wpiHal.so
        wpilibc.so
        wpiHal.so
        cameraserver.so
        cscore.so
        ntcore.so
        wpiutil.so
        opencv_ml.so
        opencv_objdetect.so
        opencv_shape.so
        opencv_stitching.so
        opencv_superres.so
        opencv_videostab.so
        opencv_calib3d.so
        opencv_features2d.so
        opencv_highgui.so
        opencv_videoio.so
        opencv_imgcodecs.so
        opencv_video.so
        opencv_photo.so
        opencv_imgproc.so
        opencv_flann.so
        opencv_core.so
        )

set(EXTERNAL_LIBS
        ${Boost_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        ${NT_LIBRARIES}
        )

message(STATUS "System: ${G2O_LIBRARIES}")

link_libraries(${EXTERNAL_LIBS})

#IF(NOT MSVC)
#  include(CheckCXXCompilerFlag)
#  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required
#  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
#endif()
#endif()

set(SOURCE_FILES
        src/timed_elastic_band.cpp
        src/optimal_planner.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/recovery_behaviors.cpp
        src/teb_config.cpp
        src/practical_socket.cpp
        src/practical_socket.h

        src/main.cpp
        src/main.h
        )

add_executable(frc_teb_local_planner ${SOURCE_FILES})



