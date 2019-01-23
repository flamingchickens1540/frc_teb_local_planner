# Include toolchain config
include(cmake_toolchains/wpi-pi.cmake)

cmake_minimum_required(VERSION 2.8.3)
project(frc_teb_local_planner)

add_definitions(-Wall) # Verbose compile

set(CMAKE_VERBOSE_MAKEFILE ON)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -pthread")

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

set(BOOST_LIBRARYDIR ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf)

find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

set(EXTERNAL_INCLUDE_DIRS
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}

        ${CMAKE_FIND_ROOT_PATH}/usr/include/eigen3/
        ${CMAKE_FIND_ROOT_PATH}/usr/include/pcl-1.7/
        ${CMAKE_FIND_ROOT_PATH}/usr/include/arm-linux-gnueabihf

        # Iffy includes
        /opt/ros/kinetic/include/
        )

include_directories(
        include
        src
        lib
        ${EXTERNAL_INCLUDE_DIRS}
)

link_directories(lib
        ${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf
        ${CMAKE_FIND_ROOT_PATH}/usr/lib/lapack
        ${CMAKE_FIND_ROOT_PATH}/usr/lib/libblas
        )


set(NT_LIBRARIES
        boost_regex.so.1.62.0
        icudata.so.57
        icui18n.so.57
        icuuc.so.57
        suitesparseconfig.so.4
        metis.so.5
        lapack.so.3
        lapack.so
        blas.so.3
        gfortran.so.3

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

link_libraries(${EXTERNAL_LIBS})

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
        src/network_config.h)

add_executable(frc_teb_local_planner ${SOURCE_FILES})



