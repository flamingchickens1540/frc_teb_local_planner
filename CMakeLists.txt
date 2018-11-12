cmake_minimum_required(VERSION 2.8.3)
project(jni_teb_local_planner)

#set(CMAKE_SYSTEM_NAME Linux)
#set(CMAKE_SYSTEM_PROCESSOR arm)
#-D
#CMAKE_C_COMPILER=/usr/bin/arm-frc-linux-gnueabi-gcc
#-D
#CMAKE_CXX_COMPILER=/usr/bin/arm-frc-linux-gnueabi-g++

#set(tools /home/devel/gcc-4.7-linaro-rpi-gnueabihf)
#set(CMAKE_C_COMPILER /usr/bin/arm-frc-linux-gnueabi-gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/arm-frc-linux-gnueabi-g++)
#
#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

find_package(JNI REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS
        ${Eigen_INCLUDE_DIRS}
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${JNI_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include/

        # Iffy includes
        /opt/ros/kinetic/include/
        /usr/include/eigen3/
        /usr/include/pcl-1.7/
#        /usr/include/suitesparse/
#        ~/Documents/ADT/
#        /home/wangl/Documents/network_tables/include
        /usr/local/include/
        )

include_directories( ${EXTERNAL_INCLUDE_DIRS} )

set(NT_LIBRARIES
        /usr/local/lib/libntcore.so
        /usr/local/lib/libwpiutil.so
        )
set(EXTERNAL_LIBS
        ${Boost_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        ${NT_LIBRARIES}
        )

message(STATUS "System: ${G2O_LIBRARIES}")

link_libraries(${EXTERNAL_LIBS})

IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()

set(SOURCE_FILES
#        teb_jni_interface.cpp
        src/timed_elastic_band.cpp
        src/optimal_planner.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/recovery_behaviors.cpp
        src/teb_config.cpp
        src/PracticalSocket.cpp

#        src/homotopy_class_planner.cpp
#        src/teb_local_planner_ros.cpp
#        src/graph_search.cpp
        src/main.cpp
        src/main.h
        )

#add_library(jni_teb_local_planner SHARED ${SOURCE_FILES})
add_executable(jni_teb_local_planner ${SOURCE_FILES})



