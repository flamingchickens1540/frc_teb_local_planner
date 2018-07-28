cmake_minimum_required(VERSION 2.8.3)
project(jni_teb_local_planner)

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

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

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

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIRS} )


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


set(SOURCE_FILES teb_jni_interface.cpp src/timed_elastic_band.cpp
        src/optimal_planner.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/recovery_behaviors.cpp
        src/teb_config.cpp
#        src/homotopy_class_planner.cpp
#        src/teb_local_planner_ros.cpp
#        src/graph_search.cpp
        )
add_library(jni_teb_local_planner SHARED ${SOURCE_FILES})


include_directories(${JNI_INCLUDE_DIRS})
include_directories(/usr/include/eigen3/)
include_directories(/opt/ros/kinetic/include/)
include_directories(include/)
include_directories(/usr/include/pcl-1.7/)
include_directories(/usr/include/suitesparse/)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

TARGET_LINK_LIBRARIES( jni_teb_local_planner LINK_PUBLIC ${Boost_LIBRARIES} )
TARGET_LINK_LIBRARIES( jni_teb_local_planner LINK_PUBLIC ${G2O_LIBRARIES} )
TARGET_LINK_LIBRARIES( jni_teb_local_planner LINK_PUBLIC ${SUITESPARSE_LIBRARIES} )

