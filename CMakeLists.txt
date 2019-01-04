cmake_minimum_required(VERSION 2.8.3)
project(frc_teb_local_planner)

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

        /usr/include/eigen3/
        /usr/local/include/networktables/
        /usr/include/pcl-1.7/

        # Iffy includes
        /opt/ros/kinetic/include/
        )

include_directories(
        include
        ${EXTERNAL_INCLUDE_DIRS}
)

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
        src/timed_elastic_band.cpp
        src/optimal_planner.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/recovery_behaviors.cpp
        src/teb_config.cpp
        src/practical_socket.cpp

        src/main.cpp
        src/main.h
        )

add_executable(frc_teb_local_planner ${SOURCE_FILES})



