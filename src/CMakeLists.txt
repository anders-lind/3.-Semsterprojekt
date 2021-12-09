cmake_minimum_required(VERSION 2.8)
project( pylon_opencv )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OpenCV
find_package( OpenCV REQUIRED )

# Pylon 
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
include_directories(${Pylon_INCLUDE_DIRS})

#RL libaries til gripper controller:
find_package(RL COMPONENTS HAL REQUIRED)

#UR-rtde
find_package(ur_rtde REQUIRED)

#Database
find_package(Qt5Sql REQUIRED)

add_executable( main main.cpp colourdetection.cpp colourdetection.h objectdetection.cpp objectdetection.h camera.cpp camera.h pylon.cpp pylon.h gripper.cpp gripper.h robot.cpp robot.h jacobian.cpp jacobian.h kinematics.cpp kinematics.h throw.cpp throw.h database.cpp database.h)
target_link_libraries( main ${OpenCV_LIBS} ${Pylon_LIBRARIES} ${RL_LIBRARIES})
target_link_libraries( main PRIVATE ur_rtde::rtde)
target_link_libraries( main Qt5::Sql)
