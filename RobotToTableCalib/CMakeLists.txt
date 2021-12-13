cmake_minimum_required(VERSION 2.8)
project( Robot2Table )
#OpenCV
find_package( OpenCV REQUIRED )
#UR-rtde
find_package(ur_rtde REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Robot2Table main.cpp objectcoordinates.cpp objectcoordinates.h cameracali.cpp cameracali.h)
target_link_libraries( Robot2Table ${OpenCV_LIBS} )
target_link_libraries( Robot2Table PRIVATE ur_rtde::rtde)
