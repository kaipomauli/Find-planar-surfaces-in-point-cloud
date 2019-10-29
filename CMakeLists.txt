cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(findPlanes)

find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src/inc "C:\\Program Files\\openCv\\build\\include" )
link_directories(${PCL_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/src/lib  "C:\\Program Files\\openCv\\build\\x64\\vc14\\lib")

add_definitions(${PCL_DEFINITIONS})

add_executable (findPlanes main.cpp vizualize.cpp write_pcd.cpp stdafx.cpp ThreadSafePrint.cpp CubicVOI.cpp colorSubset.cpp segObject.cpp MyColor.cpp findPlane.cpp)
target_link_libraries (findPlanes ${PCL_LIBRARIES} opencv_world331d.lib opencv_world331.lib )
add_custom_command(TARGET findPlanes POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different  "C:\\Program Files\\opencv\\build\\x64\\vc14\\bin\\$<CONFIGURATION>" $<TARGET_FILE_DIR:findPlanes>)