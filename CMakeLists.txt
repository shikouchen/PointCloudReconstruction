cmake_minimum_required( VERSION 2.8 )
project( test_pcl )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include_directories(include/)
include_directories(src/)

add_library(${PROJECT_NAME} SHARED
        src/Plane.cpp
        src/SimpleView.cpp
        src/Reconstruction.cpp
        src/SimpleView.cpp)

# eigen
include_directories( "/usr/include/eigen3/" )


# pcl
find_package( PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES})


add_executable(extractWall extract_walls.cpp)
target_link_libraries (extractWall ${PROJECT_NAME})
