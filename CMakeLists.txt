# cmake_minimum_required( VERSION 2.8 )

# project(EXTRACT_WALLS)

# include_directories(
# ${PROJECT_SOURCE_DIR}
# ${PROJECT_SOURCE_DIR}/include)

# add_executable( ${PROJECT_NAME} extract_walls.cpp 
# )
# # add_library(${PROJECT_NAME} SHARED
# # src/model.cpp
# # )

# # Set StartUp Project (Option)
# # (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
# set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME} )

# # Find Packages
# find_package( PCL 1.8 REQUIRED )

# if( PCL_FOUND )
#   # [C/C++]>[General]>[Additional Include Directories]
#   include_directories( ${PCL_INCLUDE_DIRS} )

#   # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
#   add_definitions( ${PCL_DEFINITIONS} )
  
#   # For Use Not PreCompiled Features 
#   add_definitions( -DPCL_NO_PRECOMPILE )

#   # [Linker]>[General]>[Additional Library Directories]
#   link_directories( ${PCL_LIBRARY_DIRS} )

#   # [Linker]>[Input]>[Additional Dependencies]
#   target_link_libraries( ${PROJECT_NAME}  
#     ${PCL_LIBRARIES})
# endif()


# # target_link_libraries(project ${PROJECT_NAME})

cmake_minimum_required( VERSION 2.8 )

# Create Project
project( extract_walls )
add_executable( project extract_walls.cpp )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include)

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( project ${PCL_LIBRARIES} )
endif()