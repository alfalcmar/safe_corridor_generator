cmake_minimum_required(VERSION 2.8.3)
project(safe_corridor_generator)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
	pcl_ros
	pcl_conversions
	decomp_ros_utils
  nav_msgs
  geometry_msgs
  )

# include eigen and pcl library
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include utility for free space decomposition
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

# include jps3d library
find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
  LIBRARIES DecomposeWrapper DecomposeNode SafeCorridorGenerator
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


# decompose wrapper
add_library(DecomposeWrapper
  src/decompose_wrapper.cpp
)

add_dependencies(DecomposeWrapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(DecomposeWrapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
  ${DECOMP_UTIL_LIBRARIES}
  )

# JPS manager
add_library(JPSManager
  src/jps_manager.cpp)

#SafeCorridorGenerator
add_library(SafeCorridorGenerator
  src/safe_corridor_generator.cpp
)

add_dependencies(SafeCorridorGenerator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(SafeCorridorGenerator
  DecomposeWrapper
  JPSManager
  ${catkin_LIBRARIES}
  ${JPS3D_LIBRARIES}
  ${DECOMP_UTIL_LIBRARIES}
  ${PCL_LIBRARIES}
  )


# decompose node
add_library(DecomposeNode
  src/decompose_node.cpp
  )

add_dependencies(DecomposeNode
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(DecomposeNode
  SafeCorridorGenerator
  DecomposeWrapper
  JPSManager
  ${catkin_LIBRARIES}
  ${JPS3D_LIBRARIES}
  ${DECOMP_UTIL_LIBRARIES}
  ${PCL_LIBRARIES}
  )
# add_executable(${PROJECT_NAME}_node src/decompose_node.cpp src/decompose_wrapper.cpp src/jps_manager.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )

install(TARGETS SafeCorridorGenerator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
