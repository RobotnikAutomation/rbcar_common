cmake_minimum_required(VERSION 2.8.3)
project(rbcar_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	message_runtime 
	std_msgs 
	move_base
	gmapping
	amcl
	roscpp
	)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(laser_bounds_filter src/laser_bounds_filter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(laser_bounds_filter
   ${catkin_LIBRARIES}
)

