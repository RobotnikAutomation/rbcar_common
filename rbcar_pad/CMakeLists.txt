cmake_minimum_required(VERSION 2.8.3)
project(rbcar_pad)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
  std_srvs
  std_msgs
  ackermann_msgs
  message_runtime
)


catkin_package(
	CATKIN_DEPENDS robotnik_msgs ackermann_msgs
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rbcar_joystick_pad src/rbcar_joystick.cpp)
target_link_libraries(rbcar_joystick_pad ${catkin_LIBRARIES})
add_dependencies(rbcar_joystick_pad rbcar_joystick_generate_messages_cpp)



