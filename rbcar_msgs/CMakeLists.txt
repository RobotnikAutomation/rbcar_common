cmake_minimum_required(VERSION 3.0.2)
project(rbcar_msgs)


find_package(catkin REQUIRED COMPONENTS actionlib_msgs std_msgs geometry_msgs robotnik_msgs message_generation)

add_message_files(DIRECTORY msg
    FILES
    Status.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#    DIRECTORY srv
#    FILES
# )

#add_action_files(
#    FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
   actionlib_msgs
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   robotnik_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
  std_msgs
  geometry_msgs
  robotnik_msgs
)
