cmake_minimum_required(VERSION 2.8.3)
project(rbcar_navigation)


find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch config kinematic_model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)