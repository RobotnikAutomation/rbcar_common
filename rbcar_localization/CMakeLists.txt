cmake_minimum_required(VERSION 2.8.3)
project(rbcar_localization)


find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch maps scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)