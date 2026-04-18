cmake_minimum_required(VERSION 3.0.2)
project(echo_q_peripheral_interfacing)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
catkin_install_python(PROGRAMS scripts/app.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY templates/ static/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
