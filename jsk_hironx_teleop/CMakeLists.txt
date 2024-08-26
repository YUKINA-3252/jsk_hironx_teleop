cmake_minimum_required(VERSION 3.0.2)
project(jsk_hironx_teleop)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FloatVector.msg
  )

add_service_files(
  FILES
  ControlBilateral.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

# catkin_package(
#   CATKIN_DEPENDS
#   message_runtime
#   roscpp
#   rospy
#   std_msgs
# )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python2
  CHECK_VENV FALSE
  )

file(GLOB PYTHON_FILES python/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES
  requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
