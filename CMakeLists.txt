cmake_minimum_required(VERSION 2.8.3)
project(bobble_controllers)
add_compile_options(-std=c++11)

find_package(
  catkin
  REQUIRED COMPONENTS
  control_msgs
  controller_interface
  controller_manager
  message_generation
  pluginlib
  rospy
  rostest
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

add_message_files(
  FILES
  BobbleBotStatus.msg
  ControlCommands.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  control_msgs
  controller_interface
  controller_manager
  message_runtime
  pluginlib
  rospy
  rostest
  sensor_msgs
  std_msgs
  tf2_ros
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BalanceBaseController.cpp
  src/BalanceSimController.cpp
  src/Filter.cpp
  src/HighPassFilter.cpp
  src/LowPassFilter.cpp
  src/MadgwickAHRS.cpp
  src/PidControl.cpp
  src/PIDFilters.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(filter_test
            test/filter_test.cpp)
    target_link_libraries(filter_test ${PROJECT_NAME})
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(
   PROGRAMS
   src/bobble_controllers_nodes/BobbleStateHold
   src/bobble_controllers_nodes/JoystickControl
   src/bobble_controllers_nodes/KeyboardControl
   src/bobble_controllers_nodes/ApplyImpulse
   DESTINATION
   ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${PROJECT_NAME}_plugins.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
