cmake_minimum_required(VERSION 2.8.3)
project(bobble_controllers)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_interface
  control_msgs
  message_generation
  pluginlib
  rospy
  sensor_msgs
  std_msgs
)
find_package(cmake_modules REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  BobbleBotStatus.msg
  ControlCommands.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  controller_interface
  controller_manager
  control_msgs
  message_runtime
  pluginlib
  rospy
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BobbleBalanceController.cpp
  src/Filter.cpp
  src/MadgwickAHRS.cpp
  src/PidControl.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
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
   src/nodes/BobbleStateHold
   src/nodes/JoystickControl
   src/nodes/KeyboardControl
   DESTINATION
   ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${PROJECT_NAME}_plugins.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS controller_manager rostest)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_rostest_gtest(balance_controller_test
          test/balance_controller.test
          test/balance_controller_test.cpp)
  target_link_libraries(balance_controller_test ${catkin_LIBRARIES})
  add_rostest_gtest(no_balance_test
          test/no_balance.test
          test/no_balance_test.cpp)
  target_link_libraries(no_balance_test ${catkin_LIBRARIES})
endif()
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bobble_controllers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
