cmake_minimum_required(VERSION 2.8.3)
project(baxter_tasker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS baxter_core_msgs baxter_interface roscpp rospy std_msgs sensor_msgs tf message_generation hr_helper)
#MESSAGE("CATKIN LIBRARIES")
#MESSAGE ( ${catkin_LIBRARIES} )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html



#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#   DIRECTORY srv 
#   FILES
#   SetStartPose.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   baxter_core_msgs
#   sensor_msgs
#)


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
#  INCLUDE_DIRS include
#  LIBRARIES baxter_pickplace
  CATKIN_DEPENDS baxter_core_msgs baxter_interface roscpp rospy std_msgs sensor_msgs  tf message_runtime message_generation hr_helper
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${moveit_INCLUDE_DIRS}
)

#MESSAGE("CATKIN INCLUDE")
#MESSAGE ( ${catkin_INCLUDE_DIRS} )

## Declare a cpp library
# add_library(baxter_pickplace
#   src/${PROJECT_NAME}/baxter_pickplace.cpp
# )

## Declare a cpp executable
#add_executable(move_it_wrapper src/move_it_wrapper.cpp src/MoveItWrapper.cpp)
#add_executable(move_interface src/moveInterface.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(move_interface
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)
#target_link_libraries(move_it_wrapper
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)
#add_executable(move_random src/moveGroup.cpp)
#target_link_libraries(move_random
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(baxter_pickplace_node baxter_pickplace_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baxter_pickplace baxter_pickplace_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_pickplace.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
