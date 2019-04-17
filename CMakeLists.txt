cmake_minimum_required(VERSION 2.8.3)
project(mars_mot_plan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
  actionlib
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PoseTrajectory.action
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_mot_plan
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_FILES 
  src/kinematics/Quat.cpp 
  src/kinematics/DualQuat.cpp 
  src/kinematics/PadenKahan.cpp 
  src/kinematics/MarsUR5.cpp
  src/kinematics/MarsJacobians.cpp
  src/kinematics/Pose.cpp
  src/traj_plan/TrajPlan.cpp
  src/traj_plan/PoseTrajectory.cpp
  src/traj_plan/PoseIterTrajectory.cpp
)

add_executable(mobman_mot_plan_sim_node src/mobman_mot_plan_sim.cpp ${SOURCE_FILES})
target_include_directories(mobman_mot_plan_sim_node PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mobman_mot_plan_sim_node ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(mobman_mot_plan_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mobman_mot_plan_exp_node src/mobman_mot_plan_exp.cpp ${SOURCE_FILES})
target_include_directories(mobman_mot_plan_exp_node PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mobman_mot_plan_exp_node ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(mobman_mot_plan_exp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mars_mot_plan_sim_server_node src/mars_mot_plan_sim_server.cpp ${SOURCE_FILES})
target_include_directories(mars_mot_plan_sim_server_node PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mars_mot_plan_sim_server_node ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(mars_mot_plan_sim_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mars_mot_plan_exp_server_node src/mars_mot_plan_exp_server.cpp ${SOURCE_FILES})
target_include_directories(mars_mot_plan_exp_server_node PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mars_mot_plan_exp_server_node ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(mars_mot_plan_exp_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
install(TARGETS mobman_mot_plan_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mars_mot_plan.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
