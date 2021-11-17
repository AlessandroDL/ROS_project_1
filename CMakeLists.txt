cmake_minimum_required(VERSION 3.0.2)
project(project)

find_package(
  catkin REQUIRED
  COMPONENTS
    message_generation
    genmsg
    nav_msgs
    std_msgs
    roscpp
    message_filters
    dynamic_reconfigure
    tf
    robotics_hw1
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

add_message_files(
  FILES
    ##MotorSpeed.msg
    CustomOdometry.msg
)

add_service_files(
  FILES
    SetPose.srv
    ResetPose.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    robotics_hw1
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
    message_filters
    robotics_hw1
)

## Build estimation
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(estimator src/estimator.cpp)
target_link_libraries(estimator ${catkin_LIBRARIES})
add_dependencies(estimator robotics_hw1_generate_messages_cpp)

## Build Transform
add_executable(Transform src/Transform.cpp)
target_link_libraries(Transform ${catkin_LIBRARIES})


## Build DifferentialDrive
add_executable(DifferentialDrive src/DifferentialDrive.cpp)
target_link_libraries(DifferentialDrive ${catkin_LIBRARIES})

add_dependencies(DifferentialDrive robotics_hw1_generate_messages_cpp)
add_dependencies(DifferentialDrive ${PROJECT_NAME}_gencfg)
add_dependencies(DifferentialDrive ${catkin_EXPORTED_TARGETS})
add_dependencies(DifferentialDrive ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
