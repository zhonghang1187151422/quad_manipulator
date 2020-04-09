cmake_minimum_required(VERSION 2.8.3)
project(quad_manipulator_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  std_msgs
  mavros_msgs
  nav_msgs
  cmake_modules
  cv_bridge
  image_transport
  OpenCV
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(manipulator_velocity_node src/manipulator_velocity_node.cpp)
target_link_libraries(manipulator_velocity_node ${catkin_LIBRARIES})

add_executable(quad_velocity_control src/quad_velocity_control.cpp)
target_link_libraries(quad_velocity_control ${catkin_LIBRARIES})

add_executable(quad_manipulator_startup src/quad_manipulator_startup.cpp)
target_link_libraries(quad_manipulator_startup ${catkin_LIBRARIES})

add_executable(listener_arm_pose src/listener_arm_pose.cpp)
target_link_libraries(listener_arm_pose ${catkin_LIBRARIES})

add_executable(broadcaster_base_world src/broadcaster_base_world.cpp)
target_link_libraries(broadcaster_base_world ${catkin_LIBRARIES})

add_executable(image_circlemarkers_detection src/image_circlemarkers_detection.cpp)
target_link_libraries(image_circlemarkers_detection
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

