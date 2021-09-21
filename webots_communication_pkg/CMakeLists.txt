cmake_minimum_required(VERSION 3.0.2)
project(webots_communication_pkg)
set(CMAKE_CXX_STANDARD 11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(base_node src/base_node.cpp include/webots_communication_pkg/base_node.hpp include/webots_communication_pkg/base_node_configure.h)
target_link_libraries(base_node ${catkin_LIBRARIES})
add_dependencies(base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_node src/camera_node.cpp include/webots_communication_pkg/camera_node.hpp include/webots_communication_pkg/camera_node_configure.h)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(imu_node src/imu_node.cpp include/webots_communication_pkg/imu_node.hpp include/webots_communication_pkg/imu_node_configure.h)
target_link_libraries(imu_node ${catkin_LIBRARIES})
add_dependencies(imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_node src/lidar_node.cpp src/lidar_node.cpp include/webots_communication_pkg/lidar_node.hpp)
target_link_libraries(lidar_node ${catkin_LIBRARIES})
add_dependencies(lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})