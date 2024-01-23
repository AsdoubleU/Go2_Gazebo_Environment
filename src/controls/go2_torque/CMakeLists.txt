cmake_minimum_required(VERSION 3.0.2)
project(go2_torque)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  urdf
  tf
  xacro
  tf_conversions
  gazebo_ros
  rviz
  controller_manager
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
 DEPENDS gazebo_ros roscpp rospy std_msgs Boost pinocchio
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${PCL_LIBRARY_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  )

link_directories(
    ${pinocchio_LIBRARY_DIRS}
)

include_directories(
  include
  include/Tools
  include/OsqpEigen
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(include/Tools)
add_subdirectory(include/OsqpEigen)

add_library(${PROJECT_NAME}
  include/controller.h
  src/controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${pinocchio_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME}_node src/controller_node.cpp)
# add_executable(plate_node src/plate_controller.cpp)
# add_executable(wrench_signal_generate src/WrenchSignalGenerate.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(plate_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} Tools)
target_link_libraries(${PROJECT_NAME} osqpeigen)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# target_link_libraries(plate_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
