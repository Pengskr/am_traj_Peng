cmake_minimum_required(VERSION 3.0.2)
project(example3)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  visualization_msgs
  mav_trajectory_generation
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../include
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs roslib
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME}_node src/example3_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)