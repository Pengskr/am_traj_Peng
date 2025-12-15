cmake_minimum_required(VERSION 3.0.2)
project(example2)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  mav_trajectory_generation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example2
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../include
)

add_executable(${PROJECT_NAME}_node src/example2_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)