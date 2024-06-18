cmake_minimum_required(VERSION 3.0.2)
project(collision_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  goal_allocator
)

find_package(
  RVO3D REQUIRED
  #RVO 2.0.3 REQUIRED
)
catkin_package(
  INCLUDE_DIRS include 
  #LIBRARIES my_drone_swarm
  CATKIN_DEPENDS roscpp  std_msgs geometry_msgs nav_msgs 
  DEPENDS RVO3D
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RVO_INCLUDE_DIRS}
  )

add_executable(collision_avoidance_node src/main.cpp src/collision_avoidance_algo.cpp)


add_dependencies(collision_avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(collision_avoidance_node
  ${catkin_LIBRARIES}
  RVO3D
  
)





#