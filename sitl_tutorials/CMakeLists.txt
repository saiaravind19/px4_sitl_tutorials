cmake_minimum_required(VERSION 3.0.2)
project(sitl_tutorials)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  
  std_msgs
  geometry_msgs
  
)

catkin_package(
  LIBRARIES sitl_tutorials
  CATKIN_DEPENDS mavros roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_node src/offboard.cpp)
add_executable(velpub src/velocity.cpp)

target_link_libraries(offboard_node ${catkin_LIBRARIES})
target_link_libraries(velpub ${catkin_LIBRARIES})

