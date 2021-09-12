cmake_minimum_required(VERSION 3.0.2)
project(labiagi_proj)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  message_generation
  laser_geometry
)

##Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LabiagiProj
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(collision_avoidance src/collision_avoidance.cpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})
add_dependencies(collision_avoidance labiagi_proj_generate_messages_cpp)