cmake_minimum_required(VERSION 2.8.3)
project(robotics_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs message_generation message_filters tf dynamic_reconfigure)##message_generation is a necessary dependence to have if you want custom messages

add_message_files(
 FILES
  customOdom.msg
  floatStamped.msg
 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
  nav_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/Parameters.cfg 
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime message_filters)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(odometry_node src/odometry_node.cpp)##comment from this line on and compile, then uncomment and recompile again <-this solve dependencies problems
add_dependencies(odometry_node ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry_node ${catkin_LIBRARIES})