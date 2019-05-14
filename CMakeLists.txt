cmake_minimum_required(VERSION 2.8.3)
project(robotics_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation message_filters tf)##message_generation is a necessary dependence to have if you want custom messages

add_message_files(
 FILES
 floatStamped.msg

 )
 
generate_messages(
   DEPENDENCIES
  std_msgs
 )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime message_filters)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(project_robotics src/ackerman.cpp)##comment from this line on and compile, then uncomment and recompile again <-this solve dependencies problems
target_link_libraries(project_robotics ${catkin_LIBRARIES})