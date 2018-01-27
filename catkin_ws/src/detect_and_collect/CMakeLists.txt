cmake_minimum_required(VERSION 2.8.3)
project(detect_and_collect)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  darknet_ros_msgs
)

##add_message_files(
##  FILES
##    BoundingBox.msg
##    BoundingBoxes.msg
##)

#Declare CatkinPackage
catkin_package(CATKIN_DEPENDS 
  message_runtime)


install(DIRECTORY parse_and_publish
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(parse_and_publish parse_and_publish/parse_and_publish.cpp)
target_link_libraries(parse_and_publish ${catkin_LIBRARIES})
add_dependencies(parse_and_publish std_msgs)