cmake_minimum_required(VERSION 2.8.3)
project(detect_and_collect)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


#Declare CatkinPackage
catkin_package()


install(DIRECTORY parse_and_publish
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(parse_and_publish parse_and_publish/parse_and_publish.cpp)
target_link_libraries(parse_and_publish ${catkin_LIBRARIES})
add_dependencies(parse_and_publish std_msgs)