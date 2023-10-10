cmake_minimum_required(VERSION 2.8.3)
project(localization_msgs)

find_package(catkin REQUIRED 
	COMPONENTS
	roscpp
	rospy
	message_generation
	std_msgs
        sensor_msgs
	geometry_msgs
)


add_message_files(
  FILES
  gnss.msg
  
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

install(FILES LICENSE.txt
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
