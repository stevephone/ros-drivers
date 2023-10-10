cmake_minimum_required(VERSION 3.0.2)
project(msg_converter)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O2 ")#  -g  O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -std=c++11 -O2 ")#  -g  O3

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nmea_msgs
  #gps_common
)

	# add_message_files(FILES ASENSING.msg)
generate_messages(DEPENDENCIES 
#common_msgs
#perception_msgs
nmea_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
     include
  ${catkin_INCLUDE_DIRS}
)

add_executable(converter_node converter_node.cpp)
target_link_libraries(converter_node ${catkin_LIBRARIES})
# add_dependencies(converter_node nmea_msgs)
	add_dependencies(converter_node ${PROJECT_NAME}_generate_messages_cpp)



install(TARGETS
converter_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

 install(FILES
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )



