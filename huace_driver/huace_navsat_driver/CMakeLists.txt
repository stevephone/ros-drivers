cmake_minimum_required(VERSION 2.8.3)
project(huace_navsat_driver)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()
catkin_package()

install(PROGRAMS
   nodes/huace_serial_driver
   nodes/huace_topic_driver
   nodes/huace_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
#roslint_add_test()
