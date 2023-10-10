sudo chmod 777 /dev/ttyUSB0
sleep 1
source devel/setup.bash
roslaunch src/huace_driver/huace_driver.launch
