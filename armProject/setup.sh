#!/bin/bash
cp -R ~/RobotSystems/armProject/parrot ~/armpi_fpv/src/
cp ~/RobotSystems/armProject/start_functions.launch ~/armpi_fpv/src/armpi_fpv_bringup/launch/
cd ~/armpi_fpv && catkin_make
source ~/armpi_fpv/devel/setup.bash
roslaunch /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/launch/start_functions.launch