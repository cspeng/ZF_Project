#!/bin/bash

source /home/eaibot/dashgo_ws/devel/setup.bash

ProgName="bplus_rplidar.launch"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   echo "${ProgName} has been launched!"
else
   roslaunch ydlidar view_ydlidar.launch
fi

