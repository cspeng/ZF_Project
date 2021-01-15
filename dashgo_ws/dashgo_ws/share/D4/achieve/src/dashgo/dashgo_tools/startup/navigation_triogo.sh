#!/bin/bash

source /home/eaibot/dashgo_ws/devel/setup.bash

ProgName="navigation_triogo.launch"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   echo "${ProgName} has been launched!"
else
   roslaunch dashgo_nav navigation_triogo.launch
fi

