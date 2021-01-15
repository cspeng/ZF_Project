#!/bin/bash

source /home/eaibot/dashgo_ws/devel/setup.bash

ProgName="arrcam.launch"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)
_file="/home/eaibot/log/log_2018.txt"

if [ $num -gt 0 ] 
then 
   echo "${ProgName} has been launched!" 
else
   roslaunch camplus arrcam.launch 2>&1 | tee -a "$_file"
fi
