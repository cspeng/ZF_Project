#!/bin/bash
source /home/eaibot/dashgo_ws/devel/setup.bash

ProgName="triogo_track.py"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   echo "${ProgName} has been launched!"
else
   rosrun dashgo_tools triogo_track.py
fi
