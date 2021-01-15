#!/bin/sh
# launcher.sh
_ld=$(date +"%Y_%m-%d")
_lt=$(date +"%T")
cd /home/pi/tools
_file="/home/pi/tools/log/log_$_ld.txt"
echo >> "$_file"
echo "###" >> "$_file"
echo "Tool reboot at $_lt" >> "$_file"
sudo chmod 666 "$_file"
sudo python startTorque-bsp.py
cd ~
