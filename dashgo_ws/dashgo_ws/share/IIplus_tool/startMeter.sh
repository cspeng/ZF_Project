#!/bin/sh
# launcher.sh
_ld=$(date +"%Y_%m-%d")
_lt=$(date +"%T")
cd /home/pi/tools
_file="/home/pi/tools/log/log_$_ld.txt"
echo >> "$_file"
echo "Tool start at $_lt" >> "$_file"
sudo python /home/pi/tools/bspview.py 2>&1 | tee -a "$_file"
_lt=$(date +"%T")
echo "Tool end at $_lt" >> "$_file"
cd ~
