#!/bin/sh
# launcher.sh
_ld=$(date +"%Y_%m-%d")
_lt=$(date +"%T")
cd /home/eaibot/III_inone
_file="/home/eaibot/III_inone/log/log_$_ld.txt"
echo "Console connects to TMICS and starts III_inone."
echo >> "$_file"
echo >> "$_file"
echo "III_inone start at $_lt" >> "$_file"
sudo python /home/eaibot/III_inone/bspview.py 2>&1 | tee -a "$_file"
_lt=$(date +"%T")
echo "III_inone end at $_lt" >> "$_file"
echo "Console stops."
cd ~
