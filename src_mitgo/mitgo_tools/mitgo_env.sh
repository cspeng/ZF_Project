#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyCOM3"' >/etc/udev/rules.d/inone.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8037", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyCOM4"' >/etc/udev/rules.d/cpanel.rules


echo  'KERNEL=="tty*", ATTRS{devpath}=="1.3", MODE:="0666", GROUP:="dialout",  SYMLINK+="port1"' >/etc/udev/rules.d/port1.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4", MODE:="0666", GROUP:="dialout",  SYMLINK+="port2"' >/etc/udev/rules.d/port2.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1", MODE:="0666", GROUP:="dialout",  SYMLINK+="port3"' >/etc/udev/rules.d/port3.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.2", MODE:="0666", GROUP:="dialout",  SYMLINK+="port4"' >/etc/udev/rules.d/port4.rules


echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1.3", MODE:="0666", GROUP:="dialout",  SYMLINK+="portc1"' >/etc/udev/rules.d/portc1.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1.4", MODE:="0666", GROUP:="dialout",  SYMLINK+="portc2"' >/etc/udev/rules.d/portc2.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1.1", MODE:="0666", GROUP:="dialout",  SYMLINK+="portc3"' >/etc/udev/rules.d/portc3.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1.2", MODE:="0666", GROUP:="dialout",  SYMLINK+="portc4"' >/etc/udev/rules.d/portc4.rules


echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4.3", MODE:="0666", GROUP:="dialout",  SYMLINK+="portb1"' >/etc/udev/rules.d/portb1.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4.4", MODE:="0666", GROUP:="dialout",  SYMLINK+="portb2"' >/etc/udev/rules.d/portb2.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4.1", MODE:="0666", GROUP:="dialout",  SYMLINK+="portb3"' >/etc/udev/rules.d/portb3.rules

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4.2", MODE:="0666", GROUP:="dialout",  SYMLINK+="portb4"' >/etc/udev/rules.d/portb4.rules

service udev reload
sleep 2
service udev restart


chmod +x  /home/pi/catkin_ws/src/mitgo/mitgo_tools/startup/*sh
