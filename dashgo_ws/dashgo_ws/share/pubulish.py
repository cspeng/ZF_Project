 // rosserial_hello_world.cpp : Example of sending command velocities from Windows using rosserial  
 //  
 #include "stdafx.h"
 #include <string>
 #include <stdio.h>
 #include "ros.h"
 #include <geometry_msgs/Twist.h>
 #include <windows.h>

 using std::string;

 int _tmain(int argc, _TCHAR * argv[])
 {
   ros::NodeHandle nh;
   char *ros_master = "192.168.112.125";

   printf("Connecting to server at %s\n", ros_master);
   nh.initNode(ros_master);

   printf("Advertising cmd_vel message\n");
   geometry_msgs::Twist twist_msg;
   ros::Publisher cmd_vel_pub("cmd_vel", &twist_msg);
   nh.advertise(cmd_vel_pub);

   printf("Go robot go!\n");
   while (1)
   {
     twist_msg.linear.x = 5.1;
     twist_msg.linear.y = 0;
     twist_msg.linear.z = 0;
     twist_msg.angular.x = 0;
     twist_msg.angular.y = 0;
    twist_msg.angular.z = -1.8;
     cmd_vel_pub.publish(&twist_msg);

     nh.spinOnce();
     Sleep(100);
   }

   printf("All done!\n");
   return 0;
 }