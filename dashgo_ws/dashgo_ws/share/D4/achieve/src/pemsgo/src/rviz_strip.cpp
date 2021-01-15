#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "pemsgo/Marker.h"

using namespace geometry_msgs;
using namespace std;
ros::Publisher _strip;
v_msgs::Marker marker_AB[5];


geometry_msgs::Point p2P(Point p)
{
    geometry_msgs::Point msg_P;
    msg_P.y = (320.0 - p.x)/320;
    msg_P.x = (480.0 - p.y)/80;
    
    return msg_P;
    
}

visualization_msgs::Marker v2V(v_msgs::Marker vm)
{
	visualization_msgs::Marker marker;
	marker.id = vm.id;
	marker.header = vm.header;
	marker.header.frame_id = "/odom_combined";
	marker.ns = "rviz_strip";
	marker.action = vm.action;
	marker.type = vm.type;
	marker.scale = vm.scale;
	marker.pose = vm.pose;
	marker.color = vm.color;
	marker.points = vm.points;

	return marker;

}

void markerAB_Update(uint ix, Point va, Point vb, bool is_onstamp)
{
    // %Tag(ID)%
    marker_AB[ix].id= ix;
    marker_AB[ix].header.frame_id = "/base_footprint";
    marker_AB[ix].header.stamp = ros::Time::now();
    marker_AB[ix].ns = "base_strip";
    marker_AB[ix].action = v_msgs::Marker::ADD;
    marker_AB[ix].type = v_msgs::Marker::LINE_STRIP;
    marker_AB[ix].scale.x = 0.03;
    marker_AB[ix].pose.orientation.w = 1.0;
    marker_AB[ix].color.r = 0.9;
    marker_AB[ix].color.g = 0.9;
    marker_AB[ix].color.b = 0.8;
    marker_AB[ix].color.a = 1.0;

    // setup coordinates of marker_AB
    geometry_msgs::Point p1 = p2P(va);
    geometry_msgs::Point p2 = p2P(vb);
    marker_AB[ix].points.clear();  
    if (is_onstamp) 
    {  
		marker_AB[ix].points.push_back(p1);
		marker_AB[ix].points.push_back(p2);
	}
    
}

/*
void findMarker()
{
    
    bool is_onstamp = not_outstamp(maskhist[2]);
    Point va = maskhist[2].vec.a;
    Point vb = maskhist[2].vec.b;
    markerAB_Update(1, va, vb, is_onstamp);

    Point v1 = cept_at(300.0, va, vb);
    Point v2 = cept_at(330.0, va, vb);
    markerAB_Update(2, v1, v2, is_onstamp);

    is_onstamp = not_outstamp(maskhist[1]);
    va = maskhist[1].vec.a;
    vb = maskhist[1].vec.b;

    v1 = cept_at(350.0, va, vb);
    v2 = cept_at(380.0, va, vb);
    markerAB_Update(3, v1, v2, is_onstamp);

    v1 = cept_at(400.0, va, vb);
    v2 = cept_at(430.0, va, vb);
    markerAB_Update(4, v1, v2, is_onstamp);

}
*/

void mask_Fill()
{
        _strip.publish(marker_AB[1]);
        _strip.publish(marker_AB[2]);
        _strip.publish(marker_AB[3]);
        _strip.publish(marker_AB[4]);

    //if (not_outstamp(maskhist[1]) and not_outstamp(maskhist[2]))
    {
        // freeze valid line for two frames in worse condition
        //draw_Line(dst);
        v2V(marker_AB[1]);
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_strip");
    ros::NodeHandle _nh;
    _strip = _nh.advertise<v_msgs::Marker>("strip_test", 2);

    ros::NodeHandle private_node("~");
    //private_node.param<string>("fpath", fpath, _fpath);
    //private_node.param<bool>("is_video", is_video, false);    

    ros::Rate r(10); // 10 hz

    try
    {
      while(ros::ok())
      {
        
        ros::spinOnce();
        r.sleep();
      }
    }
    catch (...)
    {
        ROS_ERROR_STREAM("rviz_strip failed.");
        return 1;
    }

    return 0;
    
}

