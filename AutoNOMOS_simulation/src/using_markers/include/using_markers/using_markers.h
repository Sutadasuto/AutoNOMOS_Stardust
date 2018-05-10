#ifndef USING_MARKERS
#define USING_MARKERS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"

using namespace std;

class using_markers
{

//private:
public:

	ros::NodeHandle n;
	ros::Publisher marker_pub;
	ros::Subscriber key_;
	ros::Subscriber points_;
	ros::Subscriber state_;

	visualization_msgs::Marker unit_box_0;    
	visualization_msgs::Marker unit_cylinder_0;
	visualization_msgs::Marker unit_cylinder_1;    
	visualization_msgs::Marker unit_box_1;
	visualization_msgs::Marker unit_box_2;
	visualization_msgs::Marker unit_box_3;

	visualization_msgs::Marker points, line_strip;
	geometry_msgs::Point p;

	gazebo_msgs::ModelState state;
	
public:
	using_markers();
	~using_markers();
	
	void newPoint(const geometry_msgs::Point &point);
	void moveCylinder(const std_msgs::Int8 &key);
	void getState(const gazebo_msgs::ModelStates &msg);

private:
};

#endif
