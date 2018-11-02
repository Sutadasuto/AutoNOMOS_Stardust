#ifndef USING_MARKERS
#define USING_MARKERS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Int8.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"

using namespace std;

class UsingMarkers
{

//private:
public:

	ros::NodeHandle n_;
	ros::Publisher marker_pub_;
	ros::Subscriber key_;
	ros::Subscriber points_;
	ros::Subscriber state_;

	visualization_msgs::Marker unit_box_0_;    
	visualization_msgs::Marker unit_cylinder_0_;
	visualization_msgs::Marker unit_cylinder_1_;    
	visualization_msgs::Marker unit_box_1_;
	visualization_msgs::Marker unit_box_2_;
	visualization_msgs::Marker unit_box_3_;

	visualization_msgs::Marker marker_points_, line_strip_;
	geometry_msgs::Point p_;

	gazebo_msgs::ModelState model_state_;
	
public:
	UsingMarkers();
	~UsingMarkers();
	
	void NewPoint(const geometry_msgs::Point &point);
	void MoveCylinder(const std_msgs::Int8 &key);
	void GetState(const gazebo_msgs::ModelStates &msg);

private:
};

#endif
