#ifndef SKYCAM
#define SKYCAM

#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>

#include <math.h> 
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <string>

#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include "tf2_msgs/TFMessage.h" //sonar
#include "std_msgs/Float32.h"

#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/Int8.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace cv;
using namespace std;

class skycam
{

//private:
public:

	ros::NodeHandle nh_;
	
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber sky_image_;
	ros::Subscriber laser_;
	ros::Publisher velocity_;
	ros::Publisher steering_;
	ros::Subscriber states_;
	ros::Publisher state_;
	ros::Subscriber key_;
	ros::Publisher path3D_;
	ros::Subscriber goal2D_;
	ros::Publisher rvizImage_;
	ros::Publisher rvizLaser_;
	ros::Publisher rvizRRT_;

	Mat sky_image;
	
	cv_bridge::CvImagePtr cv_ptr_1;	

	//Image segmentation
	Mat lab;
	Mat labChannels[3];
	Mat lChannel, aChannel, bChannel;	
	Mat car;
	Mat map;	
	Mat laserMap;
	float scale;
	float resizeScale;

	//Handle Mouse
	Point goal;
	bool flag1;
	

	// Path planning
	float minCurve;
	Mat path;
	int flag;
	Point position;
	float angle;
	int beam;
	vector<int> indices;
	vector<Point> route;
	vector<Point> newPoints;
	bool closeFlag;

	// Control
	Mat prevMap;	
	int p1, p2;
	float direction;
	tf::Vector3 axis;
	tf::Quaternion q;
	tf::Matrix3x3 R;
	tf::Vector3 positionVector, originVector, goalVector;
	float errorVelocity, errorSteering, errorSteering2;
	float IerrorVelocity, IerrorSteering, IerrorSteering2;
	float prevErrorSteering, prevErrorSteering2;
	float DerrorSteering, DerrorSteering2;
	float prevErrorVelocity;
	float DerrorVelocity;
	float Pv, Iv, Dv, Ps, Is, Ds, Ps2, Is2, Ds2;
	float alpha;
	float speedLimit;
	float velRange, steerRange;
	int errorTolerance;
	int skip;
	std_msgs::Float32 velocity;
	std_msgs::Float32 steering;
	float laserFOV;
	float laserRange;
	bool newObstacleFlag;
	bool readyFlag;
	int obstacleThreshold;
	int deadPathCounter;
	bool beginTrack;

	vector<float> errorSteeringVector;
	vector<int> goalPointsVector;
	std::string filename;
	int testNum;
	Mat routes;

	tf::Matrix3x3 Rt;
	tf::Matrix3x3 Rd;
	tf::Matrix3x3 minusRd;
	tf::Matrix3x3 Re;
	tf::Vector3 y;

	// Para mover el cilindro
	gazebo_msgs::ModelState state;

	// Rviz
	visualization_msgs::Marker unit_box_0;    
	visualization_msgs::Marker unit_cylinder_0;
	visualization_msgs::Marker unit_cylinder_1;    
	visualization_msgs::Marker unit_box_1;
	visualization_msgs::Marker unit_box_2;
	visualization_msgs::Marker unit_box_3;
	sensor_msgs::ImagePtr rvizImage;
	sensor_msgs::ImagePtr rvizLaser;
	sensor_msgs::ImagePtr rvizRRT;
	
public:
	skycam();
	~skycam();
	
	void skyImage(const sensor_msgs::ImageConstPtr &msg);
	void readLaser(const sensor_msgs::LaserScan &msg);
	vector<Point> RRT(Mat analyzedMap, Mat analyzedCar, Point goal, int beam, float scale, float angle);		
	Point getPosition(Mat carMap);
	vector<Point> smoothPath(Mat analyzedMap, vector<Point> path);

	void getStates(const gazebo_msgs::ModelStates &msg);
	void moveCylinder(const std_msgs::Int8 &key);
	void goalFromRviz(const geometry_msgs::PoseStamped &goalRviz);
	tf::Vector3 inverseHatMap(tf::Matrix3x3 re);

private:
	static void onMouseStatic( int event, int x, int y, int flags, void* point );
	void onMouse( int event, int x, int y, int flags );
};

#endif
