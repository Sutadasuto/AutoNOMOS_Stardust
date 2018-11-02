#ifndef SKYCAM
#define SKYCAM

#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include <stdlib.h>     /* srand, rand */
#include <string>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "tf2_msgs/TFMessage.h" //sonar
#include <time.h>       /* time */
#include <visualization_msgs/Marker.h>

using namespace cv;
using namespace std;

class Skycam
{

//private:
public:

	ros::NodeHandle nh_;	
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber sky_image_receiver_;	
	ros::Publisher path3D_;	
	ros::Publisher rviz_image_publisher_;
	ros::Publisher rviz_laser_publisher_;
	ros::Publisher rviz_rrt_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher steering_publisher_;
    ros::Publisher velocity_publisher_;	
	ros::Subscriber key_;
    ros::Subscriber laser_;
    ros::Subscriber goal2D_;
    ros::Subscriber states_;    	
    
    // Config parameters
    float alpha_;
    float laser_fov_;
	float laser_obstacle_range_;
    float min_curve_radius_;
    float P_velocity_, I_velocity_, D_velocity_, P_line_, I_line_, D_line_, P_orientation_, I_orientation_, D_orientation_;
	float pixels_per_centimeter_;
	float resize_grid_scale_;                
	float vel_range_, steer_range_;
    int beam_;
    int index_skip_;    
	int max_error_tolerance_;	
    int obstacle_threshold_;

	//Handle Mouse
	Point goal_;
	bool goal_selected_;
	
    // AutoNOMOS monitoring
    float current_yaw_;
    Point current_position_;	
    std_msgs::Float32 velocity_;
	std_msgs::Float32 steering_;
    std_msgs::Float32 control_actions_[2];	
    
	// Path planning	
    vector<Point> planned_path_;    
    
    // Maps
    cv_bridge::CvImagePtr cv_ptr_1;	
	Mat laser_map_;
    Mat path_map_;
    Mat previous_map_;	    
    Mat sky_image_;			
		
	// States
    int state_machine_;
    bool object_too_close_;
    bool object_near_;
	bool status_ok_;	
	bool first_move_;	
    
    // Control	
	float integral_error_velocity_, integral_error_line_, integral_error_orientation_;
    int local_begin_index_, local_goal_index_;
	float prev_error_velocity_, prev_error_line_, prev_error_orientation_;		
    
    // For evaluation only
    vector<float> errorSteeringVector;
	vector<int> goalPointsVector;
	std::string filename;
	int testNum;

    // For orientation error estimation
	tf::Matrix3x3 Rt_;

	// To move cylinder
	gazebo_msgs::ModelState state_;

	// Rviz
	visualization_msgs::Marker unit_box_0;    
	visualization_msgs::Marker unit_cylinder_0;
	visualization_msgs::Marker unit_cylinder_1;    
	visualization_msgs::Marker unit_box_1;
	visualization_msgs::Marker unit_box_2;
	visualization_msgs::Marker unit_box_3;
	sensor_msgs::ImagePtr rviz_image_;
	sensor_msgs::ImagePtr rviz_laser_;
	sensor_msgs::ImagePtr rviz_rrt_;
	
public:
	Skycam();
	~Skycam();
    
	bool NewObjectInPath();
    float* GetControlActions(vector<Point> planned_path, Point current_position, 
        int& local_starting_point, int& local_goal_point);
    Mat GetCarMap(Mat sky_image);
    Mat GetOccupancyGrid(Mat sky_image);
    Point GetPosition(Mat carMap);
    tf::Vector3 InverseHatMap(tf::Matrix3x3 re);    
    vector<Point> RRT(Mat analyzedMap, Mat analyzedCar, Point goal, int beam, float scale, float angle);			
    void DrawMapElements();
    void FinishTrack();	
    void GetStates(const gazebo_msgs::ModelStates &msg);
    void GoalFromRviz(const geometry_msgs::PoseStamped &goal_rviz);	
    void MoveCylinder(const std_msgs::Int8 &key);	
    void ReadLaser(const sensor_msgs::LaserScan &msg);		
    void SkyImage(const sensor_msgs::ImageConstPtr &msg);		

private:
	static void onMouseStatic( int event, int x, int y, int flags, void* point );
	void onMouse( int event, int x, int y, int flags );
};

#endif
