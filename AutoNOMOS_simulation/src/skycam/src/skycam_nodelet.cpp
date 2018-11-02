#include "skycam.h"

Skycam::Skycam():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init image_viewer");
	laser_ = nh_.subscribe("/laser_scan", 10, &Skycam::ReadLaser, this);        
    sky_image_receiver_ = it_.subscribe("/sky_camera/image_raw", 1, &Skycam::SkyImage, this);
    steering_publisher_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/steering", 3);
    velocity_publisher_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/velocity", 3);	
        
    // For cylinder moving    
    key_ = nh_.subscribe("/cylinder/keyboard", 10, &Skycam::MoveCylinder, this);
    state_publisher_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 3);   
    states_ = nh_.subscribe("/gazebo/model_states", 10, &Skycam::GetStates, this);
    
    // For Rviz
    goal2D_ = nh_.subscribe("/move_base_simple/goal", 10, &Skycam::GoalFromRviz, this);
    path3D_ = nh_.advertise<geometry_msgs::Point>("/skycam/plannedPath", 10);    
    rviz_image_publisher_ = nh_.advertise<sensor_msgs::Image>("/skycam/sky_image", 10);
    rviz_laser_publisher_ = nh_.advertise<sensor_msgs::Image>("/skycam/laser_image", 10);
    rviz_rrt_publisher_ = nh_.advertise<sensor_msgs::Image>("/skycam/RRT", 10);
            
    nh_.getParam("pixels_per_centimeter_", pixels_per_centimeter_); //pixels per centimeter
    nh_.getParam("resize_grid_scale_", resize_grid_scale_);
    nh_.getParam("min_curve_radius_", min_curve_radius_); 
    nh_.getParam("beam_", beam_);
    nh_.getParam("vel_range_", vel_range_);
    nh_.getParam("steer_range_", steer_range_);
    nh_.getParam("max_error_tolerance_", max_error_tolerance_);
    nh_.getParam("index_skip_", index_skip_);
    nh_.getParam("P_velocity_", P_velocity_);
    nh_.getParam("I_velocity_", I_velocity_);
    nh_.getParam("D_velocity_", D_velocity_);
    nh_.getParam("P_line_", P_line_);
    nh_.getParam("I_line_", I_line_);
    nh_.getParam("D_line_", D_line_);
    nh_.getParam("P_orientation_", P_orientation_);
    nh_.getParam("I_orientation_", I_orientation_);
    nh_.getParam("D_orientation_", D_orientation_);
    nh_.getParam("alpha_", alpha_);
    nh_.getParam("laser_fov_", laser_fov_);
    nh_.getParam("laser_obstacle_range_", laser_obstacle_range_);
    nh_.getParam("obstacle_threshold_", obstacle_threshold_);
    
    goal_selected_ = false;
    state_machine_ = 1;
    object_near_ = false;
    object_too_close_ = false; 
    prev_error_velocity_ = 0;
    prev_error_line_ = 0;
    prev_error_orientation_ = 0;
    integral_error_velocity_ = 0;
    integral_error_line_ = 0;
    integral_error_orientation_ = 0;
    
    goal_.x = -1;
    goal_.y = -1;       
    laser_map_ = Mat::zeros(640, 640, CV_8UC1);                    
}

Skycam::~Skycam()
{

}

bool Skycam::NewObjectInPath()
{
    float degrees_per_radian = 180/M_PI;
    int laser_range = 600; // given in centimeters
    Mat current_map = GetOccupancyGrid(sky_image_);
    Mat previous_map = previous_map_.clone();    
    
    // Get an image representing the laser reach in its Field Of View
    Mat mask = Mat::zeros(current_map.rows, current_map.cols, CV_8UC1);
    Point center = Skycam::GetPosition(GetCarMap(sky_image_));
    float rotation = (-current_yaw_ * degrees_per_radian) - laser_fov_;
    cv::Rect rectangle = cv::Rect(int(center.x - laser_map_.cols/2), 
        int(center.y - laser_map_.rows/2), int(2*laser_map_.cols/2), int(2*laser_map_.rows/2));
    cv::Rect roi = rectangle & cv::Rect(0, 0, current_map.cols, current_map.rows);                                                                     
    ellipse(mask, center, Size(laser_range*pixels_per_centimeter_, laser_range*pixels_per_centimeter_ ),                 
    rotation, 0, (2*laser_fov_), Scalar(255), -1);
    
    // Get the difference between the current map and the old map in the Region Of Interest           
    Mat diff = current_map - previous_map;    
    bitwise_and(diff, mask, diff);
    diff = diff(roi);             
    Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 5,5));
    erode(diff, diff, element);
    dilate(diff, diff, element);
        
    double absolute_difference = cv::sum(diff)[0] / 255; 
    if(absolute_difference >= obstacle_threshold_)
    {
        // The object in front of the car wasn't there when tha path was planned. New path required.
        state_machine_ = 1;
        velocity_.data = 0;
        steering_.data = 0;
        velocity_publisher_.publish(velocity_);                        
        steering_publisher_.publish(steering_);                        
        status_ok_ = false;
        // Show the analyzed conflicting area
        rviz_laser_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", diff).toImageMsg();
        rviz_laser_publisher_.publish(rviz_laser_);                           
        sleep(1);
        return true;
    }
    else return false;
}

float* Skycam::GetControlActions(vector<Point> planned_path, Point current_position, 
    int& local_starting_point, int& local_goal_point){
        
    float *control_actions = new float[2]; // [velocity, steering]
    float speed_limit;
    float velocity_data, steering_data;
    float error_velocity, error_line, error_orientation;  
    float derivative_error_velocity, derivative_error_line, derivative_error_orientation;
    tf::Vector3 z_axis(0,0,1);

    float local_error = cv::norm(planned_path[local_goal_point]-current_position);
    if(local_error < max_error_tolerance_)
    {
        local_starting_point = local_goal_point;
        local_goal_point = local_goal_point + index_skip_;                         
        if(local_starting_point < planned_path.size() - 1 && local_goal_point >= planned_path.size() )
        {
                // Local goal is last point
                local_goal_point = planned_path.size() - 1;
        }
        // The vehicle is at goal
        if(local_starting_point == planned_path.size() - 1)
        {            
            FinishTrack();
            control_actions[0] = 0;
            control_actions[1] = 0;
            return control_actions;
        }  
    }
    // Define a speed limit proportional to the distance to the final goal
    speed_limit = vel_range_ * (planned_path.size() - local_starting_point/1.33)/planned_path.size();
    
    float local_line_orientation = atan2(planned_path[local_goal_point].y - planned_path[local_starting_point].y,
        planned_path[local_goal_point].x - planned_path[local_starting_point].x);
    tf::Vector3 current_position_vector = tf::Vector3(current_position.x, current_position.y, 0);
    tf::Vector3 local_origin = tf::Vector3(planned_path[local_starting_point].x, planned_path[local_starting_point].y, 0);
    tf::Vector3 goal_vector = tf::Vector3(planned_path[local_goal_point].x, planned_path[local_goal_point].y, 0);
    // Here we get the rotation matrix corresponding to the orientation of the local line to follow
    tf::Quaternion q = tf::Quaternion(z_axis, -local_line_orientation);
    tf::Matrix3x3 r = tf::Matrix3x3(q);
    
    // Velocity errors
    tf::Vector3 local_goal_vector = r*(goal_vector - local_origin);
    tf::Vector3 local_current_position_vector = r*(current_position_vector - local_origin);
    // We calculate the error along the local x axis        
    error_velocity = (local_goal_vector[0] - local_current_position_vector[0]) / pixels_per_centimeter_;
    if(first_move_ == true)
    {
        prev_error_velocity_ = error_velocity;
    }
    integral_error_velocity_ = integral_error_velocity_ + error_velocity;    
    derivative_error_velocity = error_velocity - prev_error_velocity_; 
    prev_error_velocity_ = error_velocity;     
    // If the accumulated integral error is greater than the speed limit, we force it down so 
    // that it results 1 when multiplied by the integral constant
    if(abs(integral_error_velocity_ * I_velocity_) > speed_limit){
        integral_error_velocity_ = (speed_limit/I_velocity_) * (integral_error_velocity_/abs(integral_error_velocity_));
    }
    
    // Line errors
    error_line = -(local_goal_vector[1] - local_current_position_vector[1]) / pixels_per_centimeter_;   
    // The next commented line is used for evaluation purposes
    // errorSteeringVector.push_back(errorSteering);        
    if(first_move_ == true)
    {
            prev_error_line_ = error_line;
    }                     
    integral_error_line_ = integral_error_line_ + error_line; 
    derivative_error_line = error_line - prev_error_line_;
    prev_error_line_ = error_line;
    // Same as we did with velocity when the accumulated integral error was out of range
    if(abs(integral_error_line_ * I_line_) > steer_range_){
        integral_error_line_ = (steer_range_/I_line_) * (integral_error_line_/abs(integral_error_line_));
    }
    
    // Orientation errors  
    // Please refer to the chapter to look for the paper from which this calcs were coded
    tf::Quaternion q0(z_axis, -local_line_orientation);         
    tf::Matrix3x3 Rd(q0);
    Rd = Rd.transpose();                         
    tf::Matrix3x3 Re = Rd * Rt_;
    tf::Vector3 y = InverseHatMap(Re);                                
    y *= asin(y.length())/y.length(); 
    // The index 2 corresponds to the z-axis                                                               
    error_orientation = y[2]; 
    if(first_move_ == true)
    {
            prev_error_orientation_ = error_orientation;
            first_move_ = false;
    }                        
    integral_error_orientation_ = integral_error_orientation_ + error_orientation; 
    derivative_error_orientation = error_orientation - prev_error_orientation_;
    prev_error_orientation_ = error_orientation;
    // The same as with the line integral error when it overflows the maximum range
    if(abs(integral_error_orientation_ * I_orientation_) > steer_range_) integral_error_orientation_ = (steer_range_/I_orientation_) * (integral_error_orientation_/abs(integral_error_orientation_));
    
    // Velocity PID
    velocity_data = P_velocity_ * error_velocity + 
        I_velocity_ * integral_error_velocity_ + 
        D_velocity_ * derivative_error_velocity;
    // Be sure the ouput signal is within the accepted range
    if(abs(velocity_data) > speed_limit) velocity_data = speed_limit * velocity_data/abs(velocity_data);                
    
    // Steering PID
    steering_data = P_line_ * error_line + 
        I_line_ * integral_error_line_ +
        D_line_ * derivative_error_line;                        
    // Here we do the weighted sum of line and orientation PIDs
    steering_data = (1-alpha_)*steering_data + 
        alpha_*(P_orientation_ * error_orientation + 
                I_orientation_ * integral_error_orientation_ + 
                D_orientation_ * derivative_error_orientation);                                                
    // Be sure the ouput signal is within the accepted range
    if(abs(steering_data) > steer_range_) steering_data = steer_range_ * steering_data/abs(steering_data); 
    
    // Adjust velocity in curves    
    // The velocity decreases exponentially, being 20% of the maximum velocity at maximum steering
    float curve_limit = vel_range_ * (1 - (exp(abs(steering_data) * log(1.8)/steer_range_) - 1)); 
    if(abs(velocity_data) > abs(curve_limit)) velocity_data =  curve_limit * velocity_data/abs(velocity_data); 
    
    // For worst case scenario when the AutoNOMOS needs to move backwards, the steering needs to change direction
    if(velocity_data < 0) steering_data = -0.5 * steering_data; 
    
    control_actions[0] = velocity_data;        
    control_actions[1] = steering_data;
    return control_actions;
}                     

Mat Skycam::GetCarMap(Mat sky_image)
{
    Mat car_map;
    int green_lower_bound = 70;
    int green_upper_bound = 255; 
    Mat lab_space_image;
	Mat lab_channels[3];
        
    cvtColor(sky_image, lab_space_image, COLOR_BGR2Lab);
    split(lab_space_image,lab_channels);       
             
    threshold(lab_channels[1], car_map, green_lower_bound, green_upper_bound, 1); 
    Mat element = getStructuringElement( MORPH_RECT,
                                   Size( 10,10));
    dilate(car_map, car_map, element);
    return car_map;
}

Mat Skycam::GetOccupancyGrid(Mat sky_image)
{
    Mat occupancy_grid;
    int red_lower_bound = 140;
    int red_upper_bound = 255;
    Mat lab_space_image;
	Mat lab_channels[3];
    
    cvtColor(sky_image, lab_space_image, COLOR_BGR2Lab);
    split(lab_space_image,lab_channels);  
    threshold(lab_channels[1], occupancy_grid, red_lower_bound, red_upper_bound, 0);   
    return occupancy_grid;
}

Point Skycam::GetPosition(Mat carMap)
{
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;   
        Point calculatedPosition;  
        
        findContours( carMap, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        vector<Moments> mu(contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
                mu[i] = moments( contours[i], false );
        }        
        
        vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        {
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
        }
        
        float maxArea = 0;
        for( int i = 0; i < contours.size(); i++ )
        {
                if(contourArea(contours[i]) > maxArea)
                {
                        maxArea = contourArea(contours[i]);
                        calculatedPosition = Point(int(mc[i].x), int(mc[i].y));
                }
        } 
        
        return calculatedPosition;
}

tf::Vector3 Skycam::InverseHatMap(tf::Matrix3x3 re)
{
        tf::Matrix3x3 transposed(re.transpose());
        for( int i = 0; i < 3; i += 1 )
        {
                for( int j = 0; j < 3; j += 1 )
                {
                        re[i][j] -= transposed[i][j];
                }
        }
        
        tf::Vector3 y(0.5*re[2][1],0.5*re[0][2],0.5*re[1][0]);
                        
        return y;
}

vector<Point> Skycam::RRT(Mat map_to_analyze, Mat car_map, Point goal, int beam, float scale, float car_pose)
{    
    bool draw_paths = true;
    double slope;        
    double x, y;
    double x_best, y_best;
    
    float camera_rotation_respect_to_world = M_PI_2;
    float centimeters_per_meter = 100;
    float current_angle = car_pose - camera_rotation_respect_to_world; 
    float distance_to_move;
    float horizontal_neighborhood = 50, vertical_neighborhood = 40;    
    float min_expected_cost;
    float min_radius = min_curve_radius_*scale*resize_grid_scale_, min_distance_to_move = ceil(min_radius/2), max_distance_to_move = floor(min_radius*M_PI);    
    float radius, angle;
    float x_target, y_target, y0;        
    
    geometry_msgs::Point path_point_3d;   
     
    int center;    
    int counter, counter2;
    int death_path_counter = 0;
    int death_path_limit = 100;
    int goal_radius = 20, structuring_element_radius = 25;
    int maximum_radius = 139;    
    int quadrant;
    
    Scalar white = Scalar(255,255,255); 
    
    vector<Point> planned_path;
    
    vector<tf::Vector3> visited_3d_points;
    vector<tf::Vector3> best_visited_3d_points;     
        
    previous_map_ = map_to_analyze.clone();    
    path_point_3d.z = 0;   
    path3D_.publish(path_point_3d);
    
    Mat goal_map = Mat::zeros(map_to_analyze.rows, map_to_analyze.cols, CV_8UC1);
    circle(goal_map, goal, goal_radius*scale, white, -1);     
    cv::resize(goal_map, goal_map, cv::Size(), resize_grid_scale_, resize_grid_scale_);
    
    tf::Vector3 goal_vector(goal.x, goal.y, 0);
    goal_vector *= resize_grid_scale_;  
    goal = Point(goal_vector[0], goal_vector[1]);
    
    cv::resize(map_to_analyze, map_to_analyze, cv::Size(), resize_grid_scale_, resize_grid_scale_);
    cv::resize(car_map, car_map, cv::Size(), resize_grid_scale_, resize_grid_scale_);
    
    Point resized_map_center = Point(int(map_to_analyze.cols/2), int(map_to_analyze.rows/2));    
    Point car_position = Skycam::GetPosition(car_map);
    Point local_car_position = Point(car_position.x - resized_map_center.x, resized_map_center.y - car_position.y);         
    
    goal = Point(goal_vector[0] - resized_map_center.x, resized_map_center.y - goal_vector[1]);   
 
    planned_path.push_back(car_position);
    planned_path.push_back(car_position);   
          
    Mat candidate_paths_map = Mat::zeros(map_to_analyze.rows, map_to_analyze.cols, CV_8UC1); 
    
    // The occupancy grid gets dilated to afford with the AutoNOMOS dimensions while planning the path
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                   Size( structuring_element_radius*2*scale*resize_grid_scale_,
                                            structuring_element_radius*2*scale*resize_grid_scale_));
    Mat dilated;
    dilate(map_to_analyze, dilated, element);
    
    Point initial_position = local_car_position;
    float initial_angle = current_angle;
                             
    while(draw_paths)
    {       
        counter = 0;
        min_expected_cost = sqrt(pow(map_to_analyze.cols, 2) + pow(map_to_analyze.rows, 2));
        counter2 = 0;
        while(counter < beam)
        {              
            visited_3d_points.clear();          
            distance_to_move = rand() % int(min_distance_to_move) + min_distance_to_move;
            quadrant = rand() % 3;                   
            bool path_is_acceptable = true;
            Point analyzed_pixel;
            tf::Vector3 car_coordinates(local_car_position.x, local_car_position.y, 0); 
            //ROS_INFO_STREAM("Cartesian Coordinates: " << car_coordinates[0] << ", " << car_coordinates[1] << ", " << car_coordinates[2]);  
            tf::Vector3 z_axis(0,0,1);
            tf::Quaternion q0(z_axis, current_angle);
            tf::Matrix3x3 rotation(q0);          
            if(quadrant > 0)
            {
                radius = rand() % int(maximum_radius*scale*resize_grid_scale_) + int(min_radius); //first number is maximum extra expected radius to wide open the curve
                angle = distance_to_move/radius;
                slope = pow(-1, quadrant - 1)/tan(angle);
                center = pow(-1, quadrant - 1)*radius;
                y_target = radius*sin(angle);
                x_target = center - pow(-1, quadrant - 1)*radius*cos(angle);                                                                
                                        
                uchar pixel;                                     
                for( int index = 0; index <= ceil(abs(center) - radius*cos(angle)); index++ )
                {       
                    int x_value = index * pow(-1, quadrant - 1);
                    tf::Vector3 new_car_position(x_value, sqrt(pow(radius,2) - pow(x_value-center,2)), 0); 
                    
                    visited_3d_points.push_back(rotation*new_car_position + car_coordinates);
                    analyzed_pixel.x = int(visited_3d_points[index][0] + resized_map_center.x);
                    analyzed_pixel.y = int(resized_map_center.y - visited_3d_points[index][1]);
                    if(analyzed_pixel.y > map_to_analyze.rows || analyzed_pixel.y < 0 || analyzed_pixel.x > map_to_analyze.cols || analyzed_pixel.x < 0)
                    {
                        path_is_acceptable = false;
                        visited_3d_points.clear();
                        break;
                    }
                    pixel = dilated.at<uchar>(analyzed_pixel.y, analyzed_pixel.x);
                    if(int(pixel) == 255)
                    {                                                
                        path_is_acceptable = false;
                        visited_3d_points.clear();
                        break;
                    }
                    else
                    {
                        candidate_paths_map.at<uchar>(analyzed_pixel.y, analyzed_pixel.x) = 255;
                    }
                    pixel = goal_map.at<uchar>(analyzed_pixel.y, analyzed_pixel.x);
                    if(int(pixel) == 255)
                    {                                                
                        path_is_acceptable = true;
                        draw_paths = false;
                        break;
                    }
                                                      
                }
                
                if(path_is_acceptable)
                {
                   y0 = y_target - slope*x_target;   
                   tf::Vector3 P1(0, y0, 0);                            
                   tf::Vector3 P2(x_target, y_target, 0);                               
                   P1 = rotation*P1;
                   P2 = rotation*P2;
                   x = P2[0]-P1[0];
                   y = P2[1]-P1[1];                                
                }                                                
            }
            else
            {
                slope = 32000; // i.e. significatively great, for an almost vertical slope
                uchar pixel;                                     
                for( int index = 0; index <= distance_to_move; index++ )
                {       
                    tf::Vector3 new_car_position(0, index, 0);                                
                    visited_3d_points.push_back(rotation*new_car_position + car_coordinates);                                
                    analyzed_pixel.x = int(visited_3d_points[index-1][0] + resized_map_center.x);
                    analyzed_pixel.y = int(resized_map_center.y - visited_3d_points[index-1][1]);                                
                    if(analyzed_pixel.y > map_to_analyze.rows || analyzed_pixel.y < 0 || analyzed_pixel.x > map_to_analyze.cols || analyzed_pixel.x < 0)
                    {
                        path_is_acceptable = false;
                        visited_3d_points.clear();
                        break;
                    }
                    pixel = dilated.at<uchar>(analyzed_pixel.y, analyzed_pixel.x);
                    if(int(pixel) == 255)
                    {
                        path_is_acceptable = false;
                        visited_3d_points.clear();
                        break;
                    }
                    else
                    {
                        candidate_paths_map.at<uchar>(analyzed_pixel.y, analyzed_pixel.x) = 255;
                    }
                    pixel = goal_map.at<uchar>(analyzed_pixel.y, analyzed_pixel.x);
                    if(int(pixel) == 255)
                    {
                        path_is_acceptable = true;
                        draw_paths = false;
                        break;
                    }                                                                
                        
                }
                
                if(path_is_acceptable)
                {
                   tf::Vector3 P1(0, 0, 0);                            
                   tf::Vector3 P2(0, distance_to_move, 0);                                 
                   P1 = rotation*P1;
                   P2 = rotation*P2;
                   x = P2[0]-P1[0];
                   y = P2[1]-P1[1];                              
                }   
                    
            }                
            if(path_is_acceptable)
            {
                counter++;
                Point local_goal_candidate = Point(int(visited_3d_points.back()[0]), int(visited_3d_points.back()[1]));
                //ROS_INFO_STREAM("Candidate: " << local_goal_candidate);
                //ROS_INFO_STREAM("Goal: " << goal);
                float expected_cost = cv::norm(goal-local_goal_candidate);
                bool flag2 = false;     
                int width = int(horizontal_neighborhood*scale*resize_grid_scale_);                                                           
                for(int i2 = 1; i2 <= width; i2++)
                {
                    for(int i1 = 1; i1 <= 2; i1++)
                    {
                        uchar pixel;
                        float direction, neighbor_x, neighbor_y; 
                        direction = atan2(y,x);
                        neighbor_x = visited_3d_points.back()[0] + i2*scale*resize_grid_scale_*cos(direction + M_PI_2*pow(-1, i1)) + resized_map_center.x;
                        neighbor_y = resized_map_center.y - (visited_3d_points.back()[1] + i2*scale*resize_grid_scale_*sin(direction + M_PI_2*pow(-1, i1)));
                        if(neighbor_y > dilated.rows || neighbor_y < 0 || neighbor_x > map_to_analyze.cols || neighbor_x < 0)
                        {
                            pixel = 255;
                        }                                                
                        else
                        {
                            pixel = dilated.at<uchar>(neighbor_y, neighbor_x);
                        }
                        if(pixel == 255)
                        {
                            expected_cost = expected_cost + expected_cost*(width - i2)/width;
                            flag2 = true;
                            break;
                        }
                    }
                    if(flag2 == true) break;
                }      
                flag2 = false;                          
                for(int i2 = 1; i2 <= width; i2++)
                {

                    uchar pixel;
                    float direction, neighbor_x, neighbor_y; 
                    direction = atan2(y,x);
                    neighbor_x = visited_3d_points.back()[0] + i2*scale*resize_grid_scale_*cos(direction) + resized_map_center.x;
                    neighbor_y = resized_map_center.y - (visited_3d_points.back()[1] + i2*scale*resize_grid_scale_*sin(direction));
                    if(neighbor_y > dilated.rows || neighbor_y < 0 || neighbor_x > map_to_analyze.cols || neighbor_x < 0)
                    {
                        pixel = 255;
                    }
                    else
                    {
                        pixel = dilated.at<uchar>(neighbor_y, neighbor_x);
                    }
                    if(pixel == 255)
                    {
                        expected_cost = expected_cost + expected_cost*(width - i2)/width;
                        flag2 = true;
                        break;
                    }                                        
                    if(flag2 == true) break;
                }                                   
                if(expected_cost < min_expected_cost)
                {
                    float direction, neighbor_x, neighbor_y; 
                    direction = atan2(y,x);
                    int height = int(vertical_neighborhood*scale*resize_grid_scale_); 
                    neighbor_x = visited_3d_points.back()[0] + height*cos(direction) + resized_map_center.x;
                    neighbor_y = resized_map_center.y - (visited_3d_points.back()[1] + height*sin(direction));
                    uchar pixel;
                    if(neighbor_y > dilated.rows || neighbor_y < 0 || neighbor_x > map_to_analyze.cols || neighbor_x < 0)
                    {
                        pixel = 255;
                    }
                    else
                    {
                        pixel = dilated.at<uchar>(neighbor_y, neighbor_x);
                    }
                    if(pixel == 0)
                    {
                        min_expected_cost = expected_cost;
                        best_visited_3d_points.clear();
                        for(int index = 0; index < visited_3d_points.size(); index++)
                        {                                                                                                
                            best_visited_3d_points.push_back(visited_3d_points[index]);                                                                                                    
                        }
                        x_best = x;
                        y_best = y;
                    }
                    else
                    {
                        counter--;
                        counter2++;
                    }
                }                                
            }
            else
            {
                    counter2++;                                
            }                        
            if(counter == beam || draw_paths == false)
            {                 
                counter = beam;                                                               
                for(int index = 1; index < best_visited_3d_points.size(); index++)
                {
                    Point new_point = Point(int(best_visited_3d_points[index][0] + resized_map_center.x), int(resized_map_center.y - best_visited_3d_points[index][1]));                                        
                    planned_path.push_back(new_point);          
                    path_point_3d.x = ((best_visited_3d_points[index][0] / resize_grid_scale_) / scale) / centimeters_per_meter;
                    path_point_3d.y = ((best_visited_3d_points[index][1] / resize_grid_scale_) / scale) / centimeters_per_meter;
                    path_point_3d.z = 0.2;    
                    path3D_.publish(path_point_3d);  
                }
                visited_3d_points.clear();                                
                current_angle = atan2(y_best,x_best) - camera_rotation_respect_to_world; 
                local_car_position = Point(planned_path.back().x - resized_map_center.x, resized_map_center.y - planned_path.back().y);
        
                //ROS_INFO_STREAM("Final angle: " << current_angle);
                //ROS_INFO_STREAM("Final positon in cartesian plane: " << local_car_position);
                rviz_rrt_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", (candidate_paths_map + dilated + goal_map)).toImageMsg();        
                rviz_rrt_publisher_.publish(rviz_rrt_); 
                //ROS_INFO_STREAM("Goal resized: " << goal);
                cv::waitKey(1);
            }                                                                        
            if(counter2 - counter > beam)
            {
                death_path_counter++;
                ROS_INFO_STREAM("Counter to dead: " << death_path_counter);
                if(death_path_counter >= death_path_limit)
                {
                    object_too_close_ = true;
                    counter = beam;
                    draw_paths = false;
                }
                best_visited_3d_points.clear();   
                planned_path.push_back(car_position);                                                           
                planned_path.clear();
                planned_path.push_back(car_position);
                local_car_position = initial_position;
                current_angle = initial_angle;
                counter = beam;
                candidate_paths_map = Mat::zeros(map_to_analyze.rows, map_to_analyze.cols, CV_8UC1);       
                path_point_3d.z = 0;   
                path3D_.publish(path_point_3d);                                                 
            }
        }
    }                
                    
    path_point_3d.z = 0.4;
    path3D_.publish(path_point_3d);
    return planned_path;
}

void Skycam::DrawMapElements()
{
    Scalar blue = Scalar(255,0,0), green = Scalar(0,255,0);
    int radius = 1, thick = 5, thin = 1;
    
    // Draw Goal	
    circle(sky_image_, goal_, radius, blue, thick);
    // Draw path if it exists
    if(planned_path_.size() > 1)
    {
        for (int point = 0; point < planned_path_.size(); point++)
        {
            circle(sky_image_, planned_path_[point], radius, blue, thick);                        
            circle(path_map_, planned_path_[point], radius, blue, thick);                        
        }
    } 
    if(state_machine_ == 2) circle(path_map_, current_position_, radius, green, thin);  
    rviz_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sky_image_).toImageMsg();
    rviz_image_publisher_.publish(rviz_image_);        
    rviz_rrt_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", path_map_).toImageMsg();
    rviz_rrt_publisher_.publish(rviz_rrt_);   
}

void Skycam::FinishTrack()
{
    // "Turn-off" the PID paramters
    integral_error_velocity_ = 0;
    integral_error_line_ = 0;
    integral_error_orientation_ = 0;
    
    ROS_INFO_STREAM("Success!!!!");
    // An artificial 3D point to notify Rviz about the success
    geometry_msgs::Point path_point_3d; 
    path_point_3d.z = 0;
    path3D_.publish(path_point_3d);
    // Reset the variables for a new goal
    goal_.x = -1;
    goal_.y = -1;
    goal_selected_ = false;
    velocity_publisher_.publish(velocity_);                        
    steering_publisher_.publish(steering_);
    state_machine_ = 1;
    goal_selected_ = false;
    // The commented block is used for numeric evaluation purposes
    /*filename = "/home/sutadasuto/EK_AutoNOMOS/pathFollowing" + std::to_string(testNum) + ".csv";
    ofstream myfile;                                
    myfile.open(filename.c_str());      
    for(int i=0; i<errorSteeringVector.size(); i++)
    {
        myfile << errorSteeringVector[i]/pixels_per_centimeter_;
        if(i < errorSteeringVector.size()) myfile << ",";
        else myfile << std::endl;
    }
    myfile.close();                                
    testNum++;*/
}

void Skycam::GetStates(const gazebo_msgs::ModelStates &msg)
{
    for(int model = 0; model < msg.name.size(); model++)
    {
        if(msg.name[model] == "unit_cylinder_0")
        {
            state_.model_name = msg.name[model];
            state_.pose = msg.pose[model];
            state_.twist = msg.twist[model];
        }
        if(msg.name[model] == "AutoNOMOS_mini")
        {
            float imuX = msg.pose[model].orientation.x;
            float imuY = msg.pose[model].orientation.y;
            float imuZ = msg.pose[model].orientation.z;
            float imuW = msg.pose[model].orientation.w;
            
            tf::Quaternion q1(imuX,imuY,imuZ,imuW);
            tf::Matrix3x3 m(q1);
            Rt_ = m;
            
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_yaw_ = float(yaw);
            //ROS_INFO_STREAM("Yaw: " << current_yaw_);
            status_ok_ = true;                       
        }
    }
}

void Skycam::GoalFromRviz(const geometry_msgs::PoseStamped &goal_rviz)
{
    FinishTrack();
    float centimeters_per_meter = 100;
    goal_.x = int(goal_rviz.pose.position.x * centimeters_per_meter * pixels_per_centimeter_ + 
                    sky_image_.cols / 2);
    goal_.y = int(sky_image_.cols / 2 - 
                    goal_rviz.pose.position.y * centimeters_per_meter * pixels_per_centimeter_);
    ROS_INFO_STREAM(goal_);  
    state_machine_ = 1;
}

void Skycam::MoveCylinder(const std_msgs::Int8 &key)
{        
        if(key.data == 1)
        {
                state_.pose.position.y =  state_.pose.position.y + 0.1;
        }
        else if(key.data == 2)
        {
                state_.pose.position.x =  state_.pose.position.x + 0.1;
        }
        else if(key.data == 3)
        {
                state_.pose.position.y =  state_.pose.position.y - 0.1;
        }
        else if(key.data == 4)
        {
                state_.pose.position.x =  state_.pose.position.x - 0.1;
        }
        
        state_publisher_.publish(state_);
}

void Skycam::onMouseStatic( int event, int x, int y, int flags, void* point )
{
        if(event == CV_EVENT_LBUTTONDOWN) 
        {
                cv::Point& goal_ = *(cv::Point*) point;
                goal_.x = x;
                goal_.y = y;
                ROS_INFO_STREAM(goal_);        
        }
}

void Skycam::ReadLaser(const sensor_msgs::LaserScan &msg)
{
    float centimeters_per_meter = 100;
    float radians_per_degree = M_PI/180.0;
    object_near_ = false;
    vector<float> distances;
    distances = msg.ranges;
    float angle;
    float angle_min = msg.angle_min;
    float orientation = M_PI_2; // first laser begins at 90° from the front of the car
    float angle_increment = msg.angle_increment;
    float range_max = msg.range_max/3; 
    float range_min = 0.10;
    
    laser_map_ = Mat::zeros(int(range_max*centimeters_per_meter*pixels_per_centimeter_*2 + 1), 
        int(range_max*centimeters_per_meter*pixels_per_centimeter_*2 + 1), CV_8UC1);  
           
    
    for (int i = 0; i < distances.size(); i++)
    {
        angle = angle_min + orientation + i*angle_increment;
        
        if(angle_min + i*angle_increment < laser_fov_ * radians_per_degree || 
        angle_min + i*angle_increment > 2*M_PI - laser_fov_ * radians_per_degree)
        {
            if(distances[i] <= laser_obstacle_range_ && distances[i] >= range_min)
            {
                    object_near_ = true;                                
                    ROS_INFO_STREAM("Obstacle detected at " << distances[i]);
            }
        }
        
        if(abs(distances[i]) <= range_max && distances[i] >= range_min)
        {
            int x = int((range_max + distances[i] * cos(angle)) * centimeters_per_meter * pixels_per_centimeter_);
            int y = int((range_max - distances[i] * sin(angle)) * centimeters_per_meter * pixels_per_centimeter_);
            laser_map_.at<uchar>(y,x) = 255;
        }
    }
    rviz_laser_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", laser_map_).toImageMsg();
    rviz_laser_publisher_.publish(rviz_laser_);         
}

void Skycam::SkyImage(const sensor_msgs::ImageConstPtr &msg)
{	
    //Get image from cam
    if(goal_.x > 0) goal_selected_ = true;
	try
	{
		cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}	
	sky_image_ = cv_ptr_1->image;     
    
    current_position_ = Skycam::GetPosition(GetCarMap(sky_image_));
    
    if(goal_selected_ == true && state_machine_ == 1)
    {       
        if(status_ok_ == true)
        {                 
            Scalar white = Scalar(255,255,255);
            path_map_ = Mat(sky_image_.rows, sky_image_.cols, CV_8UC3, white);
            
            planned_path_ = Skycam::RRT(GetOccupancyGrid(sky_image_), GetCarMap(sky_image_), goal_, beam_, pixels_per_centimeter_, current_yaw_); 
            if(planned_path_.size() > 1)
            {      
                // The RRT implementation reduces the map's resolution to do pixel-wise grids
                // The following loop increases the calculated path's resolution to match the original map
                for (int point = 0; point < planned_path_.size(); point++)
                {
                    planned_path_[point].x = planned_path_[point].x/resize_grid_scale_;
                    planned_path_[point].y = planned_path_[point].y/resize_grid_scale_;                                                                                       
                }
                local_begin_index_ = 0; 
                local_goal_index_ = index_skip_; 
                // Be sure the goal index is not beyond the size of the planned path
                if(local_begin_index_ < planned_path_.size() - 1 && local_goal_index_ >= planned_path_.size() )
                {
                    local_goal_index_ = planned_path_.size() - 1;
                }                    
                first_move_ = true;
                object_too_close_ = false;                             
                state_machine_ = 2;                 
            }
        }                    
    }            
    
    if(object_near_) 
    {                
        if(NewObjectInPath())
        {
            velocity_.data = -5;
            steering_.data = 0;
            velocity_publisher_.publish(velocity_);                        
            steering_publisher_.publish(steering_);
            sleep(1);
            velocity_.data = 0;
            velocity_publisher_.publish(velocity_); 
        }
    }
    if(object_too_close_)
    {
        velocity_.data = -5;
        steering_.data = 0;
        velocity_publisher_.publish(velocity_);                        
        steering_publisher_.publish(steering_);
        sleep(1);
        velocity_.data = 0;
        velocity_publisher_.publish(velocity_); 
    }
    
    if(state_machine_ == 2)
    {   
        // Take control actions to follow the planned path
        float *signals = GetControlActions(planned_path_, current_position_, local_begin_index_, local_goal_index_);        
        velocity_.data = signals[0];
        steering_.data = signals[1];
        velocity_publisher_.publish(velocity_);                        
        steering_publisher_.publish(steering_);
    }
           
    DrawMapElements();                                                
} 
