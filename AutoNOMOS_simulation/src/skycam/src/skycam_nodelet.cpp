#include "skycam.h"

skycam::skycam():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init image_viewer");
	sky_image_ = it_.subscribe("/sky_camera/image_raw", 1, &skycam::skyImage, this);
        laser_ = nh_.subscribe("/laser_scan", 10, &skycam::readLaser, this);        
        velocity_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/velocity", 3);
	steering_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/steering", 3);
        
        //Para mover el cilindro
        states_ = nh_.subscribe("/gazebo/model_states", 10, &skycam::getStates, this);
        key_ = nh_.subscribe("/cylinder/keyboard", 10, &skycam::moveCylinder, this);
        state_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 3);        
        
        //Para Rviz
        path3D_ = nh_.advertise<geometry_msgs::Point>("/skycam/plannedPath", 10);
        goal2D_ = nh_.subscribe("/move_base_simple/goal", 10, &skycam::goalFromRviz, this);
        rvizImage_ = nh_.advertise<sensor_msgs::Image>("/skycam/sky_image", 10);
        rvizLaser_ = nh_.advertise<sensor_msgs::Image>("/skycam/laser_image", 10);
        rvizRRT_ = nh_.advertise<sensor_msgs::Image>("/skycam/RRT", 10);
                
        nh_.getParam("flag1", flag1);
        nh_.getParam("flag", flag);
        nh_.getParam("scale", scale); //pixels per centimeter
        nh_.getParam("resizeScale", resizeScale);
        nh_.getParam("minCurve", minCurve); 
        minCurve = minCurve; // * 1.1;
        nh_.getParam("beam", beam); 
        nh_.getParam("angle", angle); 
        nh_.getParam("velRange", velRange);
        nh_.getParam("steerRange", steerRange);
        nh_.getParam("errorTolerance", errorTolerance);
        nh_.getParam("skip", skip);
        nh_.getParam("Pv", Pv);
        nh_.getParam("Iv", Iv);
        nh_.getParam("Dv", Dv);
        nh_.getParam("Ps", Ps);
        nh_.getParam("Is", Is);
        nh_.getParam("Ds", Ds);
        nh_.getParam("Ps2", Ps2);
        nh_.getParam("Is2", Is2);
        nh_.getParam("Ds2", Ds2);
        nh_.getParam("alpha", alpha);
        nh_.getParam("laserFOV", laserFOV);
        nh_.getParam("laserRange", laserRange);
        nh_.getParam("obstacleThreshold", obstacleThreshold);
        newObstacleFlag = false;
        closeFlag = false;
        deadPathCounter= 0;        
        prevErrorVelocity = 0;
        prevErrorSteering = 0;
        prevErrorSteering2 = 0;
        IerrorVelocity = 0;
        IerrorSteering = 0;
        IerrorSteering2 = 0;
        DerrorVelocity = 0;
        DerrorSteering = 0;
        DerrorSteering2 = 0;
        
        testNum = 0;
        routes = Mat(640, 640, CV_8UC3, Scalar(255,255,255));
        
        goal.x = -1;
        goal.y = -1;       
        laserMap = Mat::zeros(640, 640, CV_8UC1);
        
        axis = tf::Vector3(0,0,1);                       
}

skycam::~skycam()
{

}

void skycam::skyImage(const sensor_msgs::ImageConstPtr &msg)
{	
        //Get image from cam
        if(goal.x > 0) flag1 = true;
	try
	{
		cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}	
	sky_image = cv_ptr_1->image;                       
        
        //Image Segmentation (Original code from https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/)
        cvtColor(sky_image, lab, COLOR_BGR2Lab);
        split(lab,labChannels);       
                 
        threshold(labChannels[1], map, 140, 255, 0);        
        threshold(labChannels[1], car, 70, 255, 1); 
        Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 10,10));
        dilate(car, car, element);       
        
        position = skycam::getPosition(car);
        //ROS_INFO_STREAM(position);

        
        //Draw Goal	
        Point center = goal;
        circle(sky_image, center,1,CV_RGB(0,0,255),5);
        
        if(flag1 == true && flag == 1)
        {       
                if(readyFlag == true)
                {                    
                        routes = Mat(640, 640, CV_8UC3, Scalar(255,255,255));                   
                        route = skycam::RRT(map, car, goal, beam, scale, angle); 
                        if(route.size() > 1)
                        {                
                                for (int point = 0; point < route.size(); point++)
                                {
                                        route[point].x = route[point].x/resizeScale;
                                        route[point].y = route[point].y/resizeScale; 
                                        circle(routes, route[point],1,CV_RGB(0,0,255),2);                                                                                       
                                }
                                p1 = 0; 
                                p2 = skip;  
                                if(p1 < route.size() - 1 && p2 >= route.size() )
                                {
                                        p2 = route.size() - 1;
                                }                                
                                speedLimit = velRange;
                                closeFlag = false;                             
                                flag = 2; 
                                beginTrack = true;
                        }
                }  
                else
                {
                        ROS_INFO_STREAM("Waiting orientation");
                }                     
        }
        
        if(closeFlag == true)
        {
                velocity.data = -5;
                steering.data = 0;
                velocity_.publish(velocity);                        
                steering_.publish(steering);
                sleep(1);
                velocity.data = 0;
                velocity_.publish(velocity); 
        }
                
        
        if(route.size() > 1)
        {
                for (int point = 0; point < route.size(); point++)
                {
                        circle(sky_image, route[point],1,CV_RGB(0,0,255),5);                        
                }
        } 
        
        
        if(newObstacleFlag == true) 
        {                
                
                Mat currentMap = map.clone();
                Mat previousMap = prevMap.clone();
                Mat mask = Mat::zeros(currentMap.rows, currentMap.cols, CV_8UC1);
                Point center = skycam::getPosition(car);
                float rotation = (-angle * 180 / 3.1416) - laserFOV;
                //ROS_INFO_STREAM("Rotation: " << rotation);
                cv::Rect _rectangle = cv::Rect(int(center.x - laserMap.cols/2), 
                int(center.y - laserMap.rows/2), int(2*laserMap.cols/2), int(2*laserMap.rows/2));
                cv::Rect roi = _rectangle & cv::Rect(0, 0, currentMap.cols, currentMap.rows);                                                         
                
                ellipse(mask, center, Size( 600*scale, 600*scale ),                 
                rotation, 0, (2*laserFOV), 
                Scalar(255), -1);
                //ellipse(previousMap, center, Size( 600*scale, 600*scale ),                 
                //rotation, 0, (360-2*laserFOV), 
                //Scalar(0), -1);
                //rvizLaser = cv_bridge::CvImage(std_msgs::Header(), "mono8", currentMap).toImageMsg();
                //rvizLaser_.publish(rvizLaser);  
                //rvizRRT = cv_bridge::CvImage(std_msgs::Header(), "mono8", prevMap).toImageMsg();
                //rvizRRT_.publish(rvizRRT);  
                //sleep(5); 
                              
                Mat diff;
                //absdiff(currentMap(roi), prevMap(roi), diff);
                diff = currentMap - previousMap;
                bitwise_and(diff, mask, diff);
                diff = diff(roi);
                //rvizLaser = cv_bridge::CvImage(std_msgs::Header(), "mono8", diff).toImageMsg();
                //rvizLaser_.publish(rvizLaser);  
                //sleep(5); 
                
                
                Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 5,5));
                erode(diff, diff, element);
                dilate(diff, diff, element);
                
                
                double s = cv::sum( diff )[0] / 255;       
                
                
                if(s >= obstacleThreshold)
                {                        
                        flag = 1;
                        velocity.data = 0;
                        steering.data = 0;
                        velocity_.publish(velocity);                        
                        steering_.publish(steering);                        
                        readyFlag = false;
                        //imshow("Difference", diff);
                        //cv::waitKey(1);
                        rvizLaser = cv_bridge::CvImage(std_msgs::Header(), "mono8", diff).toImageMsg();
                        rvizLaser_.publish(rvizLaser);                           
                        sleep(1);
                }
        }
        
        if(flag == 2)
        {                
                float errorNorm = cv::norm(route[p2]-position);
                circle(routes, position,1,CV_RGB(0,255,0),1);  
                //ROS_INFO_STREAM("P1: " << p1 << " P2: " << p2);
                //ROS_INFO_STREAM("Last Index = " << route.size()-1);
                //ROS_INFO_STREAM("Carro: " << position << " Meta: " << route[p2]);
                //ROS_INFO_STREAM("Error Norm: " << errorNorm);
                if(errorNorm < errorTolerance)
                {
                        p1 = p2;
                        p2 = p2+skip;                         
                        //IerrorVelocity = velocity.data/Iv;
                        //IerrorSteering = 0;
                        //IerrorSteering2 = 0;
                        if(p1 < route.size() - 1 && p2 >= route.size() )
                        {
                                p2 = route.size() - 1;
                        }
                        if(p1 == route.size() - 1)
                        {                                
                                velocity.data = 0;
                                steering.data = 0;
                                IerrorVelocity = 0;
                                IerrorSteering = 0;
                                IerrorSteering2 = 0;
                                DerrorVelocity = 0;
                                DerrorSteering = 0;
                                DerrorSteering2 = 0;
                                
                                ROS_INFO_STREAM("Success!!!!");
                                geometry_msgs::Point point3D; 
                                point3D.z = 0;
                                path3D_.publish(point3D);
                                goal.x = -1;
                                goal.y = -1;
                                flag1 = false;
                                velocity_.publish(velocity);                        
                                steering_.publish(steering);
                                flag = 1;
                                flag1 = false;
                                filename = "/home/sutadasuto/EK_AutoNOMOS/pathFollowing" + std::to_string(testNum) + ".csv";
                                ofstream myfile;                                
                                myfile.open(filename.c_str());      
                                for(int i=0; i<errorSteeringVector.size(); i++)
                                {
                                        myfile << errorSteeringVector[i]/scale;
                                        if(i < errorSteeringVector.size()) myfile << ",";
                                        else myfile << std::endl;
                                }
                                myfile.close();                                
                                testNum++;
                                route.clear();
                        }
                        else
                        {
                                speedLimit = velRange * (route.size() - p1/1.33)/route.size();
                        }
                }
                else
                {
                        direction = atan2(route[p2].y - route[p1].y, route[p2].x - route[p1].x);
                        positionVector = tf::Vector3(position.x, position.y, 0);
                        originVector = tf::Vector3(route[p1].x, route[p1].y, 0);
                        goalVector = tf::Vector3(route[p2].x, route[p2].y, 0);
                        q = tf::Quaternion(axis, -direction);
                        R = tf::Matrix3x3(q);
                        
                        // Velocity errors
                        goalVector = R*(goalVector - originVector);
                        positionVector = R*(positionVector - originVector);
                        errorVelocity = (goalVector[0] - positionVector[0]) / scale;
                        if(beginTrack == true)
                        {
                                prevErrorVelocity = errorVelocity;
                        }
                        IerrorVelocity = IerrorVelocity + errorVelocity;    
                        DerrorVelocity = errorVelocity - prevErrorVelocity; 
                        prevErrorVelocity = errorVelocity;     
                        if(abs(IerrorVelocity * Iv) > speedLimit) IerrorVelocity = (speedLimit/Iv) * (IerrorVelocity/abs(IerrorVelocity));
                        
                        // Line errors
                        errorSteering = -(goalVector[1] - positionVector[1]) / scale;   
                        errorSteeringVector.push_back(errorSteering);
                        goalPointsVector.push_back(p2);
                        if(beginTrack == true)
                        {
                                prevErrorSteering = errorSteering;
                        }                     
                        IerrorSteering = IerrorSteering + errorSteering; 
                        DerrorSteering = errorSteering - prevErrorSteering;
                        prevErrorSteering = errorSteering;
                        if(abs(IerrorSteering * Is) > steerRange) IerrorSteering = (steerRange/Is) * (IerrorSteering/abs(IerrorSteering));
                        
                        // Orientation errors                
                        tf::Vector3 refAxis(0,0,1);
                        tf::Quaternion q0(refAxis, -direction);         
                        tf::Matrix3x3 Rimu(q0);
                        Rd = Rimu;   
                        Rd = Rd.transpose();                         
                        Re = Rd * Rt;
                        y = inverseHatMap(Re);                                
                        y *= asin(y.length())/y.length();                                                                
                        errorSteering2 = y[2]; 
                        if(beginTrack == true)
                        {
                                prevErrorSteering2 = errorSteering2;
                                beginTrack = false;
                        }                        
                        IerrorSteering2 = IerrorSteering2 + errorSteering2; 
                        DerrorSteering2 = errorSteering2 - prevErrorSteering2;
                        prevErrorSteering2 = errorSteering2;
                        if(abs(IerrorSteering2 * Is2) > steerRange) IerrorSteering2 = (steerRange/Is2) * (IerrorSteering2/abs(IerrorSteering2));
                        
                        // Velocity PID
                        velocity.data = Pv * errorVelocity + Iv * IerrorVelocity + Dv * DerrorVelocity;                                                
                        if(abs(velocity.data) > speedLimit) velocity.data = speedLimit * velocity.data/abs(velocity.data);
                        
                        // Steering PID
                        steering.data = Ps * errorSteering + Is * IerrorSteering + Ds * DerrorSteering;                        
                        steering.data = (1-alpha) * steering.data + alpha * (Ps2 * errorSteering2 + Is2 * IerrorSteering2 + Ds2 * DerrorSteering2);                                                
                        if(abs(steering.data) > steerRange) steering.data = steerRange * steering.data/abs(steering.data); 
                        
                        // Adjust velocity in curves                       
                        float curveLimit = velRange * (1 - (exp(abs(steering.data) * log(1.8)/70) - 1)); 
                        if(abs(velocity.data) > abs(curveLimit)) velocity.data =  curveLimit * velocity.data/abs(velocity.data); 
                        if(velocity.data < 0) steering.data = -0.5 * steering.data; 
                        //ROS_INFO_STREAM("Goal: X' = " << goalVector[0] << " Y' = " << goalVector[1]);
                        //ROS_INFO_STREAM("Position: X' = " << positionVector[0] << " Y' = " << positionVector[1]);
                        //ROS_INFO_STREAM("Error Velocity: " << errorVelocity);
                        //ROS_INFO_STREAM("I Velocity: " << IerrorVelocity);
                        //ROS_INFO_STREAM("D Velocity: " << DerrorVelocity);
                        //ROS_INFO_STREAM("Error Steering: " << errorSteering);   
                        //ROS_INFO_STREAM("I Steering: " << IerrorSteering);
                        //ROS_INFO_STREAM("D Steering: " << DerrorSteering);  
                        ////ROS_INFO_STREAM("Angle = " << angle << " Direction  = " << -direction);                   
                        //ROS_INFO_STREAM("Error Steering 2: " << errorSteering2);                        
                        //ROS_INFO_STREAM("I Steering 2: " << IerrorSteering2);
                        //ROS_INFO_STREAM("D Steering 2: " << DerrorSteering2);                     
                        ////ROS_INFO_STREAM("Speed Limit: " << speedLimit);                                           
                        //ROS_INFO_STREAM("Velocity: " << velocity.data);
                        //ROS_INFO_STREAM("Steering: " << steering.data);
                        velocity_.publish(velocity);                        
                        steering_.publish(steering);
                }
        }
        
        //imshow("Sky eye", sky_image);         
        rvizImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sky_image).toImageMsg();
        rvizImage_.publish(rvizImage);
        //setMouseCallback("Sky eye", skycam::onMouseStatic, (void*)&goal);                      
        //imshow("Detected Map", map);
        //imshow("Detected Car", car);
        //imshow("Laser", laserMap);    
        //imshow("Routes", routes);
        //cv::waitKey(1);          
        rvizRRT = cv_bridge::CvImage(std_msgs::Header(), "bgr8", routes).toImageMsg();
        rvizRRT_.publish(rvizRRT);                                                   
}                                

void skycam::onMouseStatic( int event, int x, int y, int flags, void* point )
{
        //that->onMouse( event,  x, y, flags );
        if(event == CV_EVENT_LBUTTONDOWN) 
        {
                cv::Point& goal = *(cv::Point*) point;
                goal.x = x;
                goal.y = y;
                ROS_INFO_STREAM(goal);        
        }
}

void skycam::goalFromRviz(const geometry_msgs::PoseStamped &goalRviz)
{
        goal.x = int(goalRviz.pose.position.x * 100 * scale + sky_image.cols / 2);
        goal.y = int(sky_image.cols / 2 - goalRviz.pose.position.y * 100 * scale);
        ROS_INFO_STREAM(goal);  
}

void skycam::readLaser(const sensor_msgs::LaserScan &msg)
{
        newObstacleFlag = false;
        vector<float> distances;
        distances = msg.ranges;
        float angle;
        float angle_min = msg.angle_min;
        float orientation = 1.5708;
        float angle_increment = msg.angle_increment;
        float range_max = msg.range_max/3;        
        
        laserMap = Mat::zeros(int(range_max*100*scale*2 + 1), int(range_max*100*scale*2 + 1), CV_8UC1);  
               
        
        for (int i = 0; i < distances.size(); i++)
        {
                angle = angle_min + orientation + i*angle_increment;
                
                if(angle_min + i*angle_increment < laserFOV * 3.1416/180.0 || 
                angle_min + i*angle_increment > 6.2832 - laserFOV * 3.1416/180.0)
                {
                        if(distances[i] <= laserRange && distances[i] >= 0.10)
                        {
                                newObstacleFlag = true;                                
                                ROS_INFO_STREAM("Obstacle detected at " << distances[i]);
                        }
                }
                
                if(abs(distances[i]) <= range_max && distances[i] >= 0.10)
                {
                        int x = int((range_max + distances[i] * cos(angle)) * 100 * scale);
                        int y = int((range_max - distances[i] * sin(angle)) * 100 * scale);
                        //ROS_INFO_STREAM(angle);
                        //ROS_INFO_STREAM(distances[i]);
                        //ROS_INFO_STREAM(x);
                        //ROS_INFO_STREAM(y);
                        laserMap.at<uchar>(y,x) = 255;
                        //ROS_INFO_STREAM(pixel);             
                        //pixel[0] = 255;
                        //pixel[1] = 255;
                        //pixel[2] = 255;
                        //laserMap.at<Vec3b>(Point(x,y)) = pixel;
                        //circle(laserMap, Point(x,y),0,CV_RGB(255,255,255),1);
                }
        }
        rvizLaser = cv_bridge::CvImage(std_msgs::Header(), "mono8", laserMap).toImageMsg();
        rvizLaser_.publish(rvizLaser);         
}


Point skycam::getPosition(Mat carMap)
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


void skycam::getStates(const gazebo_msgs::ModelStates &msg)
{
        for(int model = 0; model < msg.name.size(); model++)
        {
                if(msg.name[model] == "unit_cylinder_0")
                {
                        state.model_name = msg.name[model];
                        state.pose = msg.pose[model];
                        state.twist = msg.twist[model];
                }
                if(msg.name[model] == "AutoNOMOS_mini")
                {
                        float imuX = msg.pose[model].orientation.x;
                        float imuY = msg.pose[model].orientation.y;
                        float imuZ = msg.pose[model].orientation.z;
                        float imuW = msg.pose[model].orientation.w;
                        
                        tf::Quaternion q1(imuX,imuY,imuZ,imuW);
                        tf::Matrix3x3 m(q1);
                        Rt = m;
                        
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        angle = float(yaw);
                        //ROS_INFO_STREAM("Yaw: " << angle);
                        readyFlag = true;                       
                }
        }
}

void skycam::moveCylinder(const std_msgs::Int8 &key)
{        
        if(key.data == 1)
        {
                state.pose.position.y =  state.pose.position.y + 0.1;
        }
        else if(key.data == 2)
        {
                state.pose.position.x =  state.pose.position.x + 0.1;
        }
        else if(key.data == 3)
        {
                state.pose.position.y =  state.pose.position.y - 0.1;
        }
        else if(key.data == 4)
        {
                state.pose.position.x =  state.pose.position.x - 0.1;
        }
        
        state_.publish(state);
}

tf::Vector3 skycam::inverseHatMap(tf::Matrix3x3 re)
{
        tf::Matrix3x3 reNormal(re);
        tf::Matrix3x3 trans(re.transpose());
        for( int i = 0; i < 3; i += 1 )
        {
                for( int j = 0; j < 3; j += 1 )
                {
                        reNormal[i][j] -= trans[i][j];
                }
        }
        
        tf::Vector3 y(0.5*reNormal[2][1],0.5*reNormal[0][2],0.5*reNormal[1][0]);
                        
        return y;
}

vector<Point> skycam::RRT(Mat analyzedMap, Mat analyzedCar, Point goal, int beam, float scale, float angle)
{
        geometry_msgs::Point point3D;    
        point3D.z = 0;   
        path3D_.publish(point3D);
        prevMap = analyzedMap.clone();
        float currentAngle = angle - 1.5708; //+ 1.5708;                
        //ROS_INFO_STREAM("Current Angle: " << currentAngle);                
        
        tf::Vector3 goalVector(goal.x, goal.y, 0);
        goalVector *= resizeScale;
        Mat goalMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);
        circle(goalMap, goal, 20*scale, CV_RGB(255,255,255), -1);
        goal = Point(goalVector[0], goalVector[1]);
        cv::resize(goalMap, goalMap, cv::Size(), resizeScale, resizeScale);
        cv::resize(analyzedMap, analyzedMap, cv::Size(), resizeScale, resizeScale);
        cv::resize(analyzedCar, analyzedCar, cv::Size(), resizeScale, resizeScale);
        Point planeOrigin = Point(int(analyzedMap.cols/2), int(analyzedMap.rows/2));
        Point scaledPosition = skycam::getPosition(analyzedCar);
        Point currentPosition = Point(scaledPosition.x - planeOrigin.x, planeOrigin.y - scaledPosition.y);
        goal = Point(goalVector[0] - planeOrigin.x, planeOrigin.y - goalVector[1]);
        //ROS_INFO_STREAM("Current Position: " << currentPosition);
        Mat pathMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);        
        vector<Point> pathVector;
        pathVector.push_back(scaledPosition);
        pathVector.push_back(scaledPosition);             
        Mat testMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);
        Mat dilated;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 25*2*scale*resizeScale,25*2*scale*resizeScale));
        dilate(analyzedMap, dilated, element);
        rvizRRT = cv_bridge::CvImage(std_msgs::Header(), "mono8", (dilated)).toImageMsg();
        rvizRRT_.publish(rvizRRT); 
        
        Point posInit = currentPosition;
        float angInit = currentAngle;
        
        //float minCurve = 72; // closest turn radius possible for the car (i.e. at maximum steerling angle); given in centimeters
        float minRad = minCurve * scale * resizeScale;
        float maxDis = floor(minRad * 3.1416);
        float minDis = ceil(minRad/2);
        
        float dis;
        int cuadrante;
        
        float rad, ang;
        int center;
        float targetX, targetY, y0;
        double slope;        
        double X, Y;
        double XBest, YBest;
        
        bool whileFlag = true;
        int counter, counter2;
        float minExpectedCost;
        vector<tf::Vector3> xyz;
        vector<tf::Vector3> xyzBest;                  
        while(whileFlag == true)
        {       
                counter = 0;
                minExpectedCost = sqrt(pow(analyzedMap.cols, 2) + pow(analyzedMap.rows, 2));
                counter2 = 0;
                while(counter < beam)
                {              
                        xyz.clear();          
                        dis = rand() % int(minDis) + minDis;
                        //ROS_INFO_STREAM("Distance: " << dis); 
                        cuadrante = rand() % 3;   
                        //ROS_INFO_STREAM("Cuadrante: " << cuadrante);                           
                        bool flag = true;
                        Point analyzed;
                        tf::Vector3 actualCoordinates(currentPosition.x, currentPosition.y, 0); 
                        //ROS_INFO_STREAM("Cartesian Coordinates: " << actualCoordinates[0] << ", " << actualCoordinates[1] << ", " << actualCoordinates[2]);  
                        tf::Vector3 refAxis(0,0,1);
                        tf::Quaternion q0(refAxis, currentAngle);
                        tf::Matrix3x3 R(q0);          
                        if(cuadrante > 0)
                        {
                                rad = rand() % int(139*scale*resizeScale) + int(minRad); //first number is maximum extra expected radius to wide open the curve
                                ang = dis/rad;
                                slope = pow(-1, cuadrante - 1)/tan(ang);
                                center = pow(-1, cuadrante - 1)*rad;
                                targetY = rad*sin(ang);
                                targetX = center - pow(-1, cuadrante - 1)*rad*cos(ang);                        
                                
                                //ROS_INFO_STREAM("Center: " << center << " Radius: " << rad); 
                                //ROS_INFO_STREAM("Target X: " << targetX << " Target Y: " << targetY);                                             
                                                        
                                uchar pixel;                                     
                                for( int index = 0; index <= ceil(abs(center) - rad*cos(ang)); index++ )
                                {       
                                        int xValue = index * pow(-1, cuadrante - 1);
                                        tf::Vector3 carVector(xValue, sqrt(pow(rad,2) - pow(xValue-center,2)), 0); 
                                        
                                        xyz.push_back(R*carVector + actualCoordinates);
                                        analyzed.x = int(xyz[index][0] + planeOrigin.x);
                                        analyzed.y = int(planeOrigin.y - xyz[index][1]);
                                        //ROS_INFO_STREAM("Analyzed X: " << analyzed.x << " Analyzed Y: " << analyzed.y); 
                                        if(analyzed.y > analyzedMap.rows || analyzed.y < 0 || analyzed.x > analyzedMap.cols || analyzed.x < 0)
                                        {
                                                flag = false;
                                                xyz.clear();
                                                break;
                                        }
                                        pixel = dilated.at<uchar>(analyzed.y, analyzed.x);
                                        if(int(pixel) == 255)
                                        {                                                
                                                flag = false;
                                                xyz.clear();
                                                break;
                                        }
                                        else
                                        {
                                                testMap.at<uchar>(analyzed.y, analyzed.x) = 255;
                                        }
                                        pixel = goalMap.at<uchar>(analyzed.y, analyzed.x);
                                        if(int(pixel) == 255)
                                        {                                                
                                                flag = true;
                                                whileFlag = false;
                                                break;
                                        }
                                                                      
                                }
                                
                                if(flag == true)
                                {
                                       y0 = targetY - slope*targetX;   
                                       tf::Vector3 P1(0, y0, 0);                            
                                       tf::Vector3 P2(targetX, targetY, 0);                               
                                       P1 = R*P1;
                                       P2 = R*P2;
                                       X = P2[0]-P1[0];
                                       Y = P2[1]-P1[1];                                
                                }                                                
                        }
                        else
                        {
                                slope = 32000; // i.e. significatively great, for an almost vertical slope
                                uchar pixel;                                     
                                for( int index = 0; index <= dis; index++ )
                                {       
                                        tf::Vector3 carVector(0, index, 0);                                
                                        xyz.push_back(R*carVector + actualCoordinates);                                
                                        analyzed.x = int(xyz[index-1][0] + planeOrigin.x);
                                        analyzed.y = int(planeOrigin.y - xyz[index-1][1]);                                
                                        if(analyzed.y > analyzedMap.rows || analyzed.y < 0 || analyzed.x > analyzedMap.cols || analyzed.x < 0)
                                        {
                                                flag = false;
                                                xyz.clear();
                                                break;
                                        }
                                        pixel = dilated.at<uchar>(analyzed.y, analyzed.x);
                                        if(int(pixel) == 255)
                                        {
                                                flag = false;
                                                xyz.clear();
                                                break;
                                        }
                                        else
                                        {
                                                testMap.at<uchar>(analyzed.y, analyzed.x) = 255;
                                        }
                                        pixel = goalMap.at<uchar>(analyzed.y, analyzed.x);
                                        if(int(pixel) == 255)
                                        {
                                                flag = true;
                                                whileFlag = false;
                                                break;
                                        }                                                                
                                        
                                }
                                
                                if(flag == true)
                                {
                                       tf::Vector3 P1(0, 0, 0);                            
                                       tf::Vector3 P2(0, dis, 0);                                 
                                       P1 = R*P1;
                                       P2 = R*P2;
                                       X = P2[0]-P1[0];
                                       Y = P2[1]-P1[1];                              
                                }   
                                
                        }                
                        if(flag == true)
                        {
                                counter++;
                                Point candidatePosition = Point(int(xyz.back()[0]), int(xyz.back()[1]));
                                //ROS_INFO_STREAM("Candidate: " << candidatePosition);
                                //ROS_INFO_STREAM("Goal: " << goal);
                                float expectedCost = cv::norm(goal-candidatePosition);
                                bool flag2 = false;     
                                int width = int(50*scale*resizeScale);                                                           
                                for(int i2 = 1; i2 <= width; i2++)
                                {
                                        for(int i1 = 1; i1 <= 2; i1++)
                                        {
                                                uchar pixel;
                                                float direction, newX, newY; 
                                                direction = atan2(Y,X);
                                                newX = xyz.back()[0] + i2*scale*resizeScale*cos(direction + 1.5708*pow(-1, i1)) + planeOrigin.x;
                                                newY = planeOrigin.y - (xyz.back()[1] + i2*scale*resizeScale*sin(direction + 1.5708*pow(-1, i1)));
                                                if(newY > dilated.rows || newY < 0 || newX > analyzedMap.cols || newX < 0)
                                                {
                                                        pixel = 255;
                                                }                                                
                                                else
                                                {
                                                        //testMap.at<uchar>(newY, newX) = 255;
                                                        pixel = dilated.at<uchar>(newY, newX);
                                                }
                                                if(pixel == 255)
                                                {
                                                        expectedCost = expectedCost + expectedCost*(width - i2)/width;
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
                                        float direction, newX, newY; 
                                        direction = atan2(Y,X);
                                        newX = xyz.back()[0] + i2*scale*resizeScale*cos(direction) + planeOrigin.x;
                                        newY = planeOrigin.y - (xyz.back()[1] + i2*scale*resizeScale*sin(direction));
                                        if(newY > dilated.rows || newY < 0 || newX > analyzedMap.cols || newX < 0)
                                        {
                                                pixel = 255;
                                        }
                                        else
                                        {
                                                //testMap.at<uchar>(newY, newX) = 255;
                                                pixel = dilated.at<uchar>(newY, newX);
                                        }
                                        if(pixel == 255)
                                        {
                                                expectedCost = expectedCost + expectedCost*(width - i2)/width;
                                                flag2 = true;
                                                break;
                                        }                                        
                                        if(flag2 == true) break;
                                }                                   
                                if(expectedCost < minExpectedCost)
                                {
                                        float direction, newX, newY; 
                                        direction = atan2(Y,X);
                                        int width = int(40*scale*resizeScale); 
                                        newX = xyz.back()[0] + width*cos(direction) + planeOrigin.x;
                                        newY = planeOrigin.y - (xyz.back()[1] + width*sin(direction));
                                        uchar pixel;
                                        if(newY > dilated.rows || newY < 0 || newX > analyzedMap.cols || newX < 0)
                                        {
                                                pixel = 255;
                                        }
                                        else
                                        {
                                                pixel = dilated.at<uchar>(newY, newX);
                                        }
                                        if(pixel == 0)
                                        {
                                                minExpectedCost = expectedCost;
                                                xyzBest.clear();
                                                for(int index = 0; index < xyz.size(); index++)
                                                {                                                                                                
                                                        xyzBest.push_back(xyz[index]);                                                                                                    
                                                }
                                                XBest = X;
                                                YBest = Y;
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
                        if(counter == beam || whileFlag == false)
                        {                 
                                counter = beam;                                                               
                                for(int index = 1; index < xyzBest.size(); index++)
                                {
                                        Point newPoint = Point(int(xyzBest[index][0] + planeOrigin.x), int(planeOrigin.y - xyzBest[index][1]));                                        
                                        pathVector.push_back(newPoint);
                                        pathMap.at<uchar>(newPoint) = 255;                  
                                        point3D.x = ((xyzBest[index][0] / resizeScale) / scale) / 100;
                                        point3D.y = ((xyzBest[index][1] / resizeScale) / scale) / 100;
                                        point3D.z = 0.2;    
                                        path3D_.publish(point3D);  
                                        //ROS_INFO_STREAM("Marco...");
                                }
                                xyz.clear();                                
                                //whileFlag = false;  
                                currentAngle = atan2(YBest,XBest) - 1.5708; 
                                currentPosition = Point(pathVector.back().x - planeOrigin.x, planeOrigin.y - pathVector.back().y);
                        
                                //ROS_INFO_STREAM("Final angle: " << currentAngle);
                                //ROS_INFO_STREAM("Final positon in cartesian plane: " << currentPosition);
                                //imshow("Sky eye", sky_image);                                        
                                //imshow("Planned path", testMap + dilated + goalMap);
                                rvizRRT = cv_bridge::CvImage(std_msgs::Header(), "mono8", (testMap + dilated + goalMap)).toImageMsg();
                                rvizRRT_.publish(rvizRRT); 
                                //ROS_INFO_STREAM("Goal resized: " << goal);
                                //imshow("Car", analyzedCar);
                                cv::waitKey(1);
                        }                                                                        
                        if(counter2 - counter > beam)
                        {
                                deadPathCounter++;
                                ROS_INFO_STREAM("Counter to dead: " << deadPathCounter);
                                if(deadPathCounter >= 100)
                                {
                                        deadPathCounter = 0;
                                        closeFlag = true;
                                        counter = beam;
                                        whileFlag = false;
                                        //ROS_INFO_STREAM("Close flag");
                                }
                                xyzBest.clear();   
                                pathVector.push_back(scaledPosition);                                                           
                                pathVector.clear();
                                pathVector.push_back(scaledPosition);
                                pathMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);
                                currentPosition = posInit;
                                currentAngle = angInit;
                                counter = beam;
                                testMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);       
                                point3D.z = 0;   
                                path3D_.publish(point3D);                                                 
                        }
                }
        }                
                        
        //imshow("Result", pathMap + analyzedMap + goalMap); 
        point3D.z = 0.4;
        path3D_.publish(point3D);
        return pathVector;
}
