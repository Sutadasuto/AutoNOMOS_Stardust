#include "skycam.h"

skycam::skycam():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init image_viewer");
	sky_image_ = it_.subscribe("/sky_camera/image_raw", 1, &skycam::skyImage, this);
        laser_ = nh_.subscribe("/laser_scan", 1, &skycam::readLaser, this);
        pose_ = nh_.subscribe("/tf", 1, &skycam::getAttitude, this);
        velocity_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/velocity", 3);
	steering_ = nh_.advertise<std_msgs::Float32>("/AutoNOMOS_mini/manual_control/steering", 3);
                
        nh_.getParam("flag1", flag1);
        nh_.getParam("flag", flag);
        nh_.getParam("scale", scale); //pixels per centimeter
        nh_.getParam("resizeScale", resizeScale);
        nh_.getParam("angle", angle); 
        nh_.getParam("velRange", velRange);
        nh_.getParam("steerRange", steerRange);
        nh_.getParam("errorTolerance", errorTolerance);
        nh_.getParam("Pv", Pv);
        nh_.getParam("Iv", Iv);
        nh_.getParam("Dv", Dv);
        nh_.getParam("Ps", Ps);
        nh_.getParam("Is", Is);
        nh_.getParam("Ds", Ds);
        
        
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
        equalizeHist( labChannels[1], aChannel );
        equalizeHist( labChannels[2], bChannel );        
        threshold(aChannel, aChannel, 10, 255, 0);
        threshold(bChannel, bChannel, 10, 255, 0);
        bitwise_not(aChannel, car);
        dilate(car, car, Mat(), Point(-1, -1), 2, 1, 1);
        bitwise_not(car, aChannel);
        bChannel.copyTo(map, aChannel);
        
        position = skycam::getPosition(car);
        //ROS_INFO_STREAM(position);
        
        //Draw Goal	
        Point center = goal;
        circle(sky_image, center,1,CV_RGB(0,0,255),5);
        
        if(flag1 == true && flag == 1)
        {                                
                route = skycam::RRT(map, car, goal, 100, scale, angle);                 
                for (int point = 0; point < route.size(); point++)
                {
                        route[point].x = route[point].x/resizeScale;
                        route[point].y = route[point].y/resizeScale;
                }
                p1 = 0; 
                p2 = 1;  
                IerrorVelocity = 0;
                speedLimit = velRange;             
                goal.x = -1;
                goal.y = -1;
                flag1 = false;
                flag = 3;                        
        }
                
        
        if(route.size() > 0)
        {
                for (int point = 0; point < route.size(); point++)
                {
                        circle(sky_image, route[point],1,CV_RGB(0,0,255),5);
                        ROS_INFO_STREAM("Point Center " << point << ": " << route[point]);
                }
        }  
        
        if(flag == 2)
        {
                ROS_INFO_STREAM("P1: " << p1 << " P2: " << p2);
                ROS_INFO_STREAM("Last Index = " << route.size()-1);
                float errorNorm = cv::norm(route[p2]-position);
                ROS_INFO_STREAM("Carro: " << position << " Meta: " << route[p2]);
                ROS_INFO_STREAM("Error Norm: " << errorNorm);
                if(errorNorm < errorTolerance)
                {
                        p1++;
                        p2++;
                        IerrorSteering = 0;
                        if(p2 == route.size())
                        {
                                flag = 1;
                                velocity.data = 0;
                                steering.data = 0;
                        }
                        else
                        {
                                speedLimit = velRange * (route.size() - p1)/route.size();
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
                        goalVector = R*(goalVector - originVector);
                        positionVector = R*(positionVector - originVector);
                        errorVelocity = (goalVector[0] - positionVector[0]) / scale;
                        IerrorVelocity = IerrorVelocity + errorVelocity;
                        IerrorSteering = IerrorSteering + errorSteering;
                        errorSteering = -(goalVector[1] - positionVector[1]) / scale;
                        ROS_INFO_STREAM("Goal: X' = " << goalVector[0] << " Y' = " << goalVector[1]);
                        ROS_INFO_STREAM("Position: X' = " << positionVector[0] << " Y' = " << positionVector[1]);
                        ROS_INFO_STREAM("Error Velocity: " << errorVelocity);
                        ROS_INFO_STREAM("Error Steering: " << errorSteering);
                        velocity.data = Pv * errorVelocity + Iv * IerrorVelocity;
                        ROS_INFO_STREAM("Speed Limit: " << speedLimit);
                        if(abs(velocity.data) > speedLimit) velocity.data = speedLimit * velocity.data/abs(velocity.data);                        
                        steering.data = Ps * errorSteering + Is * IerrorSteering;
                        if(abs(steering.data) > steerRange) steering.data = steerRange * steering.data/abs(steering.data);
                        
                        ROS_INFO_STREAM("Velocity: " << velocity.data);
                        ROS_INFO_STREAM("Steering: " << steering.data);
                        velocity_.publish(velocity);                        
                        steering_.publish(steering);
                }
        }
        
        imshow("Sky eye", sky_image);         
        setMouseCallback("Sky eye", skycam::onMouseStatic, (void*)&goal);                      
        //imshow("Detected Map", map);
        //imshow("Detected Car", car);
        //imshow("Laser", laserMap);                                        
        
        cv::waitKey(1);
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

void skycam::readLaser(const sensor_msgs::LaserScan &msg)
{
        vector<float> distances;
        distances = msg.ranges;
        float angle;
        float angle_min = msg.angle_min;
        float orientation = 1.5708;
        float angle_increment = msg.angle_increment;
        float range_max = msg.range_max / 2;        
        
        laserMap = Mat::zeros(int(range_max*100*scale*2 + 1), int(range_max*100*scale*2 + 1), CV_8UC1);  
               
        
        for (int i = 0; i < distances.size(); i++)
        {
                angle = angle_min + orientation + i*angle_increment;
                if(abs(distances[i]) <= range_max)
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
}

void skycam::getAttitude(const tf2_msgs::TFMessage &msg)
{       
        int model = 0;        
        if(msg.transforms[model].child_frame_id == "AutoNOMOS_mini::kinect::link")
        { 
                float imuX = msg.transforms[model].transform.rotation.x;
                float imuY = msg.transforms[model].transform.rotation.y;
                float imuZ = msg.transforms[model].transform.rotation.z;
                float imuW = msg.transforms[model].transform.rotation.w;
                
                tf::Quaternion q1(imuX,imuY,imuZ,imuW);
                tf::Matrix3x3 m(q1);
                
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                angle = float(yaw);
                //ROS_INFO_STREAM("Yaw: " << angle);
        }        
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
        
vector<Point> skycam::RRT(Mat analyzedMap, Mat analyzedCar, Point goal, int beam, float scale, float angle)
{
        float currentAngle = angle; //+ 1.5708;                
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
        Mat testMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);
        Mat dilated;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 20*2*scale*resizeScale,20*2*scale*resizeScale));
        dilate(analyzedMap, dilated, element);
        
        Point posInit = currentPosition;
        float angInit = currentAngle;
        
        float minCurve = 72; // closest turn radius possible for the car (i.e. at maximum steerling angle); given in centimeters
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
        vector<tf::Vector3> xyzBest;
        while(whileFlag == true)
        {       
                counter = 0;
                minExpectedCost = sqrt(pow(analyzedMap.cols, 2) + pow(analyzedMap.rows, 2));
                counter2 = 0;
                while(counter < beam)
                {                        
                        dis = rand() % int(minDis) + minDis;
                        //ROS_INFO_STREAM("Distance: " << dis); 
                        cuadrante = rand() % 3;   
                        //ROS_INFO_STREAM("Cuadrante: " << cuadrante);   
                        vector<tf::Vector3> xyz;
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
                                if(cv::norm(goal-candidatePosition) < minExpectedCost)
                                {
                                        float direction, newX, newY; 
                                        direction = atan2(Y,X);
                                        newX = xyz.back()[0] + 20*scale*resizeScale*cos(direction) + planeOrigin.x;
                                        newY = planeOrigin.y - (xyz.back()[1] + 20*scale*resizeScale*sin(direction));
                                        uchar pixel = dilated.at<uchar>(newY, newX);
                                        if(pixel == 0)
                                        {
                                                minExpectedCost = cv::norm(goal-candidatePosition);
                                                xyzBest.clear();
                                                for(int index = 0; index < xyz.size(); index++)
                                                {                                                                                                
                                                        xyzBest.push_back(xyz[index]);                                                
                                                }
                                                XBest = X;
                                                YBest = Y;
                                        }
                                }
                                xyz.clear();
                        }
                        else
                        {
                                counter2++;                                
                        }
                        if(counter == beam || whileFlag == false)
                        {                                                                                
                                for(int index = 0; index < xyzBest.size(); index++)
                                {
                                        Point newPoint = Point(int(xyzBest[index][0] + planeOrigin.x), int(planeOrigin.y - xyzBest[index][1]));
                                        pathVector.push_back(newPoint);
                                        pathMap.at<uchar>(newPoint) = 255;
                                }
                                xyz.clear();                                
                                //whileFlag = false;  
                                currentAngle = atan2(YBest,XBest) - 1.5708; 
                                currentPosition = Point(pathVector.back().x - planeOrigin.x, planeOrigin.y - pathVector.back().y);
                        
                                //ROS_INFO_STREAM("Final angle: " << currentAngle);
                                //ROS_INFO_STREAM("Final positon in cartesian plane: " << currentPosition);
                                imshow("Sky eye", sky_image);                                        
                                imshow("Planned path", testMap + dilated + goalMap);
                                //ROS_INFO_STREAM("Goal resized: " << goal);
                                //imshow("Car", analyzedCar);
                                cv::waitKey(1);
                        }                        
                        if(counter2 - counter > beam)
                        {
                                //ROS_INFO_STREAM("CheckPont");
                                xyzBest.clear();
                                xyz.clear();                                
                                pathVector.clear();
                                pathMap = Mat::zeros(analyzedMap.rows, analyzedMap.cols, CV_8UC1);
                                currentPosition = posInit;
                                currentAngle = angInit;
                                counter = beam;                                
                        }                        
                }
        }                
                        
        //imshow("Result", pathMap + analyzedMap + goalMap);
        return pathVector;
}
