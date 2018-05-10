#include "using_markers.h"

using_markers::using_markers()
{        
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        key_ = n.subscribe("/cylinder/keyboard", 10, &using_markers::moveCylinder, this);
        points_ = n.subscribe("/skycam/plannedPath", 10, &using_markers::newPoint, this);
        state_ = n.subscribe("/gazebo/model_states", 10, &using_markers::getState, this);
        
        
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        unit_box_0.header.frame_id = "ground_plane::link";
        unit_cylinder_0.header.frame_id = "ground_plane::link";
        unit_cylinder_1.header.frame_id = "ground_plane::link";
        unit_box_1.header.frame_id = "ground_plane::link";
        unit_box_2.header.frame_id = "ground_plane::link";
        unit_box_3.header.frame_id = "ground_plane::link";
        unit_box_0.header.stamp = ros::Time::now();
        unit_cylinder_0.header.stamp = ros::Time::now();
        unit_cylinder_1.header.stamp = ros::Time::now();
        unit_box_1.header.stamp = ros::Time::now();
        unit_box_2.header.stamp = ros::Time::now();
        unit_box_3.header.stamp = ros::Time::now();
        // %EndTag(MARKER_INIT)%

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // %Tag(NS_ID)%
        unit_box_0.ns = "basic_shapes0";
        unit_cylinder_0.ns = "basic_shapes1";
        unit_cylinder_1.ns = "basic_shapes2";
        unit_box_1.ns = "basic_shapes3";
        unit_box_2.ns = "basic_shapes4";
        unit_box_3.ns = "basic_shapes5";
        unit_box_0.id = 0;
        unit_cylinder_0.id = 1;
        unit_cylinder_1.id = 2;
        unit_box_1.id = 3;
        unit_box_2.id = 4;
        unit_box_3.id = 5;
        // %EndTag(NS_ID)%

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        // %Tag(TYPE)%
        unit_box_0.type = visualization_msgs::Marker::CUBE;
        unit_cylinder_0.type = visualization_msgs::Marker::CYLINDER;
        unit_cylinder_1.type = visualization_msgs::Marker::CYLINDER;
        unit_box_1.type = visualization_msgs::Marker::CUBE;
        unit_box_2.type = visualization_msgs::Marker::CUBE;
        unit_box_3.type = visualization_msgs::Marker::CUBE;
        // %EndTag(TYPE)%

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        // %Tag(ACTION)%
        unit_box_0.action = visualization_msgs::Marker::ADD;
        unit_cylinder_0.action = visualization_msgs::Marker::ADD;
        unit_cylinder_1.action = visualization_msgs::Marker::ADD;
        unit_box_1.action = visualization_msgs::Marker::ADD;
        unit_box_2.action = visualization_msgs::Marker::ADD;
        unit_box_3.action = visualization_msgs::Marker::ADD;
        // %EndTag(ACTION)%

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        // %Tag(POSE)%
        unit_box_0.pose.position.x = -0.856451;
        unit_box_0.pose.position.y = -2.31594;
        unit_box_0.pose.position.z = 0.499799;
        unit_box_0.pose.orientation.x = 0.0;
        unit_box_0.pose.orientation.y = 0.0;
        unit_box_0.pose.orientation.z = 0.0;
        unit_box_0.pose.orientation.w = 1.0;

        unit_cylinder_0.pose.position.x = 1.93601;
        unit_cylinder_0.pose.position.y = -2.08341;
        unit_cylinder_0.pose.position.z = 0.499993;
        unit_cylinder_0.pose.orientation.x = 0.0;
        unit_cylinder_0.pose.orientation.y = 0.0;
        unit_cylinder_0.pose.orientation.z = 0.0;
        unit_cylinder_0.pose.orientation.w = 1.0;

        unit_cylinder_1.pose.position.x = 1.84487;
        unit_cylinder_1.pose.position.y = 2.02617;
        unit_cylinder_1.pose.position.z = 0.499998;
        unit_cylinder_1.pose.orientation.x = 0.0;
        unit_cylinder_1.pose.orientation.y = 0.0;
        unit_cylinder_1.pose.orientation.z = 0.0;
        unit_cylinder_1.pose.orientation.w = 1.0;

        unit_box_1.pose.position.x = 2.98076;
        unit_box_1.pose.position.y = -0.067186;
        unit_box_1.pose.position.z = 0.5;
        unit_box_1.pose.orientation.x = 0.0;
        unit_box_1.pose.orientation.y = 0.0;
        unit_box_1.pose.orientation.z = 0.3826834;
        unit_box_1.pose.orientation.w = 0.9238795;

        unit_box_2.pose.position.x = 0.074298;
        unit_box_2.pose.position.y = 1.97665;
        unit_box_2.pose.position.z = 0.499996;
        unit_box_2.pose.orientation.x = 0.0;
        unit_box_2.pose.orientation.y = 0.0;
        unit_box_2.pose.orientation.z = 0.0;
        unit_box_2.pose.orientation.w = 1.0;

        unit_box_3.pose.position.x = -2.08234;
        unit_box_3.pose.position.y = -0.095289;
        unit_box_3.pose.position.z = 0.5;
        unit_box_3.pose.orientation.x = 0.0;
        unit_box_3.pose.orientation.y = 0.0;
        unit_box_3.pose.orientation.z = 0.3826834;
        unit_box_3.pose.orientation.w = 0.9238795;
        // %EndTag(POSE)%

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        // %Tag(SCALE)%
        unit_box_0.scale.x = 3.07935;
        unit_box_0.scale.y = 1.0;
        unit_box_0.scale.z = 1.0;

        unit_cylinder_0.scale.x = 1.0;
        unit_cylinder_0.scale.y = 1.0;
        unit_cylinder_0.scale.z = 1.0;

        unit_cylinder_1.scale.x = 1.0;
        unit_cylinder_1.scale.y = 1.0;
        unit_cylinder_1.scale.z = 1.0;

        unit_box_1.scale.x = 1.0;
        unit_box_1.scale.y = 1.0;
        unit_box_1.scale.z = 1.0;

        unit_box_2.scale.x = 1.0;
        unit_box_2.scale.y = 2.94625;
        unit_box_2.scale.z = 1.0;

        unit_box_3.scale.x = 1.0;
        unit_box_3.scale.y = 1.0;
        unit_box_3.scale.z = 1.0;
        // %EndTag(SCALE)%

        // Set the color -- be sure to set alpha to something non-zero!
        // %Tag(COLOR)%
        unit_box_0.color.r = 1.0f;
        unit_box_0.color.g = 0.0f;
        unit_box_0.color.b = 0.0f;
        unit_box_0.color.a = 1.0;

        unit_cylinder_0.color.r = 1.0f;
        unit_cylinder_0.color.g = 0.0f;
        unit_cylinder_0.color.b = 0.0f;
        unit_cylinder_0.color.a = 1.0;

        unit_cylinder_1.color.r = 1.0f;
        unit_cylinder_1.color.g = 0.0f;
        unit_cylinder_1.color.b = 0.0f;
        unit_cylinder_1.color.a = 1.0;

        unit_box_1.color.r = 1.0f;
        unit_box_1.color.g = 0.0f;
        unit_box_1.color.b = 0.0f;
        unit_box_1.color.a = 1.0;

        unit_box_2.color.r = 1.0f;
        unit_box_2.color.g = 0.0f;
        unit_box_2.color.b = 0.0f;
        unit_box_2.color.a = 1.0;

        unit_box_3.color.r = 1.0f;
        unit_box_3.color.g = 0.0f;
        unit_box_3.color.b = 0.0f;
        unit_box_3.color.a = 1.0;
        // %EndTag(COLOR)%

        // %Tag(LIFETIME)%
        unit_box_0.lifetime = ros::Duration();
        unit_cylinder_0.lifetime = ros::Duration();
        unit_cylinder_1.lifetime = ros::Duration();
        unit_box_1.lifetime = ros::Duration();
        unit_box_2.lifetime = ros::Duration();
        unit_box_3.lifetime = ros::Duration();
        // %EndTag(LIFETIME)%
        // %EndTag(INIT)%
        
        while (marker_pub.getNumSubscribers() < 1)
        {
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
        }
        
        ROS_INFO_STREAM("Subscribed");
        sleep(1);
        marker_pub.publish(unit_box_0);
        sleep(1);
        marker_pub.publish(unit_cylinder_0);
        sleep(1);
        marker_pub.publish(unit_cylinder_1);
        sleep(1);
        marker_pub.publish(unit_box_1);
        sleep(1);
        marker_pub.publish(unit_box_2);
        sleep(1);
        marker_pub.publish(unit_box_3);
        
        points.header.frame_id = line_strip.header.frame_id = "ground_plane::link";
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "points_and_lines";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        
        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        points.color.b = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;

}

using_markers::~using_markers()
{
        
}

void using_markers::moveCylinder(const std_msgs::Int8 &key)
{
        if(key.data == 1)
        {
                unit_cylinder_0.pose.position.y = state.pose.position.y + 0.1;                
        }
        else if(key.data == 2)
        {
                unit_cylinder_0.pose.position.x = state.pose.position.x + 0.1;
        }
        else if(key.data == 3)
        {
                unit_cylinder_0.pose.position.y = state.pose.position.y - 0.1;
        }
        else if(key.data == 4)
        {
                unit_cylinder_0.pose.position.x = state.pose.position.x - 0.1;
        }
        
        marker_pub.publish(unit_cylinder_0);
}


// %EndTag(INCLUDES)%

void using_markers::newPoint(const geometry_msgs::Point &point)
{
        //ros::Rate r(30);
        if(point.z == 0.2)
        {
                //ROS_INFO_STREAM("...Polo");
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                points.points.push_back(p);
                line_strip.points.push_back(p);
        }
        else if(point.z == 0.4)
        {
                marker_pub.publish(points);
                marker_pub.publish(line_strip);
                //r.sleep();              
        }
        else
        {
                points.points.clear();
                line_strip.points.clear();
                marker_pub.publish(points);
                marker_pub.publish(line_strip);
                //r.sleep();              
        }                
}

void using_markers::getState(const gazebo_msgs::ModelStates &msg)
{
        for(int model = 0; model < msg.name.size(); model++)
        {
                if(msg.name[model] == "unit_cylinder_0")
                {
                        state.pose = msg.pose[model];
                }
        }
}
