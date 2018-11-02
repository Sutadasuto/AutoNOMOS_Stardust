#include "using_markers.h"

UsingMarkers::UsingMarkers()
{        
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    key_ = n_.subscribe("/cylinder/keyboard", 10, &UsingMarkers::MoveCylinder, this);
    points_ = n_.subscribe("/skycam/plannedPath", 10, &UsingMarkers::NewPoint, this);
    state_ = n_.subscribe("/gazebo/model_states", 10, &UsingMarkers::GetState, this);
    
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    unit_box_0_.header.frame_id = "ground_plane::link";
    unit_cylinder_0_.header.frame_id = "ground_plane::link";
    unit_cylinder_1_.header.frame_id = "ground_plane::link";
    unit_box_1_.header.frame_id = "ground_plane::link";
    unit_box_2_.header.frame_id = "ground_plane::link";
    unit_box_3_.header.frame_id = "ground_plane::link";
    unit_box_0_.header.stamp = ros::Time::now();
    unit_cylinder_0_.header.stamp = ros::Time::now();
    unit_cylinder_1_.header.stamp = ros::Time::now();
    unit_box_1_.header.stamp = ros::Time::now();
    unit_box_2_.header.stamp = ros::Time::now();
    unit_box_3_.header.stamp = ros::Time::now();
    // %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // %Tag(NS_ID)%
    unit_box_0_.ns = "basic_shapes0";
    unit_cylinder_0_.ns = "basic_shapes1";
    unit_cylinder_1_.ns = "basic_shapes2";
    unit_box_1_.ns = "basic_shapes3";
    unit_box_2_.ns = "basic_shapes4";
    unit_box_3_.ns = "basic_shapes5";
    unit_box_0_.id = 0;
    unit_cylinder_0_.id = 1;
    unit_cylinder_1_.id = 2;
    unit_box_1_.id = 3;
    unit_box_2_.id = 4;
    unit_box_3_.id = 5;
    // %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // %Tag(TYPE)%
    unit_box_0_.type = visualization_msgs::Marker::CUBE;
    unit_cylinder_0_.type = visualization_msgs::Marker::CYLINDER;
    unit_cylinder_1_.type = visualization_msgs::Marker::CYLINDER;
    unit_box_1_.type = visualization_msgs::Marker::CUBE;
    unit_box_2_.type = visualization_msgs::Marker::CUBE;
    unit_box_3_.type = visualization_msgs::Marker::CUBE;
    // %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // %Tag(ACTION)%
    unit_box_0_.action = visualization_msgs::Marker::ADD;
    unit_cylinder_0_.action = visualization_msgs::Marker::ADD;
    unit_cylinder_1_.action = visualization_msgs::Marker::ADD;
    unit_box_1_.action = visualization_msgs::Marker::ADD;
    unit_box_2_.action = visualization_msgs::Marker::ADD;
    unit_box_3_.action = visualization_msgs::Marker::ADD;
    // %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // %Tag(POSE)%
    unit_box_0_.pose.position.x = -0.856451;
    unit_box_0_.pose.position.y = -2.31594;
    unit_box_0_.pose.position.z = 0.499799;
    unit_box_0_.pose.orientation.x = 0.0;
    unit_box_0_.pose.orientation.y = 0.0;
    unit_box_0_.pose.orientation.z = 0.0;
    unit_box_0_.pose.orientation.w = 1.0;

    unit_cylinder_0_.pose.position.x = 1.93601;
    unit_cylinder_0_.pose.position.y = -2.08341;
    unit_cylinder_0_.pose.position.z = 0.499993;
    unit_cylinder_0_.pose.orientation.x = 0.0;
    unit_cylinder_0_.pose.orientation.y = 0.0;
    unit_cylinder_0_.pose.orientation.z = 0.0;
    unit_cylinder_0_.pose.orientation.w = 1.0;

    unit_cylinder_1_.pose.position.x = 1.84487;
    unit_cylinder_1_.pose.position.y = 2.02617;
    unit_cylinder_1_.pose.position.z = 0.499998;
    unit_cylinder_1_.pose.orientation.x = 0.0;
    unit_cylinder_1_.pose.orientation.y = 0.0;
    unit_cylinder_1_.pose.orientation.z = 0.0;
    unit_cylinder_1_.pose.orientation.w = 1.0;

    unit_box_1_.pose.position.x = 2.98076;
    unit_box_1_.pose.position.y = -0.067186;
    unit_box_1_.pose.position.z = 0.5;
    unit_box_1_.pose.orientation.x = 0.0;
    unit_box_1_.pose.orientation.y = 0.0;
    unit_box_1_.pose.orientation.z = 0.3826834;
    unit_box_1_.pose.orientation.w = 0.9238795;

    unit_box_2_.pose.position.x = 0.074298;
    unit_box_2_.pose.position.y = 1.97665;
    unit_box_2_.pose.position.z = 0.499996;
    unit_box_2_.pose.orientation.x = 0.0;
    unit_box_2_.pose.orientation.y = 0.0;
    unit_box_2_.pose.orientation.z = 0.0;
    unit_box_2_.pose.orientation.w = 1.0;

    unit_box_3_.pose.position.x = -2.08234;
    unit_box_3_.pose.position.y = -0.095289;
    unit_box_3_.pose.position.z = 0.5;
    unit_box_3_.pose.orientation.x = 0.0;
    unit_box_3_.pose.orientation.y = 0.0;
    unit_box_3_.pose.orientation.z = 0.3826834;
    unit_box_3_.pose.orientation.w = 0.9238795;
    // %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
    unit_box_0_.scale.x = 3.07935;
    unit_box_0_.scale.y = 1.0;
    unit_box_0_.scale.z = 1.0;

    unit_cylinder_0_.scale.x = 1.0;
    unit_cylinder_0_.scale.y = 1.0;
    unit_cylinder_0_.scale.z = 1.0;

    unit_cylinder_1_.scale.x = 1.0;
    unit_cylinder_1_.scale.y = 1.0;
    unit_cylinder_1_.scale.z = 1.0;

    unit_box_1_.scale.x = 1.0;
    unit_box_1_.scale.y = 1.0;
    unit_box_1_.scale.z = 1.0;

    unit_box_2_.scale.x = 1.0;
    unit_box_2_.scale.y = 2.94625;
    unit_box_2_.scale.z = 1.0;

    unit_box_3_.scale.x = 1.0;
    unit_box_3_.scale.y = 1.0;
    unit_box_3_.scale.z = 1.0;
    // %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
    unit_box_0_.color.r = 1.0f;
    unit_box_0_.color.g = 0.0f;
    unit_box_0_.color.b = 0.0f;
    unit_box_0_.color.a = 1.0;

    unit_cylinder_0_.color.r = 1.0f;
    unit_cylinder_0_.color.g = 0.0f;
    unit_cylinder_0_.color.b = 0.0f;
    unit_cylinder_0_.color.a = 1.0;

    unit_cylinder_1_.color.r = 1.0f;
    unit_cylinder_1_.color.g = 0.0f;
    unit_cylinder_1_.color.b = 0.0f;
    unit_cylinder_1_.color.a = 1.0;

    unit_box_1_.color.r = 1.0f;
    unit_box_1_.color.g = 0.0f;
    unit_box_1_.color.b = 0.0f;
    unit_box_1_.color.a = 1.0;

    unit_box_2_.color.r = 1.0f;
    unit_box_2_.color.g = 0.0f;
    unit_box_2_.color.b = 0.0f;
    unit_box_2_.color.a = 1.0;

    unit_box_3_.color.r = 1.0f;
    unit_box_3_.color.g = 0.0f;
    unit_box_3_.color.b = 0.0f;
    unit_box_3_.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
    unit_box_0_.lifetime = ros::Duration();
    unit_cylinder_0_.lifetime = ros::Duration();
    unit_cylinder_1_.lifetime = ros::Duration();
    unit_box_1_.lifetime = ros::Duration();
    unit_box_2_.lifetime = ros::Duration();
    unit_box_3_.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%
    // %EndTag(INIT)%
    
    while (marker_pub_.getNumSubscribers() < 1)
    {
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
    }
    
    ROS_INFO_STREAM("Subscribed");
    sleep(1);
    marker_pub_.publish(unit_box_0_);
    sleep(1);
    marker_pub_.publish(unit_cylinder_0_);
    sleep(1);
    marker_pub_.publish(unit_cylinder_1_);
    sleep(1);
    marker_pub_.publish(unit_box_1_);
    sleep(1);
    marker_pub_.publish(unit_box_2_);
    sleep(1);
    marker_pub_.publish(unit_box_3_);
    
    marker_points_.header.frame_id = line_strip_.header.frame_id = "ground_plane::link";
    marker_points_.header.stamp = line_strip_.header.stamp = ros::Time::now();
    marker_points_.ns = line_strip_.ns = "points_and_lines";
    marker_points_.action = line_strip_.action = visualization_msgs::Marker::ADD;
    marker_points_.pose.orientation.w = line_strip_.pose.orientation.w = 1.0;
    
    marker_points_.id = 0;
    line_strip_.id = 1;

    marker_points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    marker_points_.scale.x = 0.2;
    marker_points_.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip_.scale.x = 0.1;

    marker_points_.color.b = 1.0f;
    marker_points_.color.a = 1.0;

    // Line strip is blue
    line_strip_.color.g = 1.0;
    line_strip_.color.a = 1.0;
}

UsingMarkers::~UsingMarkers()
{
        
}

void UsingMarkers::MoveCylinder(const std_msgs::Int8 &key)
{
    if(key.data == 1)
    {
        unit_cylinder_0_.pose.position.y = model_state_.pose.position.y + 0.1;                
    }
    else if(key.data == 2)
    {
        unit_cylinder_0_.pose.position.x = model_state_.pose.position.x + 0.1;
    }
    else if(key.data == 3)
    {
        unit_cylinder_0_.pose.position.y = model_state_.pose.position.y - 0.1;
    }
    else if(key.data == 4)
    {
        unit_cylinder_0_.pose.position.x = model_state_.pose.position.x - 0.1;
    }
    
    marker_pub_.publish(unit_cylinder_0_);
}

// %EndTag(INCLUDES)%

void UsingMarkers::NewPoint(const geometry_msgs::Point &new_point)
{
    if(new_point.z == 0.2)
    {
        p_.x = new_point.x;
        p_.y = new_point.y;
        p_.z = new_point.z;
        marker_points_.points.push_back(p_);
        line_strip_.points.push_back(p_);
    }
    else if(new_point.z == 0.4)
    {
        marker_pub_.publish(marker_points_);
        marker_pub_.publish(line_strip_);       
    }
    else
    {
        marker_points_.points.clear();
        line_strip_.points.clear();
        marker_pub_.publish(marker_points_);
        marker_pub_.publish(line_strip_);           
    }                
}

void UsingMarkers::GetState(const gazebo_msgs::ModelStates &msg)
{
    for(int model = 0; model < msg.name.size(); model++)
    {
        if(msg.name[model] == "unit_cylinder_0")
        {
            model_state_.pose = msg.pose[model];
        }
    }
}
