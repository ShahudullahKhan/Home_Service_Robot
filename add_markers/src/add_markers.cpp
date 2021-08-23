#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

visualization_msgs::Marker marker;
// Set our initial shape type to be a cube
uint32_t shape = visualization_msgs::Marker::CUBE;

float pickup_x = 8.01;
float pickup_y = -0.78;
float pickup_w = 1.0;

float drop_x = -2.55;
float drop_y = 2.52;
float drop_w = 1.0;

bool is_pickedup = false;
bool is_dropped = false;

float robot_x_pos, robot_y_pos, robot_w_orient;

visualization_msgs::Marker set_marker_position(float, float, float);

void getRobotPose(const nav_msgs::Odometry::ConstPtr& msg);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ros::Subscriber sub = n.subscribe("odom", 1000, getRobotPose);

  while (ros::ok())
  {

	if(is_pickedup == false && is_dropped == false){
      //Publish marker at pickup position
      marker = set_marker_position(pickup_x, pickup_y, pickup_w);
      ROS_INFO("Marker added at pickup position");
}
    
    
    //if robot position is equal to the marker position then hide the marker
    else if(is_pickedup == true && is_dropped == false){
      //Hide the marker
      ROS_INFO("Robot position is equal to the marker position");
      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Hide the Marker");
    }
    
    else if(is_pickedup == true && is_dropped == true){
      //Publish marker at drop-off position
      ROS_INFO("Robot position is equal to the drop-off position");
      marker = set_marker_position(drop_x, drop_y, drop_w);
      ROS_INFO("Marker added at drop-off position");
    }
    
	// Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    
    //pause for 5 seconds
    sleep(5);
	ROS_INFO("Paused for 5 seconds");
    ros::spinOnce();

	// Publish the marker
    /*
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
	*/

    //r.sleep();
    
  }
}

visualization_msgs::Marker set_marker_position(float x, float y, float w){
  
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

	return marker;
}

void getRobotPose(const nav_msgs::Odometry::ConstPtr& msg){
	robot_x_pos = msg->pose.pose.position.x;
  	robot_y_pos = msg->pose.pose.position.y;
  	robot_w_orient = msg->pose.pose.orientation.w;
	float pickup_dist = sqrt(pow((pickup_x - robot_x_pos), 2) + pow((pickup_y - robot_y_pos), 2));
    float drop_dist = sqrt(pow((drop_x - robot_x_pos), 2) + pow((drop_y - robot_y_pos), 2));
    if(is_pickedup == false){
    	if(pickup_dist <= 1.5){
      		is_pickedup = true;
      		ROS_INFO("Pickup zone is reached");
}
}
    else if(is_dropped == false){
		if(drop_dist <= 3.0){
			is_dropped = true;
			ROS_INFO("Dropping zone is reached");
}
}

}
