#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker marker;

float pickup_x = 8.01;
float pickup_y = -0.78;
float pickup_w = 1.0;

float drop_x = -2.55;
float drop_y = 2.52;
float drop_w = 1.0;

visualization_msgs::Marker set_marker_position(float, float, float);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Set our initial shape type to be a cube
//uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
      
      //Publish marker at pickup position
      marker = set_marker_position(pickup_x, pickup_y, pickup_w);
      ROS_INFO("Marker added at pickup position");
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
    
      //Hide the marker
      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Hide the Marker");
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
      
      marker = set_marker_position(drop_x, drop_y, drop_w);
      ROS_INFO("Marker added at drop-off position");
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


    r.sleep();
    
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
