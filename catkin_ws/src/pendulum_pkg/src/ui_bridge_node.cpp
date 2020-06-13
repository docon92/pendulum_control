#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose2D.h"
#include<math.h>


visualization_msgs::Marker marker;

void handle_position (const geometry_msgs::Pose2D::ConstPtr& new_position)
{
    float theta = new_position->theta+1.57;
    marker.pose.position.x = new_position->x;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = -sin(theta/2.0f);
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = cos(theta/2.0f);
    // ROS_INFO("received a new position!");
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(15);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber position_sub = n.subscribe<geometry_msgs::Pose2D>("pendulum/position", 5,handle_position);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::ARROW;

  //while (ros::ok())
  //{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1 && ros::ok())
    {
      if (!ros::ok())
      {
        return 0;
      }
      marker.lifetime = ros::Duration();
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
  

  while (ros::ok())
  {
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    ros::spinOnce();
    r.sleep();
  }
    

    // // Cycle between different shapes
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }

  
}
