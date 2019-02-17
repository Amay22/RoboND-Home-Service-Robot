#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>
#include <iostream>

using namespace std;

//flag to detect if goal pose has been reached
bool poseAchieved = false;

geometry_msgs::Pose goalPose;

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
  const float positionErr = sqrt(pow(msg->pose.pose.position.x- goalPose.position.x, 2) + pow(msg->pose.pose.position.y - goalPose.position.y,2));
  if (positionErr < 0.1) {
    poseAchieved = true;
  } else {
    ROS_INFO("Current pose error exceeds pose tolerance of %f radians (heading) and %f meters (position)", 0.1, positionErr);
  }
}

visualization_msgs::Marker createMarker(float (&pose)[7]){
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  marker.ns = "add_marker";
  marker.id = 0;

  // Set the marker type.
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.pose.orientation.x = pose[3];
  marker.pose.orientation.y = pose[4];
  marker.pose.orientation.z = pose[5];
  marker.pose.orientation.w = pose[6];

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  return marker;
}

void publishPose(ros::NodeHandle &nodeHandle, float (&pose)[7], ros::Duration &timeout){
  poseAchieved = false;

  //covert pose array to marker message
  visualization_msgs::Marker marker = createMarker(pose);

  //set current goal pose to the marker pose
  goalPose = marker.pose;

  //create new marker publisher
  ros::Publisher marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //check if anything is subscribing to the marker publisher
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      break;
    }
    ros::Duration(1).sleep();
  }

  ROS_INFO("Creating odom subscriber.");
  ros::Subscriber bot_pose_subscriber = nodeHandle.subscribe("odom", 1, poseCallback);

  ROS_INFO("Publishing marker.");
  marker_pub.publish(marker);

  //mark start time to track time since marker published
  ros::Time start = ros::Time::now();
  ros::Time now = ros::Time::now();


  ros::Rate loop_frequency(1); //check pose at 1Hz
  while((now - start)<timeout && !poseAchieved && ros::ok()) {
    ros::spinOnce();
    now = ros::Time::now();
    //publish again in case it was missed
    marker_pub.publish(marker);
    loop_frequency.sleep();
  }
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle nodeHandle;
  ros::Duration poseTimeout = ros::Duration(200.0);

  float pickupPose[7] = {3.50, 7.0, 0.03, 0, 0, 1.0, 0.55};
  ROS_INFO("Publishing pick up pose marker.");
  publishPose(nodeHandle, pickupPose, poseTimeout);

  if (poseAchieved) {
    ROS_INFO("Reached pick up pose");
  } else {
    ROS_INFO("Failed to reach the pick up pose.");
  }
  ros::Duration(5).sleep();

  float dropoffPose[7] = {4.0, 0.0, 0.0, 0, 0, 1.0, 2.0};
  ROS_INFO("Publishing drop off pose marker.");
  publishPose(nodeHandle, dropoffPose, poseTimeout);

  if (poseAchieved) {
    ROS_INFO("Success reached drop off pose");
  } else {
    ROS_INFO("Failed to reach the drop off pose");
  }
  ros::Duration(5).sleep();

  return 0;

}
