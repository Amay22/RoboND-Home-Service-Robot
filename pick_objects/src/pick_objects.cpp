#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

ros::Subscriber marker_subscriber;

move_base_msgs::MoveBaseGoal createGoal(const visualization_msgs::Marker & m) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = m.pose;
    return goal;
}

bool pursueGoal(move_base_msgs::MoveBaseGoal & moveBaseGoal) {
    //tell the action client that we want to spin a thread by default
    MoveBaseClient move_base_client("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!move_base_client.waitForServer()) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_client.sendGoal(moveBaseGoal);

    move_base_client.waitForResult(ros::Duration(200));

    return move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void markerCallback(const visualization_msgs::Marker & m) {
    ROS_INFO("Received new goal location. Plotting path...");
    move_base_msgs::MoveBaseGoal goal = createGoal(m);
    bool success = pursueGoal(goal);

    if (success) {
        ROS_INFO("Goal pose reached");
        ros::Duration(5.0).sleep();
    } else {
        ROS_INFO("Failed to reach goal pose");
    }
}

int main(int argc, char ** argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Create a ROS NodeHandle object
    ros::NodeHandle nodeHandle;

    //create marker subscriber
    marker_subscriber = nodeHandle.subscribe("/visualization_marker", 1, markerCallback);

    // Enter an infinite loop where the marker_callback function will be called when new marker messages arrive
    ros::Duration time_between_ros_wakeups(0.5);
    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
}