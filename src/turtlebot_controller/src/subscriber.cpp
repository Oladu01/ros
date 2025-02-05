#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

// Callback function that gets executed when a new message is received
void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle Position -> x: [%f], y: [%f], theta: [%f]", msg->x, msg->y, msg->theta);
    geometry_msgs::Twist my_vel;
    my_vel.linear.x=1.0;
    my_vel.angular.z=1.0;
    pub.publish(my_vel);
    
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "turtle_listener");

    // Create a NodeHandle
    ros::NodeHandle nh;
	
	pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	
    // Subscribe to the /turtle1/pose topic
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    // Keep the node running
    ros::spin();

    return 0;
}

