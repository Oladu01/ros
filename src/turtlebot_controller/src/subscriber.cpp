#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;


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

 
    ros::NodeHandle nh;
	
	pub=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	
   
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    
    ros::spin();

    return 0;
}

