#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void moveForward(ros::Publisher &pub, double speed, double time) {
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = speed;  
    move_msg.angular.z = 0;

    ros::Rate rate(50); 
    double start_time = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - start_time < time) {
        pub.publish(move_msg);
        rate.sleep();
    }


    move_msg.linear.x = 0;
    pub.publish(move_msg);
}


void turnLeft(ros::Publisher &pub, double angular_speed, double angle_in_degrees) {
    geometry_msgs::Twist rotate_msg;
    rotate_msg.linear.x = 0;
    rotate_msg.angular.z = angular_speed;  

    double angle_in_radians = angle_in_degrees * (3.14159265359 / 180.0);
    ros::Rate rate(50); 
    double start_time = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - start_time < angle_in_radians / angular_speed) {
        pub.publish(rotate_msg);
        rate.sleep();
    }

    rotate_msg.angular.z = 0;
    pub.publish(rotate_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_square");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/Leonardo/cmd_vel", 10);

    ros::Duration(2.0).sleep(); 

    ROS_INFO("Moving in a square...");

    
    for (int i = 0; i < 4; i++) {
        moveForward(pub, 2.0, 4.7);  
        turnLeft(pub, 1.0, 90.0);    
    }
    
     for (int i = 0; i < 3; i++) {
        moveForward(pub, 2.0, 4.5);  
        turnLeft(pub, 1.0, 120.0);   
    }

    ROS_INFO("Finished moving in a square.");
    return 0;
}

