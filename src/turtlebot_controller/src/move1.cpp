#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

#define TURTLE_NAME "turtle_YOURNAME"
#define TOLERANCE 0.1

ros::Publisher velocity_publisher;
turtlesim::Pose current_pose;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    current_pose = *msg;
}

void kill_default_turtle(ros::NodeHandle &nh) {
    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";

    if (kill_client.call(kill_srv)) {
        ROS_INFO("Killed turtle1");
    } else {
        ROS_ERROR("Failed to kill turtle1.");
    }
}

void spawn_new_turtle(ros::NodeHandle &nh) {
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.5;
    spawn_srv.request.y = 5.5;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = TURTLE_NAME;

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned %s at (5.5, 5.5)", TURTLE_NAME);
    } else {
        ROS_ERROR("Failed to spawn turtle.");
    }
}

void teleport_turtle(ros::NodeHandle &nh, double x, double y, double theta) {
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>(
        std::string("/") + TURTLE_NAME + "/teleport_absolute");

    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = x;
    teleport_srv.request.y = y;
    teleport_srv.request.theta = theta;

    if (teleport_client.call(teleport_srv)) {
        ROS_INFO("Teleported %s to (%.2f, %.2f)", TURTLE_NAME, x, y);
    } else {
        ROS_ERROR("Failed to teleport turtle.");
    }
}

void move_turtle(double linear, double angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher.publish(msg);
}

bool has_reached_corner(double target_x, double target_y) {
    return (fabs(current_pose.x - target_x) < TOLERANCE && fabs(current_pose.y - target_y) < TOLERANCE);
}

void move_turtle_square() {
    double corners[4][2] = {
        {1.0, 1.0}, {15.0, 1.0}, {15.0, 15.0}, {1.0, 15.0}
    };

    ROS_INFO("Moving in a square...");
    
    for (int i = 0; i < 4; i++) {
        move_turtle(2.0, 0.0); // Move forward
        while (!has_reached_corner(corners[i][0], corners[i][1])) {
            ros::spinOnce();
        }
        move_turtle(0.0, 1.57); // Turn 90 degrees
        ros::Duration(1.0).sleep();
    }
}

void move_turtle_triangle() {
    double corners[3][2] = {
        {1.0, 1.0}, {9.0, 1.0}, {5.0, 8.0}
    };

    ROS_INFO("Moving in a triangular path...");

    for (int i = 0; i < 3; i++) {
        move_turtle(2.0, 0.0); // Move forward
        while (!has_reached_corner(corners[i][0], corners[i][1])) {
            ros::spinOnce();
        }
        move_turtle(0.0, 2.09); // Turn 120 degrees
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_control");
    ros::NodeHandle nh;
    
    velocity_publisher = nh.advertise<geometry_msgs::Twist>(std::string("/") + TURTLE_NAME + "/cmd_vel", 10);
    ros::Subscriber pose_subscriber = nh.subscribe(std::string("/") + TURTLE_NAME + "/pose", 10, poseCallback);

    ros::Duration(2.0).sleep(); // Wait for services to initialize

    kill_default_turtle(nh);
    ros::Duration(1.0).sleep(); // Wait for turtle1 to be removed

    spawn_new_turtle(nh);
    ros::Duration(1.0).sleep(); // Wait for new turtle to spawn

    teleport_turtle(nh, 1.0, 1.0, 0.0); // Start from bottom-left

    move_turtle_square();
    teleport_turtle(nh, 1.0, 1.0, 0.0); // Reset position
    move_turtle_triangle();

    ROS_INFO("Turtle movement completed!");
    return 0;
}
