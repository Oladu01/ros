#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "spawn_turtle_client");
    ros::NodeHandle nh;

    // Create a client that will call the /spawn service
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    // Define the service request
    turtlesim::Spawn srv;
    srv.request.x = 1.0;
    srv.request.y = 5.0;
    srv.request.theta = 0.0;
    srv.request.name = "Turtle_Leonardo";  // Change "YOURNAME" to your preferred turtle name

    // Call the service
    if (client.call(srv)) {
        ROS_INFO("Spawned a new turtle: %s", srv.response.name.c_str());
    } else {
        ROS_ERROR("Failed to call /spawn service.");
        return 1;
    }

    return 0;
}

