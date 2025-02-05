#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char **argv) {
  
    ros::init(argc, argv, "spawn_turtle_client");
    ros::NodeHandle nh;


    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

   
    turtlesim::Spawn srv;
    srv.request.x = 1.0;
    srv.request.y = 5.0;
    srv.request.theta = 0.0;
    srv.request.name = "Turtle_Leonardo"; 
    
  
    if (client.call(srv)) {
        ROS_INFO("Spawned a new turtle: %s", srv.response.name.c_str());
    } else {
        ROS_ERROR("Failed to call /spawn service.");
        return 1;
    }

    return 0;
}

