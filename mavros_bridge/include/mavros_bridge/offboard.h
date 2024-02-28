#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <csignal>


    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
 

 
ros::Subscriber state_sub;
ros::Publisher local_pos_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
mavros_msgs::State current_state;

