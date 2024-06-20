#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <goal_allocator/goal.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>


class Position_control {
public:
    Position_control(ros::NodeHandle *node);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void scheduler_callback(const ros::TimerEvent&);
    bool update_goal_service_callback(goal_allocator::goal::Request &req, goal_allocator::goal::Response &res);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
    void local_home_callback(const geometry_msgs::Point::ConstPtr& msg);
    bool arm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    ros::Timer timer;
    mavros_msgs::State current_state;
    geometry_msgs::Point local_home_transform;
    nav_msgs::Odometry current_odom; 

private:
    ros::ServiceClient set_mode_client;
    ros::ServiceServer arm_and_offboard;
    ros::Subscriber plane_state_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber local_home_sub;

    ros::Publisher local_pos_pub;
    ros::Publisher global_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient client;

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;
    ros::Time last_request;
};

#endif // POSE_CONTROLLER_H
