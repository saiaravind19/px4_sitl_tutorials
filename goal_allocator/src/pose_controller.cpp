#include "pose_controller.h"


Position_control::Position_control(ros::NodeHandle *node)
{

    local_pos_pub = node->advertise<nav_msgs::Odometry>("local_pose_transform/odom", 10);
    plane_state_sub = node->subscribe<mavros_msgs::State>
            ("mavros/state", 10, &Position_control::state_cb,this);
    odom_sub = node->subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, &Position_control::odom_callback,this);
    local_home_sub = node->subscribe<geometry_msgs::Point>
            ("local_home_transform", 10, &Position_control::local_home_callback,this);



     global_pos_pub = node->advertise<geographic_msgs::GeoPoseStamped>
            ("mavros/setpoint_position/global", 10);
    arming_client = node->serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = node->serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //update_goal = node->advertiseService("afeakr", &Position_control::update_goal_service_callback,this); 

    arm_cmd.request.value = true;

    offb_set_mode.request.custom_mode = "OFFBOARD";

    local_goal_pose.pose.position.x = 0;
    local_goal_pose.pose.position.y = 0;
    local_goal_pose.pose.position.z = 0;

    last_request= ros::Time::now();
}

void Position_control::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    nav_msgs::Odometry transform_odom = *odom;
    transform_odom.pose.pose.position.x += local_home_transform.x;
    transform_odom.pose.pose.position.y += local_home_transform.y;
    transform_odom.pose.pose.position.z += local_home_transform.z;

    local_pos_pub.publish(transform_odom);
}


void Position_control::local_home_callback(const geometry_msgs::Point::ConstPtr& msg) {
    local_home_transform = *msg;
}

void Position_control::scheduler_callback(const ros::TimerEvent&)
{

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0) )){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        } 
        else {
            if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){ 
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();

            }

        }
}

void Position_control::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


bool Position_control::update_goal_service_callback(goal_allocator::goal::Request  &req,goal_allocator::goal::Response &res){

    res.status.data = true;
    local_goal_pose.pose.position.x= req.goal.x;
    local_goal_pose.pose.position.y= req.goal.y;
    local_goal_pose.pose.position.z= req.goal.z;

    ROS_INFO("Received request for global pose update");
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_control");
    ros::NodeHandle nh;
    Position_control offboard_controller = Position_control(&nh);

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !offboard_controller.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    offboard_controller.timer = nh.createTimer(ros::Duration(1), &Position_control::scheduler_callback,&offboard_controller);


    ros::spin();
    return 0;
}