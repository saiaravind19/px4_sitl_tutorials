#include "state_manager.h"


Position_control::Position_control(ros::NodeHandle *node)
{
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
    arm_and_offboard = node->advertiseService("arm_and_offboard", &Position_control::arm, this);


    local_pos_pub = node->advertise<nav_msgs::Odometry>("local_pose_transform/odom", 10);

    arm_cmd.request.value = false;

    offb_set_mode.request.custom_mode = "OFFBOARD";
    last_request= ros::Time::now();
}

bool Position_control::arm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    arm_cmd.request.value = true;
    return true;

}

void Position_control::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    nav_msgs::Odometry transform_odom = *odom;
    current_odom.header = transform_odom.header;
    current_odom.child_frame_id = transform_odom.child_frame_id;
    current_odom.pose.pose.position.x = transform_odom.pose.pose.position.x +local_home_transform.x;
    current_odom.pose.pose.position.y = transform_odom.pose.pose.position.y +local_home_transform.y;
    current_odom.pose.pose.position.z = transform_odom.pose.pose.position.z +local_home_transform.z;
    current_odom.twist = transform_odom.twist;
    local_pos_pub.publish(current_odom);
}


void Position_control::local_home_callback(const geometry_msgs::Point::ConstPtr& msg) {
    local_home_transform = *msg;

}

void Position_control::scheduler_callback(const ros::TimerEvent&)
{
    if (arm_cmd.request.value)
    {
        if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0))
        { 
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();

        }

        if( current_state.armed && current_state.mode != "OFFBOARD" &&
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
    }
}

void Position_control::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
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