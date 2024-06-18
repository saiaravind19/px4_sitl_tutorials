#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>

geometry_msgs::TwistStamped velocity;
mavros_msgs::State current_state;
geometry_msgs::Twist keyboard_cmd_vel;

// Callback function for copter state
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

// Callback function for keyboard input
void keyboard_teleop_callback(const geometry_msgs::Twist::ConstPtr& msg){
	velocity.header.stamp = ros::Time::now();
	velocity.twist.linear = msg->linear;
	velocity.twist.angular = msg->angular;
	keyboard_cmd_vel = *msg;
}

int main(int argc, char **argv)
{

	ros::init( argc, argv, "keyboard_node");
	ros::NodeHandle nh;

	// subscriber
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber keyboard_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, keyboard_teleop_callback);
	
	// publisher
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

	// client
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// the setpoint publishing rate must be faster that 2Hz
	ros::Rate rate(20.0);

	// wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	keyboard_cmd_vel.linear.x= 0;
	keyboard_cmd_vel.linear.y= 0;
	keyboard_cmd_vel.linear.z= 0;

    //mavros msg for setting the mode
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

    //mavros msg for arming
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if( !current_state.armed && (ros::Time::now() -last_request > ros::Duration(5.0)) && current_state.mode == "OFFBOARD")
			{
				if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		velocity_pub.publish(velocity); 
		ros::spinOnce();
		rate.sleep();
	}
	return 0;


}