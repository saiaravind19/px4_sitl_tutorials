
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>





geometry_msgs::TwistStamped velocity;



// Callback function for copter state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

// Callback function for keyboard input
geometry_msgs::Twist keyboard_cmd_vel;
void key_cb(const geometry_msgs::Twist::ConstPtr& msg){
	keyboard_cmd_vel = *msg;
}

geometry_msgs::TwistStamped remap_vel()
{
velocity.header.stamp = ros::Time::now();
velocity.twist.linear.x = keyboard_cmd_vel.linear.x;
velocity.twist.linear.y = keyboard_cmd_vel.linear.y;
velocity.twist.linear.z = keyboard_cmd_vel.linear.z;
velocity.twist.angular.x = 0;
velocity.twist.angular.y = 0;
velocity.twist.angular.z = keyboard_cmd_vel.angular.z;
return velocity;

}


int main(int argc, char **argv)
{

	velocity.twist.linear.x = 0;
	velocity.twist.linear.y = 0;
	velocity.twist.linear.z = -1;
	ros::init( argc, argv, "keyboard_node");
	ros::NodeHandle nh;


	// subscriber
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber keyboard_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, key_cb);


	// publisher
	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
 

	// client
	ros::ServiceClient frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// the setpoint publishing rate must be faster that 2Hz
	ros::Rate rate(20.0);

	// wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("FCU connected");



	// Change from FRAME_LOCAL_NED to FRAME_BODY_NED
	mavros_msgs::SetMavFrame frame_id;
	frame_id.request.mav_frame = 8;



	// send a few setpoints before starting
	for( int i = 100; ros::ok() && i > 0; --i){
		velocity.header.stamp = ros::Time::now();
		velocity_pub.publish(velocity);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();


	while(ros::ok()){
        if (current_state.mode == "OFFBOARD" && current_state.armed ) break;
		else if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if( !current_state.armed && (ros::Time::now() -last_request > ros::Duration(5.0)))
			{
				if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
		velocity_pub.publish(remap_vel()); 


		ros::spinOnce();
		rate.sleep();

	}




	while(ros::ok()){

		velocity_pub.publish(remap_vel()); 

		ros::spinOnce();
		rate.sleep();

	}
	return 0;


}