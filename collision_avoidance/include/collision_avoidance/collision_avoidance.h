#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <iostream>
#include <ros/ros.h>
#include <map>
#include <string>
#include <nav_msgs/Odometry.h>
#include "goal_allocator/goal.h"
#include <geometry_msgs/Point.h>
#include <RVO3D/RVO.h>
#include <ros/master.h>
#include <sstream> 
#include <vector> 
#include <geometry_msgs/TwistStamped.h>

namespace collision_avoider{



    struct plane_data{
        RVO::Vector3 position; // vector with x,y,z
        RVO::Vector3 linear_vel; //// vector with vx,vy,vz
        unsigned int agent_id;
    };

    class pose_collector{
        public:

            pose_collector(); // init all the subscribers and service server and publihsers if any
            plane_data get_drone_odom(std::string drone_id);  // drone position and velocities linear is angular needed?
            RVO::Vector3 get_current_drone_goal();
            std::vector<std::string> getall_drone_ids();
            
        private:
            void local_odom_callback(const nav_msgs::Odometry& odom, const std::string& drone_id);
            bool update_goal_service_callback(goal_allocator::goal::Request  &req,goal_allocator::goal::Response &res);
            void update_subscribers(const ros::TimerEvent&);

            ros::ServiceServer update_goal_server; 
            RVO::Vector3 current_local_goal_pose;
            ros::Timer namespace_timer;
            std::string drone_namespace;
            std::map <std::string,ros::Subscriber> local_odom_sub_list;
            std::map <std::string, collision_avoider::plane_data> all_plane_data; // store plane id and value ad position and linear velocities
            std::vector<std::string> namespace_list;

    };

    class collision_avoider_algo{
        public:
            collision_avoider_algo();
            unsigned int add_agent(std::string drone_id,struct plane_data &plane_odom);
            bool update_agent();
            void setPrefVelocity();
            void compute_velocities(const ros::TimerEvent&); //timer callback
        
        private:
            void pub_computed_vel();
            const float   neighbourDistance = 10.0;
            const int     maxNeighbour = 10;
            const float   timeHorizon = 5.0;
            const float   timeHorizonObstacles = 5.0;
            const float   radius = 2.0;
            const float   maxSpeed = 5.0;
            
            RVO::RVOSimulator *simulator;
            std::map<std::string,int> agent2planeid_mapping;
            ros::Timer compute_rvo;
            std::string drone_namespace;
            unsigned int current_agent_id;
            ros::Publisher preferred_vel;
            ros::Publisher computed_vel; 
            
            

    };
   // Declare shared pointers for collision_avoider_node and pose_collector
    extern std::shared_ptr<ros::NodeHandle> collision_avoider_node;
    extern std::shared_ptr<collision_avoider::pose_collector> collector;
    extern std::shared_ptr<collision_avoider::collision_avoider_algo> rvo_obj;

    void init_all();
    
}

#endif
