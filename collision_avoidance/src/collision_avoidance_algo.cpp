#include "collision_avoidance/collision_avoidance.h"

namespace collision_avoider
{

    collision_avoider_algo::collision_avoider_algo()
    {
        simulator = new RVO::RVOSimulator();
        simulator->setTimeStep(0.25F);
        simulator->setAgentDefaults(neighbourDistance,maxNeighbour,timeHorizon,radius,maxSpeed);
        drone_namespace = collision_avoider_node->getNamespace();
        drone_namespace = drone_namespace.erase(0,1);

        preferred_vel   = collision_avoider_node->advertise<geometry_msgs::TwistStamped>("preferred_velocity", 10);
        computed_vel    = collision_avoider_node->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
        compute_rvo     = collision_avoider_node->createTimer(ros::Duration(0.25), &collision_avoider_algo::compute_velocities,this);

    }

    unsigned int collision_avoider_algo::add_agent(std::string drone_id,struct plane_data &plane_odom)
    { 
        unsigned int agent_id = simulator->addAgent(plane_odom.position);
        simulator->setAgentVelocity(agent_id,plane_odom.linear_vel);
        agent2planeid_mapping[drone_id] = agent_id;  
        if (drone_id == drone_namespace)
        {
            current_agent_id = agent_id;
        }
        return agent_id;
    }

    bool collision_avoider_algo::update_agent(){
        try{
            std::vector<std::string> drone_id = collector->getall_drone_ids();
            if (drone_id.size()>0)
            {               
                for (int i = 0; i < drone_id.size(); ++i) 
                {
                    struct plane_data current_drone_data = collector->get_drone_odom(drone_id[i]);
                    simulator->setAgentPosition(current_drone_data.agent_id,current_drone_data.position);
                    ROS_INFO("Drone id : %d psoition X :%f Y: %f Z: %f",current_drone_data.agent_id,current_drone_data.position[0],current_drone_data.position[1],current_drone_data.position[2]);
                    simulator->setAgentVelocity(current_drone_data.agent_id,current_drone_data.linear_vel);

                }
                
                return true;
            }
            return false;
        }
        catch (...) {
            std::cerr << "Exception caught: " << std::endl;
        }
    }

    void collision_avoider_algo::setPrefVelocity()
    {   
        try{

            ROS_WARN("namespace :%s agent id : %ld ",drone_namespace.c_str(),current_agent_id);
            
            struct plane_data current_drone_data = collector->get_drone_odom(drone_namespace);
            RVO::Vector3 current_goal   = collector->get_current_drone_goal(); 
            RVO::Vector3 goalVector     = collector->get_current_drone_goal() - current_drone_data.position;
            ROS_INFO("current goal : x:%f y:%f z:%f",current_goal[0],current_goal[1],current_goal[2]);

            //if (RVO::absSq(goalVector) > 1.0F) {
            //  goalVector = RVO::normalize(goalVector);
            //}
            simulator->setAgentPrefVelocity(current_agent_id, goalVector);

            geometry_msgs::TwistStamped pref_vel;
            pref_vel.header.stamp = ros::Time::now();
            pref_vel.twist.linear.x = goalVector[0];
            pref_vel.twist.linear.y = goalVector[1];
            pref_vel.twist.linear.z = goalVector[2];   
            preferred_vel.publish(pref_vel);         
        }
       catch (...) {
            std::cerr << "Exception caught: " << std::endl;
        }
    }
    void collision_avoider_algo::pub_computed_vel(){

        RVO::Vector3 velocityVector = simulator->getAgentVelocity(current_agent_id);        
        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.twist.linear.x = velocityVector[0];
        cmd_vel.twist.linear.y = velocityVector[1];
        cmd_vel.twist.linear.z = velocityVector[2];

        computed_vel.publish(cmd_vel);


    }

    void collision_avoider_algo::compute_velocities(const ros::TimerEvent&)
    {
        if (collision_avoider_algo::update_agent())
        { 
            collision_avoider_algo::setPrefVelocity();
            simulator->doStep();
            collision_avoider_algo::pub_computed_vel();
        }
        else {
            ROS_WARN("drone lenght is 0 so skipping");
        }
        

    } 


}