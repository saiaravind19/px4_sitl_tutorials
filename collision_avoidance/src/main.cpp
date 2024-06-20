#include "collision_avoidance/collision_avoidance.h"

namespace collision_avoider
{
  std::shared_ptr<ros::NodeHandle> collision_avoider_node;
  std::shared_ptr<pose_collector> collector;
  std::shared_ptr<collision_avoider_algo> rvo_obj;

  void init_all()
  {
    collision_avoider_node = std::make_shared<ros::NodeHandle>();
    collector = std::make_shared<collision_avoider::pose_collector>();

    rvo_obj = std::make_shared<collision_avoider::collision_avoider_algo>();

  }
  
  pose_collector::pose_collector(){  

    // drone id and drone namespace are same and interchangale accross the code 

    update_goal_server = collision_avoider_node->advertiseService("goal_update", &pose_collector::update_goal_service_callback,this);
    drone_namespace = collision_avoider_node->getNamespace();
    drone_namespace = drone_namespace.erase(0,1);
    namespace_timer = collision_avoider_node->createTimer(ros::Duration(5), &pose_collector::update_subscribers,this);

  }
  std::vector<std::string> pose_collector::getall_drone_ids(){
    return namespace_list;
  }


  bool pose_collector::update_goal_service_callback(goal_allocator::goal::Request  &req,goal_allocator::goal::Response &res){

      res.status.data = true;
      current_local_goal_pose[0]= req.goal.x;
      current_local_goal_pose[1]= req.goal.y;
      current_local_goal_pose[2]= req.goal.z;

      ROS_INFO("Received request for global pose update");
      return true;
  }

  plane_data pose_collector::get_drone_odom(std::string drone_id)
  {
    
    if (std::count(namespace_list.begin(),namespace_list.end(),drone_id) !=0){
        return all_plane_data[drone_id];
    } 
    else {
        return plane_data();
    }

  }

  RVO::Vector3 pose_collector::get_current_drone_goal(){
    return current_local_goal_pose;
  }

  void pose_collector::local_odom_callback(const nav_msgs::Odometry& odom, const std::string& drone_id)
  {
    RVO::Vector3 pose = {odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z};
    RVO::Vector3 velocity = {odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z};
    all_plane_data[drone_id].position = pose;
    all_plane_data[drone_id].linear_vel = velocity;


  }

  void pose_collector::update_subscribers(const ros::TimerEvent&){
        std::vector<ros::master::TopicInfo> topics;
        std::string odom_topic= "/local_pose_transform/odom";
        // Get the list of topics from the ROS master
        if (ros::master::getTopics(topics))
        {
          for (const auto& topic : topics)
          {
            if (topic.name.find(odom_topic) != std::string::npos) 
            {
              
              std::stringstream topic_string(topic.name); 
              std::string substring; 
              std::vector<std::string> substring_array; 
              char delimiter = '/'; 
            
              while (getline(topic_string, substring, delimiter)) { 
                  substring_array.push_back(substring); 
              }
              std::string drone_id = substring_array.at(1);
              
              
              if (std::count(namespace_list.begin(),namespace_list.end(),drone_id) ==0 )
              {
                
                local_odom_sub_list[drone_id] = collision_avoider_node->subscribe<nav_msgs::Odometry>('/'+drone_id+"/local_pose_transform/odom", 10,[this, drone_id](const nav_msgs::Odometry::ConstPtr& odom) {
                    this->local_odom_callback(*odom, drone_id);}
                );
                
                namespace_list.push_back(drone_id);
                all_plane_data[drone_id].agent_id = rvo_obj->add_agent(drone_id,all_plane_data[drone_id]);
                ROS_INFO("Appending %s namespace to the vector list",drone_id.c_str());
                ROS_INFO("RVO agent ID for drone_id: %s is %d",drone_id.c_str(),all_plane_data[drone_id].agent_id);
              } 
  

            }
              
          }
        }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "collission_avoider");
  ROS_INFO("ros ini done");
  collision_avoider::init_all();
  ROS_INFO("objected is called ");

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}