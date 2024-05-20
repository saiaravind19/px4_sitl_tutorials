#!/usr/bin/env python

import pymap3d,copy
import rospy
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import Point,PoseStamped
from nav_msgs.msg import Odometry
from goal_allocator.srv import goal
import numpy as np
import math
import rosservice

class drone_state():
    def __init__(self):
        self.local_odom = Point()
        self.goal_point = Point()
        self.drone_home_pose = GeoPoint()
        self.global2local_transformation = Point()

class GoalAllocator():
    def __init__(self):
        rospy.init_node("goal_allocator")
        topic_list = rospy.get_published_topics()
        print("goal allocator class is initlised")

        self.namespace = []
        self.global_home_pose = GeoPoint()
        self.set_global_origin = '/mavros/global_position/set_gp_origin'
        #get the local pose topic with namespaces
        local_pose_odom_list = [topic_name[0] for topic_name in topic_list if '/mavros/local_position/odom' in topic_name[0]]
        print(local_pose_odom_list)
        
        #extract the namspaces from the list
        self.namespace =  [topic.split('/')[1] for topic in local_pose_odom_list if len(topic.split('/')) > 4]

        self.local_pose_subscriber = []
        self.drone_pose_dict = {}
        
        self.drone_home_subscriber = {}
        self.goal_point_list = []
        self.goal_allocator_service = {}


        for drone in self.namespace:
            self.drone_pose_dict[drone] = drone_state()
            self.local_pose_subscriber.append(
                rospy.Subscriber(
                    drone + '/mavros/local_position/odom',
                    Odometry,
                    lambda msg, drone=drone: self.local_odom_callback(msg, drone),
                    queue_size= 10
                )
            )
        
        for drone in self.namespace:
            self.drone_pose_dict[drone] = drone_state()
            self.drone_home_subscriber[drone]= rospy.Subscriber(
                    drone + '/mavros/home_position/home',
                    HomePosition,
                    lambda msg, drone=drone: self.home_pose_callback(msg, drone),
                    queue_size= 5
                )
        
        rate = rospy.Rate(50)
        while len(self.drone_home_subscriber) !=0:
            rate.sleep()
        
        for drone in self.namespace:

            self.goal_allocator_service[drone]= rospy.ServiceProxy(
                    '/'+drone + '/goal_update',
                    goal)
        
        print("all home positions are set")
        self.global_home_pose = self.drone_pose_dict[self.namespace[0]].drone_home_pose
        print(self.goal_allocator_service)
        self.compute_localcoordinate()


    #convert global gps coordinate to local coordinate considering one of the drone as global home position
    def gps_to_enu(self,current_cooridinate :GeoPoint , relative_cooridinate :GeoPoint):

        x,y,z = pymap3d.geodetic2enu(current_cooridinate.latitude, current_cooridinate.longitude, relative_cooridinate.altitude, relative_cooridinate.latitude, relative_cooridinate.longitude, current_cooridinate.altitude,)
        enu_frame = Point()
        enu_frame.x= round(x,3)
        enu_frame.y= round(y,3)
        enu_frame.z= round(z,3)
        return enu_frame
    

    def compute_localcoordinate(self):
        for drone in self.drone_pose_dict:
            self.drone_pose_dict[drone].global2local_transformation = self.gps_to_enu(self.global_home_pose,self.drone_pose_dict[drone].drone_home_pose)
    
    def home_pose_callback(self, msg :HomePosition, drone_id :str):
        #get the home position and unregister the subscriber callback
        self.drone_pose_dict[drone_id].drone_home_pose = msg.geo
        self.drone_home_subscriber[drone_id].unregister()
        self.drone_home_subscriber.pop(drone_id)

    # Populate the drone position w.r.t id
    def local_odom_callback(self,msg :Odometry,drone_id :str):
        self.drone_pose_dict[drone_id].local_odom = msg.pose.pose.position

    # scalling from image to 50x50 area 
    def map_goal_point(self,shape : list ,goal_points : list):
        max_bound = [50,50]
        min_bound = [3,0]
        self.goal_point_list = [] # empty the goal point when execuit is pressed
        for point in goal_points:
            scalled_point = Point()
            scalled_point.x = min_bound[0]+(point[1]/shape[1])*max_bound[0]
            scalled_point.y = min_bound[1]+(point[0]/shape[0])*max_bound[1]
            scalled_point.z = 0 
            self.goal_point_list.append(self.transform_to_yz(scalled_point))
        print("goal points",self.goal_point_list)
    # Y-Z frame transformation 
    def transform_to_yz(self,point : Point):
        xy_point = np.array([point.x,point.y,point.z])
        rotation_matrix = np.array([[0,0,1],[0,1,0],[1,0,0]])
        rotated_point = np.dot(rotation_matrix,xy_point)
        yz_point = Point()
        yz_point.x = rotated_point[0]
        yz_point.y = rotated_point[1]
        yz_point.z = rotated_point[2]
        return yz_point

    def get_nearest_drone(self,goal :Point, drone_list : list):
        drone_id = ''
        shortest_dist = 1000000 # giving some high value
        for drone in drone_list:
            current_pose = copy.deepcopy(self.drone_pose_dict[drone].local_odom)
            dist = math.sqrt(pow(goal.x - current_pose.x,2)+pow(goal.y - current_pose.y,2)+pow(goal.z - current_pose.z,2))
            if dist <= shortest_dist:
                shortest_dist = dist
                drone_id = drone
        return drone_id

    def assign_nearest_goal(self):
        
        drone_list = copy.deepcopy(self.namespace)
        for goal in self.goal_point_list:
            drone = self.get_nearest_drone(goal,drone_list)
            print(drone_list)
            if drone in drone_list :
                drone_list.remove(drone)

            self.drone_pose_dict[drone].goal_point.x = goal.x+self.drone_pose_dict[drone].global2local_transformation.x
            self.drone_pose_dict[drone].goal_point.y = goal.y+self.drone_pose_dict[drone].global2local_transformation.y
            self.drone_pose_dict[drone].goal_point.z = goal.z+self.drone_pose_dict[drone].global2local_transformation.z
            print("Drone ID:",drone,"goal assigned:",goal)
    
    def send_goal(self):
        
        service_list = rosservice.get_service_list()
        for drone in self.namespace:
            # Check if the service exists before calling it
            service_name = '/'+ drone + '/goal_update'
            # Wait for the service to become available (optional)
            if service_name in service_list:
                try:
                    response = self.goal_allocator_service[drone](self.drone_pose_dict[drone].goal_point)
                    rospy.loginfo("Service call successful for drone %s. Response: %s", drone, response)
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed for drone %s: %s", drone, str(e))

    
    def execuite_formation(self):

        self.assign_nearest_goal()
        rospy.loginfo("goals are assigned successfully")
        self.send_goal()


