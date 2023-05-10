#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import utm
import csv


class Waypoints(Node):
    def __init__(self):
        super().__init__('waypoint_pub')
        self.gnss_subscriber=self.create_subscription(NavSatFix,'/gnss',self.gnss_callback,10)
        self.imu_subscriber=self.create_subscription(Imu,'/imu/data',self.imu_callback,10)
        self.odom_subscriber=self.create_subscription(Odometry,'/odometry/filtered',self.odom_callback,10)

        self.waypoint_publisher=self.create_publisher(PoseStamped,'/goal_pose',10)
        timer_period=0.25
        self.timer = self.create_timer(timer_period, self.waypoint_callback) 

        self.i=0
        self.j=0
        self.waypoint_num=0
        self.temp_waypoint_num=-1

        self.goal_pose_x=0.0
        self.goal_pose_y=0.0

        self.wait_for_fix=0

        self.gnss=None
        self.utm=None

        self.temp_yaw=None
        self.initial_yaw=None
        self.yaw=None
        self.orientation=None
        
        self.utm=None
        self.initial_utm=None
        self.gps_coordinates=None
        self.gps_coordinates_string=None
        self.utm_waypoints=None
        self.temp_coordinates=None

        self.x=0.0
        self.y=0.0

        # self.initial_utm=utm.from_latlon(12.9695051, 79.1545428)
        

    def gnss_callback(self,gnss):
        self.gnss=NavSatFix()  
        self.gnss=gnss
      
        latitude=gnss.latitude
        longitude=gnss.longitude

        self.initial_utm=utm.from_latlon(latitude,longitude)

    def imu_callback(self,imu):
        orientation=imu.orientation

        w=orientation.w
        x=orientation.x
        y=orientation.y
        z=orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.initial_yaw=math.atan2(siny_cosp, cosy_cosp)
        self.i=1


    def gps_waypoints(self):
        # file = open("coordinates.csv", "r")
        # self.gps_coordinates_string = list(csv.reader(file))
        # file.close()
        # self.gps_coordinates=[[float(coordinate) for coordinate in list] for list in self.gps_coordinates_string]
        self.gps_coordinates=[[12.968842,79.155492]]
        return self.gps_coordinates


    def waypoints(self):
        self.utm_waypoints=[(utm.from_latlon(coordinates[0], coordinates[1])) for coordinates in self.gps_waypoints()]
        self.initial_x=(self.initial_utm[0]*math.cos(self.initial_yaw))-(self.initial_utm[1]*math.sin(self.initial_yaw))
        self.initial_y=(self.initial_utm[0]*math.sin(self.initial_yaw))+(self.initial_utm[1]*math.cos(self.initial_yaw))

        self.temp_coordinates=[[(utm[0]*math.cos(self.initial_yaw))-(utm[1]*math.sin(self.initial_yaw)),(utm[0]*math.sin(self.initial_yaw))+(utm[1]*math.cos(self.initial_yaw))] for utm in self.utm_waypoints]

        goal_coordinates=[[(temp_coordinate[0]-self.initial_x),(temp_coordinate[1]-self.initial_y)] for temp_coordinate in self.temp_coordinates]
        return goal_coordinates

    def odom_callback(self,odom):
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y     

    def waypoint_callback(self):

        goal_pose=PoseStamped()
        

        print(self.wait_for_fix)
        if self.wait_for_fix<10:
            self.wait_for_fix+=1    

       
        elif self.initial_utm!=None:
            waypoints=self.waypoints()

            
            if ((self.goal_pose_x-0.3)<self.x<(self.goal_pose_x+0.3)) and ((self.goal_pose_y-0.3)<self.y<(self.goal_pose_y+0.3)):
                self.waypoint_num+=1
            
            if self.waypoint_num>=len(self.gps_coordinates):
                self.waypoint_num=0

            if self.waypoint_num!=self.temp_waypoint_num:
                self.goal_pose_x=waypoints[self.waypoint_num][0]
                self.goal_pose_y=waypoints[self.waypoint_num][1]
                goal_pose.pose.position.x=self.goal_pose_x
                goal_pose.pose.position.y=self.goal_pose_y
                goal_pose.header.frame_id="base_link"
                self.waypoint_publisher.publish(goal_pose)
                self.temp_waypoint_num=self.waypoint_num

            print("yaw=",self.initial_yaw)
            print("waypoints=",waypoints)
            
            



def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher=Waypoints()
    rclpy.spin(waypoint_publisher)

    gps_odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()