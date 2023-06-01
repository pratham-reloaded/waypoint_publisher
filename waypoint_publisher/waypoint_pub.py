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
from std_msgs.msg import Bool
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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

        self.waypoint_num=0
        self.temp_waypoint_num=-1

        self.goal_pose_x=0.0
        self.goal_pose_y=0.0

        self.wait_for_fix=0

        self.gnss=None
        self.utm=None

        self.temp_yaw=None
        self.current_yaw=None
        self.yaw=None
        self.orientation=None
        
        self.utm=None
        self.current_utm=None
        self.gps_coordinates=[[0.0,0.0]]
        self.gps_coordinates_string=None
        self.temp_coordinates=None

        self.initial_base_link_pose_x=0.0
        self.initial_base_link_pose_y=0.0

        self.goal_pose_x_odom=0.0
        self.goal_pose_y_odom=0.0

        self.x=0.0
        self.y=0.0

        # self.current_utm=utm.from_latlon(12.9695051, 79.1545428)
        

    def gnss_callback(self,gnss):
        self.gnss=NavSatFix()  
        self.gnss=gnss
      
        self.latitude=gnss.latitude
        self.longitude=gnss.longitude

        self.current_utm=utm.from_latlon(self.latitude,self.longitude)

    def imu_callback(self,imu):
        orientation=imu.orientation

        w=orientation.w
        x=orientation.x
        y=orientation.y
        z=orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.current_yaw=math.atan2(siny_cosp, cosy_cosp)

    # def goal_status_callback(self,status):
    #     self.goal_status=status

    def gps_waypoints(self):
        # file = open("coordinates.csv", "r")
        # self.gps_coordinates_string = list(csv.reader(file))
        # file.close()
        # self.gps_coordinates=[[float(coordinate) for coordinate in list] for list in self.gps_coordinates_string]
        # self.gps_coordinates=[[12.968623,79.155336],[12.968814,79.155342],[12.968935,79.155344],[12.969050,79.155341]]
        gps_coordinates=[[42.645953,-83.166758],[42.645928,-83.166876]]
        return gps_coordinates

    def utm_waypoints(self):
        utm_waypoints=[(utm.from_latlon(coordinates[0], coordinates[1])) for coordinates in self.gps_waypoints()]
        return utm_waypoints


    def waypoints(self):
        utm_waypoints=self.utm_waypoints()
        initial_x=(self.current_utm[0]*math.cos(self.current_yaw))-(self.current_utm[1]*math.sin(self.current_yaw))
        initial_y=(self.current_utm[0]*math.sin(self.current_yaw))+(self.current_utm[1]*math.cos(self.current_yaw))

        temp_coordinates=[[(utm[0]*math.cos(self.current_yaw))-(utm[1]*math.sin(self.current_yaw)),(utm[0]*math.sin(self.current_yaw))+(utm[1]*math.cos(self.current_yaw))] for utm in utm_waypoints]

        goal_coordinates=[[(temp_coordinate[0]-initial_x),(temp_coordinate[1]-initial_y)] for temp_coordinate in temp_coordinates]
        return goal_coordinates

    def odom_callback(self,odom):
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y    
 

    def waypoint_callback(self):
        goal_pose=PoseStamped()

        print(self.wait_for_fix)
        if self.wait_for_fix<10:
            self.wait_for_fix+=1  

        elif self.current_utm!=None:
            waypoints=self.waypoints()
            utm_waypoints=self.utm_waypoints()
            
            if self.waypoint_num>=len(self.gps_coordinates) and self.gps_coordinates[0][0]!=0.0:
                self.waypoint_num=0

            if self.waypoint_num!=self.temp_waypoint_num:
                self.goal_pose_x=waypoints[self.waypoint_num][0]
                self.goal_pose_y=waypoints[self.waypoint_num][1]

                goal_pose.pose.position.x=self.goal_pose_x
                goal_pose.pose.position.y=self.goal_pose_y

                print(self.goal_pose_x)
                print(self.goal_pose_y)                
                goal_pose.header.frame_id="base_link"
                self.waypoint_publisher.publish(goal_pose)
                self.temp_waypoint_num=self.waypoint_num

            if abs(self.current_utm[0]-utm_waypoints[self.waypoint_num][0])<0.7 and abs(self.current_utm[1]-utm_waypoints[self.waypoint_num][1])<0.7:
                self.waypoint_num+=1



            # print("yaw=",self.current_yaw)
            # print("waypoints=",waypoints)
            print("distance to goal=",abs(self.current_utm[0]-self.utm_waypoints[self.waypoint_num][0]),',', abs(self.current_utm[1]-self.utm_waypoints[self.waypoint_num][1]))
            print("waypoint number=",self.waypoint_num)

            



def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher=Waypoints()
    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

