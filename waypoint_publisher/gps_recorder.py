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


class Gps_Record(Node):
    def __init__(self):
        super().__init__('gps_record')
        self.gnss_subscriber=self.create_subscription(NavSatFix,'/gnss',self.gnss_callback,10)
        

    def gnss_callback(self,gnss):
        self.gnss=NavSatFix()  
        self.gnss=gnss
      
        self.latitude=gnss.latitude
        self.longitude=gnss.longitude
        data=(self.latitude,self.longitude)

        with open('coordinates.csv', 'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(data)




def main(args=None):
    rclpy.init(args=args)

    gps_recorder=Gps_Record()
    rclpy.spin(gps_recorder)

    gps_recorder.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()