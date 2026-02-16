#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
import time
from pymavlink import mavutil
import os
from nautilus_mission.auv_pymavlink import AuvPymavlink

def euleur_from_quat(x,y,z,w):
    roll = math.atan2(2*(x*w+y*z), 1-2*(x**2+y**2))
    pitch = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(w*y-x*z)),math.sqrt(1-2*(w*y-x*z)))
    yaw = math.atan2(2*(w*z+z*y),1-2*(y**2+z**2))
    return roll, pitch, yaw

class FakeDVL(Node):
    def __init__(self):
        super().__init__('Fake_DVL')

        os.environ["MAVLINK20"] = "1"
        os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

        self.last_pose_enu = None
        self.last_t = 0.0
        self.msg = None
        self.last_msg = None

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/gz/tf',
            self.msg_callback,
            1
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/auv/vision_pose_delta',
            10
        )

        self.timer = self.create_timer(0.1,self.timer_callback)

        self.dvl = AuvPymavlink(self)
        self.dvl.Connect("udpin:localhost:14552",False)
        self.get_logger().info('Fake DVL started')
    
    def msg_callback(self,msg):
        self.msg = msg

    def timer_callback(self):
        # ROS → OpenCV
        if self.msg is not None:
            t = self.msg.header.stamp.sec + (self.msg.header.stamp.nanosec) * 1e-9

            x_enu = self.msg.pose.position.x
            y_enu = self.msg.pose.position.y
            z_enu = self.msg.pose.position.z

            qx = self.msg.pose.orientation.x
            qy = self.msg.pose.orientation.y
            qz = self.msg.pose.orientation.z
            qw = self.msg.pose.orientation.w

            roll_enu, pitch_enu, yaw_enu = euleur_from_quat(qx,qy,qz,qw)
            # print(f"yaw_enu = {yaw_enu}")

            current_pose_enu = np.array([t, x_enu, y_enu, z_enu, roll_enu, pitch_enu, yaw_enu])
            if self.last_pose_enu is not None:
                delta_pose_enu = current_pose_enu - self.last_pose_enu
                delta_pose_frd = delta_pose_enu.copy()
                delta_pose_frd[1] = math.cos(yaw_enu) * delta_pose_enu[1] + math.sin(yaw_enu) * delta_pose_enu[2]
                delta_pose_frd[2] = math.sin(yaw_enu) * delta_pose_enu[1] - math.cos(yaw_enu) * delta_pose_enu[2]
                delta_pose_frd[3] *= -1
                if abs(delta_pose_frd[6]) > 2*math.pi*0.5:
                    delta_pose_frd[6] -= np.sign(delta_pose_frd[5])*2*math.pi
                    delta_pose_frd[6] *= -1

                pose_msg = PoseStamped()

                # Pose expects a quaternion but publishing as euleur because thats what real DVL will return
                pose_msg.header.stamp.sec = int(delta_pose_frd[0])
                pose_msg.header.stamp.nanosec = int(int(delta_pose_frd[0] * 1e9) - int(delta_pose_frd[0]) * 1e9)
                pose_msg.pose.position.x = delta_pose_frd[1]
                pose_msg.pose.position.y = delta_pose_frd[2]
                pose_msg.pose.position.z = delta_pose_frd[3]
                pose_msg.pose.orientation.x = delta_pose_frd[4]
                pose_msg.pose.orientation.y = delta_pose_frd[5]
                pose_msg.pose.orientation.z = delta_pose_frd[6]
                pose_msg.pose.orientation.w = 0.0        

                self.pose_pub.publish(pose_msg) #timestamped important???
                self.SendDVLAsGps(t, delta_pose_frd[0], delta_pose_frd[1],delta_pose_frd[2],delta_pose_frd[3]) #dt,dx,dy. z source is gps/baro

            self.last_pose_enu = current_pose_enu
            self.last_msg = self.msg

    def SendDVLAsGps(self, t, dt, dx, dy, dz, confidence=80.0):
        """
        dx, dy, dz: pose.position increments (meters) for VISION_pose.position_DELTA
        confidence: 0..100
        """

        time_usec = int(t * 1e6)
        time_delta_usec = int(dt * 1e6)

        angle_delta = [0.0, 0.0, 0.0]  # rad
        position_delta = [dx, dy, dz]  # m

   
        self.dvl.the_connection.mav.vision_position_delta_send(
            time_usec,
            time_delta_usec,
            angle_delta,
            position_delta,
            float(confidence),
        )

        self.get_logger().info(f"Sending DVL estimated pos [{dt}, {dx}, {dy}, {dz}] to VISION_pose.position_DELTA")
        

def main(args=None):
    rclpy.init(args=args)
    node = FakeDVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()