import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Pose

def euleur_from_quat(x,y,z,w):
    roll = math.atan2(2*(x*w+y*z), 1-2*(x**2+y**2))
    pitch = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(w*y-x*z)),math.sqrt(1-2*(w*y-x*z)))
    yaw = math.atan2(2*(w*z+z*y),1-2*(y**2+z**2))
    return roll, pitch, yaw

class FakeDVL(Node):
    def __init__(self):
        super().__init__('Fake_DVL')

        self.last_pose_enu = None
        self.msg = None

        self.pose_sub = self.create_subscription(
            Pose,
            '/gz/tf',
            self.msg_callback,
            1
        )

        self.pose_pub = self.create_publisher(
            Pose,
            '/auv/vision_pose_delta',
            10
        )

        self.timer = self.create_timer(0.1,self.timer_callback)

        self.get_logger().info('Fake DVL started')
    
    def msg_callback(self,msg):
        self.msg = msg

    def timer_callback(self):
        # ROS → OpenCV
        if self.msg is not None:
            x_enu = self.msg.position.x
            y_enu = self.msg.position.y
            z_enu = self.msg.position.z

            qx = self.msg.orientation.x
            qy = self.msg.orientation.y
            qz = self.msg.orientation.z
            qw = self.msg.orientation.w

            roll_enu, pitch_enu, yaw_enu = euleur_from_quat(qx,qy,qz,qw)
            # print(f"yaw_enu = {yaw_enu}")

            # x_ned = y_enu
            # y_ned = x_enu
            # z_ned = -z_enu

            # roll_ned = pitch_enu
            # pitch_ned = roll_enu
            # yaw_ned = -yaw_enu + math.pi/2


            current_pose_enu = np.array([x_enu, y_enu, z_enu, roll_enu, pitch_enu, yaw_enu])
            if self.last_pose_enu is not None:
                delta_pose_enu = current_pose_enu - self.last_pose_enu
                delta_pose_frd = delta_pose_enu.copy()
                delta_pose_frd[0] = math.cos(yaw_enu) * delta_pose_enu[0] + math.sin(yaw_enu) * delta_pose_enu[1]
                delta_pose_frd[1] = math.sin(yaw_enu) * delta_pose_enu[0] - math.cos(yaw_enu) * delta_pose_enu[1]
                delta_pose_frd[2] *= -1
                if abs(delta_pose_frd[5]) > 2*math.pi*0.5:
                    delta_pose_frd[5] -= np.sign(delta_pose_frd[5])*2*math.pi
                    delta_pose_frd[5] *= -1


                pose_msg = Pose()

                # Pose expects a quaternion but publishing as euleur because thats what real DVL will return
                pose_msg.position.x = delta_pose_frd[0]
                pose_msg.position.y = delta_pose_frd[1]
                pose_msg.position.z = delta_pose_frd[2]
                pose_msg.orientation.x = delta_pose_frd[3]
                pose_msg.orientation.y = delta_pose_frd[4]
                pose_msg.orientation.z = delta_pose_frd[5]
                pose_msg.orientation.w = 0.0        

                self.pose_pub.publish(pose_msg) #timestamped important???

            self.last_pose_enu = current_pose_enu
        

def main(args=None):
    rclpy.init(args=args)
    node = FakeDVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()