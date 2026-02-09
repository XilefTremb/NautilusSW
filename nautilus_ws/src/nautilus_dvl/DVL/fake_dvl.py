import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import math
import numpy as np
from tf2_msgs.msg import TFMessage

def euleur_from_quat(x,y,z,w):
    roll = math.atan2(2(x*w+y*z), 1-2*(x**2+y**2))
    pitch = -math.pi/2 + 2*math.atan2(math.sqrt(1+2*(w*y-x*z)),math.sqrt(1-2*(w*y-x*z)))
    yaw = math.atan2(2*(w*z+z*y),1-2*(y**2+z**2))
    return roll, pitch, yaw

class FakeDVL(Node):
    def __init__(self):
        super().__init__('Fake_DVL')

        self.last_pose_ned = None

        self.pose_sub = self.create_subscription(
            Image,
            '/gz/tf',
            self.pose_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            Image,
            '/auv/vision_pose_delta',
            10
        )

        self.get_logger().info('Fake DVL started')

    def pose_callback(self, msg):
        # ROS → OpenCV
        x_enu = msg.transform.translation.x
        y_enu = msg.transform.translation.y
        z_enu = msg.transform.translation.z

        qx = msg.transform.rotation.x
        qy = msg.transform.rotation.y
        qz = msg.transform.rotation.z
        qw = msg.transform.rotation.w

        roll_enu, pitch_enu, yaw_enu = euleur_from_quat(qx,qy,qz,qw)

        x_ned = y_enu
        y_ned = x_enu
        z_ned = -z_enu

        roll_ned = pitch_enu
        pitch_ned = roll_enu
        yaw_ned = -yaw_enu

        current_pose_ned = np.array(x_ned, y_ned, z_ned, roll_ned, pitch_ned, yaw_ned)
        if self.last_pose_ned is not None:
            delta_pose_ned = current_pose_ned - self.last_pose_ned
        self.last_pose_ned = current_pose_ned

        delta_pose_frd = delta_pose_ned
        delta_pose_frd[0] = math.cos(yaw_ned) + math.sin(yaw_ned)
        delta_pose_frd[1] = -math.sin(yaw_ned) + math.cos(yaw_ned)

        pose_msg = TFMessage()

        # TFMessage expects a quaternion but publishing as euleur because thats what real DVL will return
        pose_msg.transform.translation.x = delta_pose_frd[0]
        pose_msg.transform.translation.y = delta_pose_frd[1]
        pose_msg.transform.translation.z = delta_pose_frd[2]
        pose_msg.transform.rotation.x = delta_pose_frd[3]
        pose_msg.transform.rotation.y = delta_pose_frd[4]
        pose_msg.transform.rotation.z = delta_pose_frd[5]
        pose_msg.transform.rotation.w = 0        
        
        pose_msg.header = msg.header

        self.pose_pub(pose_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = FakeDVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()