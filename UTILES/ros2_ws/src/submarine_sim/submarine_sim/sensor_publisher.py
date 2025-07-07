#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import Imu
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Publishers
        self.temp_pub = self.create_publisher(Float32, 'temperature', 10)
        self.humidity_pub = self.create_publisher(Float32, 'humidity', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.depthSensor_pub = self.create_publisher(int, 'depthSensor', 10)
        self.motor_current_pub = self.create_publisher(Float64MultiArray, 'motor_currents', 10)

        # Timer pour publier toutes les 0.5 secondes
        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        # Température et humidité fictives et profondeur
        temp = Float32()
        temp.data = random.uniform(10.0, 30.0)
        self.temp_pub.publish(temp)

        humidity = Float32()
        humidity.data = random.uniform(40.0, 70.0)
        self.humidity_pub.publish(humidity)

        depth = random.uniform(1,300)
        self.depthSensor_pub.publish(depth)

        # Données IMU fictives (seulement l'accélération linéaire)
        imu = Imu()
        imu.linear_acceleration.x = random.uniform(-1.0, 1.0)
        imu.linear_acceleration.y = random.uniform(-1.0, 1.0)
        imu.linear_acceleration.z = random.uniform(-9.8, -8.0)
        self.imu_pub.publish(imu)

        # Courants moteurs fictifs (8 moteurs)
        currents = Float64MultiArray()
        currents.data = [random.uniform(0.2, 4.0) for _ in range(8)]
        self.motor_current_pub.publish(currents)

        # Log console (facultatif)
        #self.get_logger().info(f'Temp: {temp.data:.1f} °C | Humidity: {humidity.data:.1f} % | IMU.z: {imu.linear_acceleration.z:.2f} m/s²')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
