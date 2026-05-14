#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID
from nautilus_mission.state import State


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        self._state = State()

        # Current target can be one ID or a list of IDs
        self.target_ids = None

        # Parsed detections: list of [id, px, angle, depth]
        self.detections = []

        self.mean_depth_forward_cam = None

        # Subscribers
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.mean_depth_sub = self.create_subscription(
            Int16,
            '/yolo/mean_depth_forward_cam',
            self.mean_depth_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(Int8, '/mission/state', 10)

        self.target_detection_pub = self.create_publisher(
            Float32MultiArray,
            '/mission/target_detection',
            10
        )

        self.forward_cmd_pub = self.create_publisher(
            Int16,
            '/control/cmd/forward',
            10
        )

        self.depth_threshold_pub = self.create_publisher(
            Int16,
            '/yolo/depth_threshold',
            10
        )

        # Timers
        self.timer_state_machine = self.create_timer(1 / 20, self.state_machine)
        self.timer_state_sender = self.create_timer(1 / 10, self.state_targets_sender)

        # Initial state
        self.state = RobotState.TRAVERSE_GATE

        self.set_target([ObjectID.GATE_LEFT_MID])

    def state_machine(self):
        if self.state == RobotState.SEARCH:
            if self.is_target_present():
                self.state = RobotState.CENTER_GATE

        elif self.state == RobotState.CENTER_GATE:
            if self.is_target_centered() and self.state.lifespan > 10.0:
                self.set_target([ObjectID.GATE_LEFT_MID])
                self.state = RobotState.APPROACH_GATE

        elif self.state == RobotState.APPROACH_GATE:
            if not self.is_target_present():
                self.publish_depth_threshold(15000)
                self.state = RobotState.TRAVERSE_GATE

        elif self.state == RobotState.TRAVERSE_GATE:
            self.publish_forward_cmd(1700)

            if self.state.lifespan > 2.0:
                self.set_target([ObjectID.GATE_LEG_L])

                if self.is_target_approached(5000):
                    self.publish_depth_threshold(15000)

                    self.set_target(ObjectID.GATE_LEG_L)

                    self.state = RobotState.CIRCLE_MARKER

        elif self.state == RobotState.CIRCLE_MARKER:
            if (
                self.state.lifespan > 10.0
                and self.is_target_present()
                and self.mean_depth_forward_cam is not None
                and self.mean_depth_forward_cam > 8200
            ):
                self.publish_depth_threshold(15000)
                self.state = RobotState.RETURN_GATE

        elif self.state == RobotState.RETURN_GATE:
            self.publish_forward_cmd(1700)

            if self.state.lifespan > 5.0:
                self.set_target([
                    ObjectID.GATE_LEG_L,
                    ObjectID.GATE_LEFT_MID,
                    ObjectID.REQUIN,
                    ObjectID.POISSON
                ])

                self.state = RobotState.APPROACH_ANY

        elif self.state == RobotState.APPROACH_ANY:
            if self.is_target_approached(2000):
                self.publish_depth_threshold(5000)
                self.state = RobotState.CENTER_GATE

    def state_targets_sender(self):
        if self.state._state is not None:
            msg = Int8()
            msg.data = self.state.value
            self.state_pub.publish(msg)

        target = self.get_target_detection()

        if target is not None:
            msg = Float32MultiArray()
            msg.data = [
                float(target[0]),  # id
                float(target[1]),  # px
                float(target[2]),  # angle
                float(target[3]),  # depth
            ]
            self.target_detection_pub.publish(msg)

    def detection_callback(self, msg):
        data = msg.data

        if len(data) % 4 != 0:
            self.get_logger().warn(
                f'Received malformed detection array of length {len(data)}. Expected multiple of 4.'
            )
            return

        self.detections = [
            data[i:i + 4]
            for i in range(0, len(data), 4)
        ]

    def mean_depth_callback(self, msg):
        self.mean_depth_forward_cam = msg.data

    def set_target(self, target_ids):
        if target_ids is None:
            self.target_ids = None
            return

        if not isinstance(target_ids, list):
            target_ids = [target_ids]

        self.target_ids = [int(target_id) for target_id in target_ids]

        self.get_logger().info(f'Set target IDs to: {self.target_ids}')

    def get_target_detection(self):
        if self.target_ids is None:
            return None

        return next(
            (
                detection for detection in self.detections
                if int(detection[0]) in self.target_ids
            ),
            None
        )

    def is_target_present(self):
        target = self.get_target_detection()

        if target is not None:
            self.get_logger().info(f'Target ID {int(target[0])} was found!')
            return True

        return False

    def is_target_centered(self):
        target = self.get_target_detection()

        if target is None:
            return False

        px = target[1]
        angle = target[2]

        # For normal objects, angle may be the useful centering error.
        # For gate-like objects, px and angle can both matter.
        if abs(angle) < 15:
            self.get_logger().info(f'Target ID {int(target[0])} is centered!')
            return True

        return False

    def is_target_approached(self, distance):
        target = self.get_target_detection()

        if target is None:
            return False

        depth = target[3]

        if depth < distance:
            self.get_logger().info(
                f'Target ID {int(target[0])} is in range! Depth: {depth}'
            )
            return True

        return False

    def publish_forward_cmd(self, value):
        msg = Int16()
        msg.data = value
        self.forward_cmd_pub.publish(msg)

    def publish_depth_threshold(self, value):
        msg = Int16()
        msg.data = value
        self.depth_threshold_pub.publish(msg)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        self._state.set(new_state)
        self.get_logger().info(f'Set state to: {new_state.name}')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()