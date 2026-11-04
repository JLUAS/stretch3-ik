#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CameraScan(Node):
    def __init__(self):
        super().__init__('camera_scan')
        self.enabled = self.declare_parameter('enabled', False).value
        self.pan_center = self.declare_parameter('pan_center', -2.18666628).value
        self.pan_amplitude = self.declare_parameter('pan_amplitude', 0.35).value
        self.tilt = self.declare_parameter('tilt', 0.645725305).value
        self.period = self.declare_parameter('period', 4.0).value
        if self.period <= 0.0 or self.pan_amplitude < 0.0:
            raise ValueError('camera scan period and amplitude must be non-negative')
        self.publisher = self.create_publisher(
            JointTrajectory, '/camera_scan/set_joint_trajectory', 1)
        self.direction = 1.0
        self.timer = self.create_timer(
            self.period / 2.0 if self.enabled else 1.0, self.publish_target)
        self.create_timer(0.1, self.publish_initial)
        self.initial_published = False

    def publish_initial(self):
        if self.initial_published:
            return
        self.initial_published = True
        self.publish(self.pan_center)

    def publish_target(self):
        if self.enabled:
            self.publish(self.pan_center + self.direction * self.pan_amplitude)
            self.direction *= -1.0
        else:
            self.publish(self.pan_center)

    def publish(self, pan):
        message = JointTrajectory()
        message.header.frame_id = 'world'
        message.joint_names = [
            'gazebo_camera_pan_joint', 'gazebo_camera_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [pan, self.tilt]
        message.points = [point]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
