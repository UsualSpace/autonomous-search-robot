#!/usr/bin/env python3

"""
odom_to_tf.py

Subscribes to nav_msgs/Odometry and republishes it as a TF transform.
Designed to sit next to nav_stack.py and nav_explorer.py.

This version shuts down cleanly when nav_stack.py terminates it.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_odom_child_frame', False)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_odom_child_frame = bool(
            self.get_parameter('use_odom_child_frame').value
        )

        self.br = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.get_logger().info(
            f'Publishing TF from odom_topic={self.odom_topic}: '
            f'{self.odom_frame} -> {self.base_frame}'
        )

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp

        if self.odom_frame:
            t.header.frame_id = self.odom_frame
        else:
            t.header.frame_id = msg.header.frame_id

        if self.use_odom_child_frame and msg.child_frame_id:
            t.child_frame_id = msg.child_frame_id
        else:
            t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except RCLError as e:
        if (
            'context is not valid' not in str(e)
            and 'rcl_shutdown already called' not in str(e)
        ):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
