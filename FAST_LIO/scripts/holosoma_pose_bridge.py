#!/usr/bin/env python3

import json
import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class HolosomaPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("holosoma_pose_bridge")

        self.declare_parameter("destination_ip", "127.0.0.1")
        self.declare_parameter("destination_port", 5005)

        self.destination_ip = self.get_parameter("destination_ip").get_parameter_value().string_value
        self.destination_port = self.get_parameter("destination_port").get_parameter_value().integer_value

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (self.destination_ip, self.destination_port)

        self.pose_subscriptions = [
            self.create_subscription(PoseStamped, "/object_pose_torso", self.make_pose_callback("/object_pose_torso"), 20),
            self.create_subscription(PoseStamped, "/pelvis_pose_world", self.make_pose_callback("/pelvis_pose_world"), 20),
            self.create_subscription(PoseStamped, "/object_pose_world", self.make_pose_callback("/object_pose_world"), 20),
        ]

        self.get_logger().info(
            f"Forwarding PoseStamped topics to UDP {self.destination_ip}:{self.destination_port}"
        )

    def make_pose_callback(self, topic_name: str):
        def callback(msg: PoseStamped) -> None:
            payload = {
                "topic": topic_name,
                "type": "PoseStamped",
                "pos": [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ],
                "quat_xyzw": [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ],
            }

            self.socket.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                self.destination,
            )

        return callback

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HolosomaPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
