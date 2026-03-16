#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
RDF_TO_FLU_QUATERNION: Quaternion = (0.5, -0.5, 0.5, -0.5)


def normalize_quaternion(quat: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quat))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quat)


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    x1, y1, z1, w1 = lhs
    x2, y2, z2, w2 = rhs
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(quat: Quaternion) -> Quaternion:
    x, y, z, w = quat
    return (-x, -y, -z, w)


def rotate_vector(quat: Quaternion, vector: Vector3) -> Vector3:
    qvec = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(quat, qvec), quaternion_conjugate(quat))
    return rotated[0], rotated[1], rotated[2]


def compose_transform(parent_translation: Vector3, parent_rotation: Quaternion,
                      child_translation: Vector3, child_rotation: Quaternion) -> Tuple[Vector3, Quaternion]:
    rotated_child = rotate_vector(parent_rotation, child_translation)
    translation = (
        parent_translation[0] + rotated_child[0],
        parent_translation[1] + rotated_child[1],
        parent_translation[2] + rotated_child[2],
    )
    rotation = normalize_quaternion(quaternion_multiply(parent_rotation, child_rotation))
    return translation, rotation


def inverse_transform(translation: Vector3, rotation: Quaternion) -> Tuple[Vector3, Quaternion]:
    inverse_rotation = normalize_quaternion(quaternion_conjugate(rotation))
    inverse_translation = rotate_vector(
        inverse_rotation,
        (-translation[0], -translation[1], -translation[2]),
    )
    return inverse_translation, inverse_rotation


def transform_from_msg(transform_msg: TransformStamped) -> Tuple[Vector3, Quaternion]:
    translation = transform_msg.transform.translation
    rotation = transform_msg.transform.rotation
    return (
        (translation.x, translation.y, translation.z),
        normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w)),
    )


def pose_from_msg(pose_msg: PoseStamped) -> Tuple[Vector3, Quaternion]:
    pose = pose_msg.pose
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        normalize_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
    )


def convert_pose_rdf_to_flu(translation: Vector3, rotation: Quaternion) -> Tuple[Vector3, Quaternion]:
    return (
        rotate_vector(RDF_TO_FLU_QUATERNION, translation),
        normalize_quaternion(quaternion_multiply(RDF_TO_FLU_QUATERNION, rotation)),
    )


class ObjectPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("object_pose_bridge")

        self.declare_parameter("bundle_pose_topic", "/bundle_pose")
        self.declare_parameter("camera_frame", "d435_link")
        self.declare_parameter("camera_init_frame", "camera_init")
        self.declare_parameter("pelvis_frame", "pelvis")
        self.declare_parameter("pelvis_anchor_frame", "pelvis_init")
        self.declare_parameter("tracked_object_frame", "tracked_object")
        self.declare_parameter("use_bundle_header_frame", False)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("bundle_pose_is_rdf", True)
        self.declare_parameter("publish_camera_alias_tf", True)
        self.declare_parameter("cube_size_x", 0.08)
        self.declare_parameter("cube_size_y", 0.08)
        self.declare_parameter("cube_size_z", 0.08)
        self.declare_parameter("marker_alpha", 1.0)
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)

        self.bundle_pose_topic = self.get_parameter("bundle_pose_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.camera_init_frame = self.get_parameter("camera_init_frame").get_parameter_value().string_value
        self.pelvis_frame = self.get_parameter("pelvis_frame").get_parameter_value().string_value
        self.pelvis_anchor_frame = self.get_parameter("pelvis_anchor_frame").get_parameter_value().string_value
        self.tracked_object_frame = self.get_parameter("tracked_object_frame").get_parameter_value().string_value
        self.use_bundle_header_frame = self.get_parameter("use_bundle_header_frame").get_parameter_value().bool_value
        self.use_latest_tf = self.get_parameter("use_latest_tf").get_parameter_value().bool_value
        self.bundle_pose_is_rdf = self.get_parameter("bundle_pose_is_rdf").get_parameter_value().bool_value
        self.publish_camera_alias_tf = self.get_parameter("publish_camera_alias_tf").get_parameter_value().bool_value
        self.cube_size_x = self.get_parameter("cube_size_x").get_parameter_value().double_value
        self.cube_size_y = self.get_parameter("cube_size_y").get_parameter_value().double_value
        self.cube_size_z = self.get_parameter("cube_size_z").get_parameter_value().double_value
        self.marker_alpha = self.get_parameter("marker_alpha").get_parameter_value().double_value
        self.tf_lookup_timeout = Duration(seconds=self.get_parameter("tf_lookup_timeout_sec").get_parameter_value().double_value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.object_pose_camera_init_pub = self.create_publisher(PoseStamped, "/object_pose_camera_init", 20)
        self.object_pose_pelvis_init_pub = self.create_publisher(PoseStamped, "/object_pose_pelvis_init", 20)
        self.marker_pub = self.create_publisher(Marker, "/object_pose_marker", 20)
        self.bundle_pose_sub = self.create_subscription(PoseStamped, self.bundle_pose_topic, self.bundle_pose_cb, 20)

        self.camera_init_from_pelvis_anchor: Optional[Tuple[Vector3, Quaternion]] = None
        self.warned_frame_override = False
        self.last_lookup_warn_ns = 0

    def bundle_pose_cb(self, msg: PoseStamped) -> None:
        source_camera_frame = msg.header.frame_id if self.use_bundle_header_frame and msg.header.frame_id else self.camera_frame
        if not self.use_bundle_header_frame and msg.header.frame_id and msg.header.frame_id != self.camera_frame and not self.warned_frame_override:
            self.get_logger().info(
                f"Using configured camera frame '{self.camera_frame}' instead of bundle header frame '{msg.header.frame_id}'."
            )
            self.warned_frame_override = True

        output_stamp = self.get_clock().now().to_msg()
        lookup_time = Time() if self.use_latest_tf else Time.from_msg(msg.header.stamp)

        camera_init_from_camera_msg = self.lookup_transform(self.camera_init_frame, source_camera_frame, lookup_time)
        if camera_init_from_camera_msg is None:
            return

        camera_init_from_pelvis_msg = self.lookup_transform(self.camera_init_frame, self.pelvis_frame, lookup_time)
        if camera_init_from_pelvis_msg is None:
            return

        camera_init_from_camera = transform_from_msg(camera_init_from_camera_msg)
        camera_init_from_pelvis = transform_from_msg(camera_init_from_pelvis_msg)
        camera_from_object = pose_from_msg(msg)
        if self.bundle_pose_is_rdf:
            camera_from_object = convert_pose_rdf_to_flu(camera_from_object[0], camera_from_object[1])

        camera_init_from_object = compose_transform(
            camera_init_from_camera[0], camera_init_from_camera[1],
            camera_from_object[0], camera_from_object[1],
        )

        if self.camera_init_from_pelvis_anchor is None:
            self.camera_init_from_pelvis_anchor = camera_init_from_pelvis
            self.get_logger().info(
                f"Locked pelvis anchor frame '{self.pelvis_anchor_frame}' at first valid pelvis pose in '{self.camera_init_frame}'."
            )

        pelvis_anchor_from_camera_init = inverse_transform(
            self.camera_init_from_pelvis_anchor[0], self.camera_init_from_pelvis_anchor[1]
        )
        pelvis_anchor_from_object = compose_transform(
            pelvis_anchor_from_camera_init[0], pelvis_anchor_from_camera_init[1],
            camera_init_from_object[0], camera_init_from_object[1],
        )

        self.publish_pose(
            self.object_pose_camera_init_pub,
            self.camera_init_frame,
            output_stamp,
            camera_init_from_object,
        )
        self.publish_pose(
            self.object_pose_pelvis_init_pub,
            self.pelvis_anchor_frame,
            output_stamp,
            pelvis_anchor_from_object,
        )

        if self.publish_camera_alias_tf and msg.header.frame_id and msg.header.frame_id != source_camera_frame:
            self.publish_camera_alias_tf_transform(source_camera_frame, msg.header.frame_id, output_stamp)

        self.publish_tf(self.camera_init_frame, f"{self.tracked_object_frame}_camera_init", output_stamp, camera_init_from_object)
        self.publish_tf(self.camera_init_frame, self.pelvis_anchor_frame, output_stamp, self.camera_init_from_pelvis_anchor)
        self.publish_tf(self.pelvis_anchor_frame, self.tracked_object_frame, output_stamp, pelvis_anchor_from_object)
        self.publish_marker(output_stamp, pelvis_anchor_from_object)

    def lookup_transform(self, target_frame: str, source_frame: str, stamp: Time) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, self.tf_lookup_timeout)
        except (LookupException, ConnectivityException, ExtrapolationException):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_lookup_warn_ns > int(2e9):
                self.get_logger().warn(f"Failed to lookup TF {target_frame} <- {source_frame}")
                self.last_lookup_warn_ns = now_ns
            return None

    def publish_pose(self, publisher, frame_id: str, stamp, transform: Tuple[Vector3, Quaternion]) -> None:
        translation, rotation = transform
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = translation[0]
        msg.pose.position.y = translation[1]
        msg.pose.position.z = translation[2]
        msg.pose.orientation.x = rotation[0]
        msg.pose.orientation.y = rotation[1]
        msg.pose.orientation.z = rotation[2]
        msg.pose.orientation.w = rotation[3]
        publisher.publish(msg)

    def publish_tf(self, parent_frame: str, child_frame: str, stamp, transform: Tuple[Vector3, Quaternion]) -> None:
        translation, rotation = transform
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = translation[0]
        msg.transform.translation.y = translation[1]
        msg.transform.translation.z = translation[2]
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(msg)

    def publish_camera_alias_tf_transform(self, parent_frame: str, child_frame: str, stamp) -> None:
        rotation = RDF_TO_FLU_QUATERNION if self.bundle_pose_is_rdf else (0.0, 0.0, 0.0, 1.0)
        self.publish_tf(parent_frame, child_frame, stamp, ((0.0, 0.0, 0.0), rotation))

    def publish_marker(self, stamp, transform: Tuple[Vector3, Quaternion]) -> None:
        translation, rotation = transform
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.pelvis_anchor_frame
        marker.ns = "tracked_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = translation[0]
        marker.pose.position.y = translation[1]
        marker.pose.position.z = translation[2]
        marker.pose.orientation.x = rotation[0]
        marker.pose.orientation.y = rotation[1]
        marker.pose.orientation.z = rotation[2]
        marker.pose.orientation.w = rotation[3]
        marker.scale.x = self.cube_size_x
        marker.scale.y = self.cube_size_y
        marker.scale.z = self.cube_size_z
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = self.marker_alpha
        marker.lifetime = Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
