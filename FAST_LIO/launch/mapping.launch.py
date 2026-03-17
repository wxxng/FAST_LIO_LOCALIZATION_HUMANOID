import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')
    urdf_path = os.path.join(package_path, 'data', 'g1', 'g1_43dof.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    object_bridge = LaunchConfiguration('object_bridge')
    bundle_pose_topic = LaunchConfiguration('bundle_pose_topic')
    object_camera_frame = LaunchConfiguration('object_camera_frame')
    object_world_frame = LaunchConfiguration('object_world_frame')
    object_torso_frame = LaunchConfiguration('object_torso_frame')
    object_pelvis_frame = LaunchConfiguration('object_pelvis_frame')
    object_anchor_frame = LaunchConfiguration('object_anchor_frame')
    object_pelvis_world_frame = LaunchConfiguration('object_pelvis_world_frame')
    object_pelvis_world_z = LaunchConfiguration('object_pelvis_world_z')
    object_frame = LaunchConfiguration('object_frame')
    bundle_pose_is_rdf = LaunchConfiguration('bundle_pose_is_rdf')
    object_cube_size = LaunchConfiguration('object_cube_size')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        # 'config_file', default_value='avia.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_joint_states_topic_cmd = DeclareLaunchArgument(
        'joint_states_topic', default_value='/g1/joint_states',
        description='JointState topic used for articulated robot visualization'
    )
    declare_object_bridge_cmd = DeclareLaunchArgument(
        'object_bridge', default_value='true',
        description='Run the bundle pose object bridge node'
    )
    declare_bundle_pose_topic_cmd = DeclareLaunchArgument(
        'bundle_pose_topic', default_value='/bundle_pose',
        description='Pose topic for the tracked object in camera coordinates'
    )
    declare_object_camera_frame_cmd = DeclareLaunchArgument(
        'object_camera_frame', default_value='d435_link',
        description='TF camera frame used to interpret the bundle pose'
    )
    declare_object_world_frame_cmd = DeclareLaunchArgument(
        'object_world_frame', default_value='camera_init',
        description='World frame used for object world pose output'
    )
    declare_object_torso_frame_cmd = DeclareLaunchArgument(
        'object_torso_frame', default_value='torso_link',
        description='Robot torso frame used for object pose output'
    )
    declare_object_pelvis_frame_cmd = DeclareLaunchArgument(
        'object_pelvis_frame', default_value='pelvis',
        description='Pelvis frame used to lock the pelvis-based world origin'
    )
    declare_object_anchor_frame_cmd = DeclareLaunchArgument(
        'object_anchor_frame', default_value='pelvis_init',
        description='Anchored pelvis frame created at the first valid pelvis pose'
    )
    declare_object_pelvis_world_frame_cmd = DeclareLaunchArgument(
        'object_pelvis_world_frame', default_value='pelvis_init_world',
        description='Shifted pelvis-based world frame used for pelvis/object world pose outputs'
    )
    declare_object_pelvis_world_z_cmd = DeclareLaunchArgument(
        'object_pelvis_world_z', default_value='0.793',
        description='Z position assigned to pelvis_init in the shifted pelvis-based world frame'
    )
    declare_object_frame_cmd = DeclareLaunchArgument(
        'object_frame', default_value='tracked_object',
        description='Child frame name for the tracked object TF'
    )
    declare_bundle_pose_is_rdf_cmd = DeclareLaunchArgument(
        'bundle_pose_is_rdf', default_value='true',
        description='Interpret bundle pose coordinates as RDF and convert them into FLU'
    )
    declare_object_cube_size_cmd = DeclareLaunchArgument(
        'object_cube_size', default_value='0.08',
        description='Cube marker size in meters for the tracked object'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', urdf_path]), value_type=str),
            'use_sim_time': use_sim_time,
        }],
        remappings=[('joint_states', joint_states_topic)],
        output='screen'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': use_sim_time,
                        'visualization.joint_states_topic': joint_states_topic,
                    }],
        output='screen'
    )
    object_pose_bridge_node = Node(
        package='fast_lio',
        executable='object_pose_bridge.py',
        parameters=[{
            'bundle_pose_topic': bundle_pose_topic,
            'camera_frame': object_camera_frame,
            'camera_init_frame': object_world_frame,
            'torso_frame': object_torso_frame,
            'pelvis_frame': object_pelvis_frame,
            'pelvis_anchor_frame': object_anchor_frame,
            'pelvis_world_frame': object_pelvis_world_frame,
            'pelvis_init_world_z': object_pelvis_world_z,
            'tracked_object_frame': object_frame,
            'bundle_pose_is_rdf': bundle_pose_is_rdf,
            'cube_size_x': object_cube_size,
            'cube_size_y': object_cube_size,
            'cube_size_z': object_cube_size,
        }],
        condition=IfCondition(object_bridge),
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_joint_states_topic_cmd)
    ld.add_action(declare_object_bridge_cmd)
    ld.add_action(declare_bundle_pose_topic_cmd)
    ld.add_action(declare_object_camera_frame_cmd)
    ld.add_action(declare_object_world_frame_cmd)
    ld.add_action(declare_object_torso_frame_cmd)
    ld.add_action(declare_object_pelvis_frame_cmd)
    ld.add_action(declare_object_anchor_frame_cmd)
    ld.add_action(declare_object_pelvis_world_frame_cmd)
    ld.add_action(declare_object_pelvis_world_z_cmd)
    ld.add_action(declare_object_frame_cmd)
    ld.add_action(declare_bundle_pose_is_rdf_cmd)
    ld.add_action(declare_object_cube_size_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(fast_lio_node)
    ld.add_action(object_pose_bridge_node)
    # ld.add_action(rviz_node)

    return ld
