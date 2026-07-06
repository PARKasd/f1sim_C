# MIT License
#
# Same terms as gym_bridge_launch.py (Copyright (c) 2020 Hongrui Zheng).
#
# Variant of gym_bridge_launch.py whose map and start pose can be overridden
# from the command line, so tools (e.g. tools/obstacle_map_maker.py) can spin
# up the simulator on a freshly generated obstacle map without editing
# config/sim.yaml:
#
#   ros2 launch f1tenth_gym_ros obstacle_sim_launch.py \
#       map_path:=/abs/path/to/map_obs map_img_ext:=.png \
#       sx:=0.0 sy:=0.0 stheta:=0.0 rviz:=true
#
# Every argument is optional; unset arguments fall back to config/sim.yaml.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _launch_setup(context):
    share_dir = get_package_share_directory('f1tenth_gym_ros')
    config = os.path.join(share_dir, 'config', 'sim.yaml')
    config_dict = yaml.safe_load(open(config, 'r'))
    params = config_dict['bridge']['ros__parameters']

    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    # map: bare names are resolved against the maps bundled with this package
    map_path = arg('map_path') or params['map_path']
    if not os.path.isabs(map_path):
        map_path = os.path.join(share_dir, 'maps', map_path)
    map_img_ext = arg('map_img_ext') or params.get('map_img_ext', '.png')

    overrides = {'map_path': map_path, 'map_img_ext': map_img_ext}
    for name, cast in (('sx', float), ('sy', float), ('stheta', float),
                       ('sx1', float), ('sy1', float), ('stheta1', float),
                       ('num_agent', int)):
        value = arg(name)
        if value != '':
            overrides[name] = cast(value)

    num_agent = overrides.get('num_agent', params['num_agent'])
    use_rviz = arg('rviz').lower() not in ('false', '0', 'no')

    actions = []

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, overrides]
    )
    actions.append(bridge_node)

    if use_rviz:
        rviz_config = os.path.join(share_dir, 'launch', 'gym_bridge.rviz')
        actions.append(ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'exec rviz2 -d "$1" --ros-args -r __node:=rviz '
                "2> >(grep -v -E 'indexed_8bit_image\\.(vert|frag)|active samplers with a different type refer to the same texture image unit' >&2)",
                'rviz2', rviz_config
            ],
            name='rviz',
            output='screen'
        ))

    actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    ))
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(share_dir, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    ))
    if num_agent > 1:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', os.path.join(share_dir, 'launch', 'opp_racecar.xacro')])}],
            remappings=[('/robot_description', 'opp_robot_description')]
        ))

    return actions


def generate_launch_description():
    declares = [
        DeclareLaunchArgument('map_path', default_value='',
                              description='Map path without extension (absolute, or bare name of a bundled map). Empty -> sim.yaml'),
        DeclareLaunchArgument('map_img_ext', default_value='',
                              description="Map image extension, e.g. '.png' or '.pgm'. Empty -> sim.yaml"),
        DeclareLaunchArgument('sx', default_value='', description='Ego start x [m]. Empty -> sim.yaml'),
        DeclareLaunchArgument('sy', default_value='', description='Ego start y [m]. Empty -> sim.yaml'),
        DeclareLaunchArgument('stheta', default_value='', description='Ego start yaw [rad]. Empty -> sim.yaml'),
        DeclareLaunchArgument('sx1', default_value='', description='Opponent start x [m]. Empty -> sim.yaml'),
        DeclareLaunchArgument('sy1', default_value='', description='Opponent start y [m]. Empty -> sim.yaml'),
        DeclareLaunchArgument('stheta1', default_value='', description='Opponent start yaw [rad]. Empty -> sim.yaml'),
        DeclareLaunchArgument('num_agent', default_value='', description='Number of agents (1 or 2). Empty -> sim.yaml'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2 (true/false)'),
    ]
    return LaunchDescription(declares + [OpaqueFunction(function=_launch_setup)])
