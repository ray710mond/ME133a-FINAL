#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------------
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    pkg_lightsaber   = get_package_share_directory('lightsaber')

    # URDF for Atlas + saber
    urdf_path = os.path.join(pkg_lightsaber, 'urdf', 'atlas_gazebo.urdf')

    # RViz config
    rviz_config = os.path.join(pkg_lightsaber, 'rviz', 'lightsaber.rviz')

    world_path = os.path.join(get_package_share_directory('lightsaber'), 'gazebo', 'lightsaber.sdf')

    # ------------------------------------------------------------------
    # 1) Gazebo (new Gazebo via ros_gz_sim)
    #    -r  = run immediately
    #    -v4 = verbose
    #    empty.sdf = built-in empty world (no GUI prompt)
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_path}'
        }.items(),
    )


    # ------------------------------------------------------------------
    # 2) Robot State Publishers (for TF / RViz)
    # ------------------------------------------------------------------
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # atlas1
    rsp1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='atlas1',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'frame_prefix': 'atlas1/'},   # TF: atlas1/pelvis, atlas1/r_saber, ...
        ],
    )

    # atlas2
    rsp2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='atlas2',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'frame_prefix': 'atlas2/'},
        ],
    )

    # ------------------------------------------------------------------
    # 3) Spawn both robots into Gazebo via ros_gz_sim::create
    # ------------------------------------------------------------------
    spawn_atlas1 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_atlas1',
        output='screen',
        arguments=[
            '-name', 'atlas1',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            # '-Y', '0.0',  # yaw if you want to set it
        ],
    )

    spawn_atlas2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_atlas2',
        output='screen',
        arguments=[
            '-name', 'atlas2',
            '-file', urdf_path,
            '-x', '1.5',
            '-y', '0.0',
            '-z', '1.0',
            # 180 deg about Z so they face each other:
            '-Y', '3.14159',
        ],
    )

    # Configure a node for the pirouette and wave demo.
    atlas1 = Node(
        name       = 'lightsaber',
        namespace = 'atlas1',
        package    = 'lightsaber',
        executable = 'atlas1',
        output     = 'screen')
    
    atlas2 = Node(
        name       = 'lightsaber',
        namespace = 'atlas2',
        package    = 'lightsaber',
        executable = 'atlas2',
        output     = 'screen')


    # ------------------------------------------------------------------
    # 4) RViz
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gazebo,
        rsp1,
        rsp2,
        spawn_atlas1,
        spawn_atlas2,
        atlas1,
        atlas2,
        rviz,
    ])
