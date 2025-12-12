import os

from math import pi
from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('lightsaber'), 'rviz/lightsaber.rviz')

    # Locate the URDF file.
    urdf = os.path.join(pkgdir('lightsaber'), 'urdf/atlas_v5.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher1 = Node(
        name       = 'robot_state_publisher',
        namespace  = 'atlas1',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [
            {'robot_description': robot_description},
            {'frame_prefix': 'atlas1/'},
        ],
    )

    node_robot_state_publisher2 = Node(
        name       = 'robot_state_publisher',
        namespace  = 'atlas2',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [
            {'robot_description': robot_description},
            {'frame_prefix': 'atlas2/'},
        ],
    )

    # Configure a node for RVIZ.
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the pirouette and wave demo.
    atlas1_controller = Node(
        package='lightsaber',         # <-- your package name
        executable='atlas',            # <-- your entrypoint name
        name='atlas1_controller',
        namespace='atlas1',
        output='screen',
        parameters=[
            {'robot_name':        'atlas1'},
            {'other_robot_name':  'atlas2'},
            {'pelvis_xyz': [-1.25, 0.0, 1.0]},
            {'pelvis_rpy': [0.0, 0.0, 0.0]},         # roll, pitch, yaw
        ],
    )

    atlas2_controller = Node(
        package='lightsaber',
        executable='atlas',
        name='atlas2_controller',
        namespace='atlas2',
        output='screen',
        parameters=[
            {'robot_name':        'atlas2'},
            {'other_robot_name':  'atlas1'},
            {'pelvis_xyz': [1.25, 0.0, 1.0]},
            {'pelvis_rpy': [0.0, 0.0, pi]},         # roll, pitch, yaw
        ],
    )
    
    target_spawner = Node(
        name       = 'lightsaber',
        package    = 'lightsaber',
        executable = 'target',
        output     = 'screen')


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([
        # Start the robot_state_publisher, RVIZ, and the demo.
        node_robot_state_publisher1,
        node_robot_state_publisher2,
        node_rviz,
        atlas1_controller,
        atlas2_controller,
        target_spawner,
    ])
