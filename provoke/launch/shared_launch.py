from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_rviz_execute_process(rviz_filename):
    return ExecuteProcess(cmd=['rviz2', '-d', rviz_filename], output='screen')


def generate_drone_component_list(tello_ros_args, vloc_args):

    return [
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='tello_driver', parameters=tello_ros_args),
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', parameters=vloc_args),
    ]


def generate_action_list(tello_ros_args, vloc_args, vmap_args, rviz_filename):
    print('tello_ros_args', tello_ros_args)
    print('vloc_args', vloc_args)
    print('vmap_args', vmap_args)
    print('rviz_filename', rviz_filename)
    drone_actions = generate_drone_component_list(tello_ros_args, vloc_args)

    main_actions = [
        generate_rviz_execute_process(rviz_filename),
        Node(package='fiducial_vlam', node_executable='vmap_node_main', output='screen',
             node_name='vmap_node', parameters=vmap_args),
    ]

    return drone_actions + main_actions
