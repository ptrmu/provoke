from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_rviz_execute_process(rviz_filename):
    return ExecuteProcess(cmd=['rviz2', '-d', rviz_filename], output='screen')


def generate_drone_component_list(ns, suffix, tello_ros_args, vloc_args):
    print('tello_ros_args', ns, suffix, tello_ros_args)
    return [
        Node(package='tello_driver', node_executable='tello_driver', output='screen',
             node_name='tello_driver', node_namespace=ns, parameters=tello_ros_args),
        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             node_name='vloc_node', node_namespace=ns, parameters=vloc_args),
    ]


def generate_action_list(ns, tello_ros_args_list, vloc_args, vmap_args, rviz_filename):

    drone_actions = []

    for i in range(len(tello_ros_args_list)):
        suffix = "" if len(ns) < 1 else "_" + str(i+1)
        ns_one = "" if len(ns) < 1 else ns + suffix
        drone_actions += generate_drone_component_list(ns_one, suffix, tello_ros_args_list[i], vloc_args)

    print('vloc_args', vloc_args)
    print('vmap_args', vmap_args)
    print('rviz_filename', rviz_filename)

    main_actions = [
        generate_rviz_execute_process(rviz_filename),
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             node_name='vmap_node', parameters=vmap_args),
    ]

    return drone_actions + main_actions
