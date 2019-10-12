import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

package_name = 'provoke'
package_share_directory = get_package_share_directory(package_name);
package_launch_directory = os.path.join(package_share_directory, 'launch')
package_cfg_directory = os.path.join(package_share_directory, 'cfg')

sys.path.append(package_launch_directory)
import shared_launch as sl

map_filename = os.path.join(package_cfg_directory, 'fiducial_marker_locations_office.yaml')
rviz_config_filename = os.path.join(package_cfg_directory, 'provoke.rviz')

number_of_drones = 1

tello_ros_args_1 = [{
    'drone_ip': '192.168.0.21',
    'command_port': '38065',
    'drone_port': '8889',
    'data_port': '8890',
    'video_port': '11111'
}]
tello_ros_args_2 = [{
    'drone_ip': '192.168.0.30',
    'command_port': '11002',
    'drone_port': '8889',
    'data_port': '13002',
    'video_port': '14002'
}]

tello_ros_args_list = [tello_ros_args_1, tello_ros_args_2]

vloc_args = [{
    'use_sim_time': False,  # Use /clock if available
    'publish_tfs': 1,  # Publish drone and camera /tf
    'stamp_msgs_with_current_time': 0,  # Use incoming message time, not now()
    # 'base_frame_id': 'base_link' + suffix,
    'map_init_pose_z': -0.035,
    # 'camera_frame_id': 'camera_link' + suffix,
    'base_odometry_pub_topic': 'filtered_odom',
    'sub_camera_info_best_effort_not_reliable': 1,
    'publish_image_marked': 1,
}]

vmap_args = [{
    'use_sim_time': False,  # Use /clock if available
    'publish_tfs': 1,  # Publish marker /tf
    'marker_length': 0.1778,  # Marker length
    'marker_map_load_full_filename': map_filename,  # Load a pre-built map from disk
    'make_not_use_map': 0  # Don't save a map to disk
}]

def generate_launch_description():
    action_list = sl.generate_action_list("drone", tello_ros_args_list, vloc_args, vmap_args, rviz_config_filename)
    print(action_list)
    return LaunchDescription(action_list)
