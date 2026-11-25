"""打开双通道综合 RViz，并把 Nav2 面板的生命周期服务重映射到地图与协调器管理器。时间源由本文件参数决定。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('nav2_regulated_modules'),
                          'rviz', 'unified.rviz')
    remaps = []
    for old, new in [('lifecycle_manager_navigation', 'lifecycle_manager_navigator'),
                     ('lifecycle_manager_localization', 'lifecycle_manager_map')]:
        for service in ['is_active', 'manage_nodes']:
            remaps.append(('/' + old + '/' + service, '/' + new + '/' + service))
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', config], parameters=[{'use_sim_time': False}],
             remappings=remaps)
    ])
