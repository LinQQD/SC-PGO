from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rvizscpgo = LaunchConfiguration('rvizscpgo')
    save_directory = LaunchConfiguration('save_directory')
    map_save_directory = LaunchConfiguration('map_save_directory')
    keyframe_meter_gap = LaunchConfiguration('keyframe_meter_gap')
    keyframe_deg_gap = LaunchConfiguration('keyframe_deg_gap')
    sc_dist_thres = LaunchConfiguration('sc_dist_thres')
    sc_max_radius = LaunchConfiguration('sc_max_radius')
    mapviz_filter_size = LaunchConfiguration('mapviz_filter_size')

    pgo_node = Node(
        package='sc_pgo',
        executable='alaserPGO',
        name='alaserPGO',
        output='screen',
        parameters=[
            {
                'keyframe_meter_gap': keyframe_meter_gap,
                'keyframe_deg_gap': keyframe_deg_gap,
                'sc_dist_thres': sc_dist_thres,
                'sc_max_radius': sc_max_radius,
                'mapviz_filter_size': mapviz_filter_size,
                'save_directory': save_directory,
                'map_save_directory': map_save_directory,
            },
        ],
        remappings=[
            ('/aft_mapped_to_init',              '/out_ugv'),
            ('/velodyne_cloud_registered_local', '/out/cld/body_ugv'),
            ('/cloud_for_scancontext',           '/out/cld/body_ugv'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizscpgo',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('sc_pgo'),
            'rviz_cfg',
            'sc_pgo.rviz',
        ])],
        output='screen',
        condition=IfCondition(rvizscpgo),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rvizscpgo', default_value='false'),
        DeclareLaunchArgument('keyframe_meter_gap', default_value='0.5'),
        DeclareLaunchArgument('keyframe_deg_gap', default_value='10.0'),
        DeclareLaunchArgument('sc_dist_thres', default_value='0.3'),
        DeclareLaunchArgument('sc_max_radius', default_value='40.0'),
        DeclareLaunchArgument('mapviz_filter_size', default_value='0.4'),
        DeclareLaunchArgument(
            'save_directory',
            default_value=[EnvironmentVariable('HOME'), '/linqiu/datasets/data/'],
        ),
        DeclareLaunchArgument(
            'map_save_directory',
            default_value=[EnvironmentVariable('HOME'), '/linqiu/datasets/map/'],
        ),
        pgo_node,
        rviz_node,
    ])
