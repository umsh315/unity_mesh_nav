from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch arguments
    world_name = LaunchConfiguration('world_name', default='real_world')
    sensor_offset_x = LaunchConfiguration('sensorOffsetX', default='0.3')
    sensor_offset_y = LaunchConfiguration('sensorOffsetY', default='0.0')
    camera_offset_z = LaunchConfiguration('cameraOffsetZ', default='0.0')
    vehicle_x = LaunchConfiguration('vehicleX', default='0.0')
    vehicle_y = LaunchConfiguration('vehicleY', default='0.0')
    check_terrain_conn = LaunchConfiguration('checkTerrainConn', default='true')

    # PS3 joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='ps3_joy',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.12,
            'autorepeat_rate': 0.0
        }]
    )

    # Point LIO launch
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('point_lio_unilidar'),
                'launch/mapping_utlidar.launch'
            ])
        ]),
        launch_arguments={'rviz': 'false'}.items()
    )

    # Local planner launch
    local_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('local_planner'),
                'launch/local_planner.launch'
            ])
        ]),
        launch_arguments={
            'sensorOffsetX': sensor_offset_x,
            'sensorOffsetY': sensor_offset_y,
            'cameraOffsetZ': camera_offset_z,
            'goalX': vehicle_x,
            'goalY': vehicle_y
        }.items()
    )

    # Terrain analysis launch
    terrain_analysis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('terrain_analysis'),
                'launch/terrain_analysis.launch'
            ])
        ])
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizGA',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vehicle_simulator'),
            'rviz/vehicle_simulator.rviz'
        ])],
        prefix='nice'
    )

    # unitree RViz node
    unitree_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizGA',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vehicle_simulator'),
            'rviz/vehicle_simulator.rviz'
        ])],
        prefix='nice'
    )

    # TF static transform publishers
    tf_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubMap',
        arguments=['0', '0', '0', '0', '0', '0', '/map', '/camera_init']
    )

    tf_vehicle_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubVehicle',
        arguments=['0', '0', '0', '0', '0', '0', '/aft_mapped', '/sensor']
    )

    return LaunchDescription([
        # joy_node,                   # ps3手柄驱动节点
        point_lio_launch,           # point_lio的slam建图节点
        # unitree_rviz_node,
        # local_planner_launch,       # 路径规划节点
        # terrain_analysis_launch,    # 地形可穿越性分析
        # rviz_node,                  # rviz节点
        # tf_map_node,                # map坐标系到camera_init坐标系静态tf变换节点
        # tf_vehicle_node             # aft_mapped坐标系到sensor坐标系静态tf变换节点
    ])
