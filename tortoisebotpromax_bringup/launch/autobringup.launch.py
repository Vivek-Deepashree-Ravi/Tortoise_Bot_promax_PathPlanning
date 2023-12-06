import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebotpromax_description').find('tortoisebotpromax_description')
  navigation_dir = os.path.join(get_package_share_directory('tortoisebotpromax_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_gazebo'), 'launch')
  # ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  rplidar_launch_dir=os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
  cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_slam'), 'launch')
  prefix_address = get_package_share_directory('tortoisebotpromax_slam') 
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebotpromax.xacro')
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml') 
  map_file=LaunchConfiguration('map_file')
  map_directory = os.path.join(get_package_share_directory(
        'tortoisebotpromax_bringup'), 'maps','maps/map_test.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')   
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
            launch_arguments={'params_file': params_file,}.items())

  cartographer_launch_cmd=IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
          launch_arguments={'params_file': params_file,
                            'slam':exploration,
                            'use_sim_time':use_sim_time}.items())
  # ydlidar_launch_cmd=IncludeLaunchDescription(
  #       PythonLaunchDescriptionSource(
  #           os.path.join(ydlidar_launch_dir, 'x2_ydlidar_launch.py')),
  #           condition=IfCondition(PythonExpression(['not ', use_sim_time])),
  #           launch_arguments={'use_sim_time':use_sim_time}.items())

  rplidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_launch_dir, 'rplidar.launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  differential_drive_node = Node(
        package='tortoisebotpromax_firmware',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='differential_publisher',
        name ='differential_drive_publisher',
    )
  robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )

  joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )
  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map_file',default_value=map_directory,
                                          description='Map to be used'),
 Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
    Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),

    rviz_launch_cmd,
    state_publisher_launch_cmd,
    robot_state_publisher_node,
    joint_state_publisher_node,
    # ydlidar_launch_cmd,
    rplidar_launch_cmd,
    differential_drive_node,
    gazebo_launch_cmd,
    navigation_launch_cmd, 
    cartographer_launch_cmd,  

  ]
)

