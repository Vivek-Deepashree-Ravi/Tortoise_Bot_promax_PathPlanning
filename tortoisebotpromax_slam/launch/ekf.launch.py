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
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_description'), 'launch')
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebotpromax.xacro')
  robot_localization_file_path = os.path.join(get_package_share_directory('tortoisebotpromax_slam'), 'config/ekf.yaml') 
  map_directory = os.path.join(get_package_share_directory(
        'tortoisebotpromax_bringup'), 'maps','room2.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time') 
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  differential_drive_node = Node(
        package='tortoisebotpromax_firmware',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='differential_publisher.py',
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
  start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])
  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),


    start_robot_localization_cmd,
    rviz_launch_cmd,
    state_publisher_launch_cmd,
    robot_state_publisher_node,
    joint_state_publisher_node,
    # differential_drive_node,

  ]
)

