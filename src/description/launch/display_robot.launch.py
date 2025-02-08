from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Declare arguments
  paused = DeclareLaunchArgument('paused', default_value='true')
  gui = DeclareLaunchArgument('gui', default_value='true')
  headless = DeclareLaunchArgument('headless', default_value='false')
  debug = DeclareLaunchArgument('debug', default_value='false')

  # Include Gazebo launch file
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'empty_world.launch.py'
      ])
    ]),
    launch_arguments={
      'world_name': PathJoinSubstitution([
        FindPackageShare('simple_meca_car'),
        'empty_world.world'
      ]),
      'debug': LaunchConfiguration('debug'),
      'gui': LaunchConfiguration('gui'),
    }.items()
  )

  # Spawn map model
  spawn_map = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-file',
      PathJoinSubstitution([FindPackageShare('simple_meca_car'), 'urdf', 'rmus_map_2.urdf']),
      '-entity',
      'simple_meca_car'
    ],
    output='screen'
  )

  # Load URDF model
  model_path = PathJoinSubstitution([
    FindPackageShare('simple_meca_car'),
    'urdf',
    'waking_robot.xacro'
  ])

  robot_description = Command(['xacro ', model_path])

  # Spawn robot
  spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    output='screen',
    arguments=[
      '-entity', 'robot',
      '-topic', 'robot_description',
      '-x', '7.5',
      '-y', '6.0',
      '-z', '0.25',
      '-R', '0',
      '-P', '0',
      '-Y', '1.57'
    ]
  )

  # Start joint state publisher
  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher'
  )

  # Start robot state publisher
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
  )

  return LaunchDescription([
    paused,
    gui,
    headless,
    debug,
    gazebo,
    spawn_map,
    spawn_robot,
    joint_state_publisher,
    robot_state_publisher,
  ])
