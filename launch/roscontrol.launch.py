import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Environment variables for GPU offload
    env_offload = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1'
    )
    env_vendor = SetEnvironmentVariable(
        name='__GLX_VENDOR_LIBRARY_NAME',
        value='nvidia'
    )

    # Robot description via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('articubot_one'),
            'description', 'robot.urdf.xacro'
        ])
    ])
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    # Controllers configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('articubot_one'),
        'config', 'ackermann_drive_controller.yaml'
    ])



    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )


    # ROS 2 <-> Ignition bridge
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            
            
        ],
        output='screen'
    )

    # Start Ignition Gazebo via gz_sim.launch.py
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-r -v1 empty.sdf'}.items()
    )

    # Spawn the robot entity in Ignition
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-allow_renaming', 'true'
        ]
    )

    # Controller spawners (to be triggered by events)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
       

        output='screen'
    )
    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_steering_controller',
            '--param-file', robot_controllers
        ],
 
        output='screen'
    )

    

    

   

    topic_remapping = Node(
    package='topic_tools',
    executable='relay',
    name='tf_relay',
    arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
    output='screen'
    )

    # RViz (delayed until simulation is ready)
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        # Launch argument for sim time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation clock'
        ),
        # GPU env vars
        env_offload,
        env_vendor,
        # Robot state publisher
        rsp_node,

    
        topic_remapping,

        # Bridge
        gz_ros2_bridge,
        # Gazebo sim
        gz_sim,
        # Spawn robot
        gz_spawn_entity,
        # Event handlers to sequence controllers
             # Sequence: spawn joint_state_broadcaster → ackermann → brakes
       RegisterEventHandler(
           OnProcessExit(
               target_action=gz_spawn_entity,
               on_exit=[joint_state_broadcaster_spawner]
           )
       ),
       RegisterEventHandler(
           OnProcessExit(
               target_action=joint_state_broadcaster_spawner,
               on_exit=[ackermann_spawner]
           )
       ),
     
      

       # RViz
       rviz_node,
    ])
    