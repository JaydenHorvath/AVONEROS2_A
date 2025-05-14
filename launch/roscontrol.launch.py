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
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time
    }

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

    ros_gz_bridge = Node(
        package='ros_gz_bridge',                    # or 'ros_ign_bridge' if that’s what you’ve installed
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # core topics
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
             '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
             '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
           
                    
            # rgbd

              # point cloud
            '/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',

             # camera‐info
            '/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # depth image
            '/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',       
                    
            '/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/image@sensor_msgs/msg/Image[ignition.msgs.Image',


                    ],
        remappings=[
        
            #rgbd
             # image
            ('/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/image',
            '/camera/rgbd/image_raw'),


             ('/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/camera_info',
            '/camera/rgbd/camera_info'),

            # Depth point cloud
            ('/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/points',
            '/camera/rgbd/points'),

            # Depth image
            ('/world/skidpad/model/my_robot/link/base_link/sensor/rgbdcamera/depth_image',
            '/camera/rgbd/depth_image'),



        ]
    )
    # Start Ignition Gazebo via gz_sim.launch.py
    gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ])
    ),
    launch_arguments={
        'gz_args': '-r -v1 /home/jay/ros2_ws/src/articubot_one/worlds/skidpad.world'
    }.items()
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

    # Load ros2_control + controllers
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )
   

    # TF relay from controller to /tf
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
        # sim time argument
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
        # TF relay
        topic_remapping,
        # Bridge
        ros_gz_bridge,
        # Gazebo sim
        gz_sim,
        # Spawn robot
        gz_spawn_entity,
        # ros2_control node
        ros2_control_node,

        
        # Sequence controllers: joint_state -> ackermann -> brake
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
