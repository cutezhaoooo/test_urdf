import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():

    # 添加world
    package_dir=get_package_share_directory('test_urdf')
    world_files=os.path.join(package_dir,"world","AAAtest.sdf")
    gazebo_world=ExecuteProcess(
        cmd=['gazebo','--verbose',world_files,'-s','libgazebo_ros_factory.so']
    )
    gazebo_world=ExecuteProcess(cmd=['gazebo', '--verbose', world_files,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')



    # 指定机器人路径
    robot_name_in_model = "four_macnum"
    urdf_path = os.path.join(get_package_share_directory('test_urdf'), "src/description", "four_macnum.urdf")

    # Start Gazebo server
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')  
    use_simulator = LaunchConfiguration('use_simulator') 
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world_files,'verbose': 'true'}.items())

    # 启动Gazebo客户端
    headless = LaunchConfiguration('headless')
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # 启动robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        output='screen'
    )

    # 启动joint_state_publisher节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        output='screen'
    )

    # 在Gazebo中生成实体
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name_in_model,
    #                '-topic', 'robot_description'],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_simulator'))
    # )

    spawn = Node( package='ros_gz_sim', executable='create', arguments=[ '-name', 'ROBOT_NAME', '-topic', 'robot_description', ], output='screen', )

    # 创建LaunchDescription并添加所有动作
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_simulator',
            default_value='True',
            description='Whether to start the simulator'
        ),
        declare_use_simulator_cmd,
        declare_simulator_cmd,
        gazebo_server_cmd,
        gazebo_client_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # spawn_entity_cmd
        spawn
    ])

    # ld = LaunchDescription()
    # ld.add_action(declare_use_simulator_cmd)
    # ld.add_action(declare_simulator_cmd)
    # ld.add_action(gazebo_server_cmd)
    # ld.add_action(gazebo_client_cmd)
    # ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    # ld.add_action(spawn_entity_cmd)

    return ld