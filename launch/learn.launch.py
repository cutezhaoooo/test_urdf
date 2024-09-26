import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 常量定义
    package_name = 'test_urdf'  # 包名
    default_model_path = 'src/description/four_macnum.urdf'  # 默认URDF模型路径
    default_rviz_config_path = 'rviz/showfour.rviz'  # 默认RViz配置文件路径
    world_file_path = 'world/selfbuild.sdf'  # 世界文件路径
    robot_name_in_model = 'four_macnum'  # 在Gazebo中使用的机器人名称

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    # 设置不同文件和文件夹的路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    model_path = os.path.join(pkg_share, default_model_path)
    rviz_config_path = os.path.join(pkg_share, default_rviz_config_path)
    world_path = os.path.join(pkg_share, world_file_path)

    # 启动配置变量
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    sdf_model = LaunchConfiguration('sdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # 声明启动参数
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Absolute path to robot urdf file'
    )

    declare_rvizconfig_path_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rviz_config_path,
        description='Absolute path to rviz config file'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    # 启动Gazebo服务器
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'gazebo_ros', 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    # 启动Gazebo客户端
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'gazebo_ros', 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    # 启动机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # 启动关节状态发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        condition=UnlessCondition(gui)
    )

    # 启动RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 在Gazebo中生成实体
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen'
    )

    # 创建并填充启动描述
    ld = LaunchDescription()

    # 添加声明的启动参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rvizconfig_path_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # 添加启动动作
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(spawn_entity_cmd)

    return ld