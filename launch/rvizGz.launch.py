import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定义启动参数 sdf_model，并设置默认值
    declare_sdf_model_path = DeclareLaunchArgument(
        name='sdf_model',
        default_value='/home/zz/resource/four_macnum_car-master/src/test_urdf/src/description/four_macnum.sdf'
    )

    pkg_share = FindPackageShare(package='test_urdf').find('test_urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/showfour.rviz')

    # 默认的 URDF 文件路径
    default_model_path_urdf = os.path.join(pkg_share, 'src/description/four_macnum.urdf')

    # 声明 model 参数，默认使用 URDF 文件
    declare_model_path = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path_urdf,
        description='Absolute path to robot urdf file'
    )

    # 声明 rviz 配置参数
    declare_rviz_config_file = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    # 获取启动参数的值
    sdf_model_path = LaunchConfiguration('sdf_model')
    model_path = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rvizconfig')

    # robot_state_publisher_node 使用 URDF 文件
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[model_path],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # 获取包目录
    package_dir = get_package_share_directory('test_urdf')
    world_files = os.path.join(package_dir, "world", "AAAtest.sdf")

    # 启动 Gazebo 世界
    gazebo_world = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_files, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 定义 spawn_entity 节点，使用 SDF 模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name="spawn_sdf_entity",
        arguments=[
            '-entity', 'four_macnum',
            '-file', sdf_model_path,
        ],
        output='screen'
    )

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 添加声明的启动参数
    ld.add_action(declare_sdf_model_path)
    ld.add_action(declare_model_path)
    ld.add_action(declare_rviz_config_file)

    # 添加 Gazebo 服务器
    ld.add_action(gazebo_world)

    # 添加 spawn_entity 节点
    ld.add_action(spawn_entity)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld