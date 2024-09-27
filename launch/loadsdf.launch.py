import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # 定义启动参数 sdf_model，并设置默认值
    declare_sdf_model_path = DeclareLaunchArgument(
        name='sdf_model',
        default_value='/home/zz/resource/four_macnum_car-master/src/test_urdf/src/description/four_macnum.sdf'
    )

    # 使用 LaunchConfiguration 获取启动参数的值
    sdf_model_path = LaunchConfiguration('sdf_model')

    package_dir=get_package_share_directory('test_urdf')
    world_files=os.path.join(package_dir,"world","AAAtest.sdf")
    
    gazebo_world=ExecuteProcess(
        cmd=['gazebo','--verbose',world_files,'-s','libgazebo_ros_factory.so']
    )
    gazebo_world=ExecuteProcess(cmd=['gazebo', '--verbose', world_files,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')


    # 定义 spawn_entity 节点
    spawn_entity = Node(
        package='gazebo_ros',  # 指定包名
        executable='spawn_entity.py',  # 指定要执行的脚本
        name="spawn_sdf_entity",  # 为节点指定一个名称
        arguments=[
            '-entity', 'four_macnum',  # 指定生成的实体名称
            '-file', sdf_model_path,  # 使用 LaunchConfiguration 获取 SDF 模型文件的路径
            # 如果需要指定初始位置和旋转角度，可以取消注释以下行
            # '-x', '1.0',  # 设置实体在 X 轴上的初始位置
            # '-y', '1.0',  # 设置实体在 Y 轴上的初始位置
            # '-z', '0.0',  # 设置实体在 Z 轴上的初始位置
            # '-R', '0.0',  # 设置实体绕 X 轴的旋转角度
            # '-P', '0.0',  # 设置实体绕 Y 轴的旋转角度
            # '-Y', '0.0'   # 设置实体绕 Z 轴的旋转角度
        ],
        output='screen'  # 将脚本的输出打印到终端屏幕上
    )

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 添加声明的启动参数
    ld.add_action(declare_sdf_model_path)
    ld.add_action(gazebo_world)

    # 添加 Gazebo 服务器和客户端的启动描述
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)

    # 添加 spawn_entity 节点
    ld.add_action(spawn_entity)

    return ld