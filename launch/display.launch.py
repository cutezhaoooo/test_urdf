import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='test_urdf').find('test_urdf')
    default_model_path = os.path.join(pkg_share, 'src/description/four_macnum.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/showfour.rviz')

    # print(f"Default model path: {default_model_path}")  # 打印路径以进行验证
    # print(f"Default RViz config path: {default_rviz_config_path}")  # 打印 RViz 配置路径以进行验证

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        # arguments=[{'robot_description':Command([
        #     'xacro',
        #     default_model_path,
        #     'laser_visual:=false',
        #     ' camera_visual:=false',
        #     ' imu_visual:=false'
        # ])}],
        # # parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # 在gazebo中启动launch文件
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # spawn_entity = launch_ros.actions.Node(
    #     # 要运行的gazebo_ros包
    #     package='gazebo_ros',
    #     # 在gazebo中生成一个实体
    #     executable='spawn_entity.py',
    #     # 将机器人的名称设为four_macnum  从robot_description中得到urdf描述
    #     arguments=['-entity', 'four_macnum', '-topic', 'robot_description'],
    #     output='screen'
    # )

    # world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    # world_path=os.path.join(pkg_share, 'world/selfbuild.sdf')
    # world_path=os.path.join(pkg_share, 'world/slam_simple.world')
    # world_path=os.path.join(pkg_share, 'world/Untitled/model.sdf')
    
    # world_path=os.path.join(pkg_share, 'world/AAAtest.sdf')

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True'+,
                                            # description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        # launch.actions.ExecuteProcess(        cmd=['gazebo','--verbose',world_files,'-s','libgazebo_ros_factory.so']),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', world_path,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # spawn_entity,
        rviz_node,
    ])
