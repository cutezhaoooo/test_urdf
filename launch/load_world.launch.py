import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir=get_package_share_directory('test_urdf')
    # 找到install的test_urdf下面
    # /home/zz/resource/four_macnum_car-master/src/install/test_urdf/share/test_urdf
    print(package_dir)
    
    # 添加world下面的路径
    world_files=os.path.join(package_dir,"world","AAAtest.sdf")
    print(world_files)


    # 添加urdf
    urdf=os.path.join(get_package_share_directory('test_urdf'),"src/description","four_macnum.urdf")
    print(urdf)
    
    xml = open(urdf, 'r').read()

    xml = xml.replace('"', '\\"')

    swpan_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'

    robotmodel=ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen')
      # Launch the robot

    # gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    spawn_entity_cmd = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=['-entity', 'four_macnum', 
                  '-topic', 'robot_description'],
      # #                 '-x', spawn_x_val,
      # #                 '-y', spawn_y_val,
      # #                 '-z', spawn_z_val,
      # #                 '-Y', spawn_yaw_val],
                      output='screen')

    # 实例化
    ld=LaunchDescription()

    gazebo_world=ExecuteProcess(
        cmd=['gazebo','--verbose',world_files,'-s','libgazebo_ros_factory.so']
    )
    gazebo_world=ExecuteProcess(cmd=['gazebo', '--verbose', world_files,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')

    

    ld.add_action(gazebo_world)
    # ld.add_action(spawn_entity_cmd)
    ld.add_action(robotmodel)
    return ld