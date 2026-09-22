import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None

def generate_launch_description():
    ur_type = 'ur3e'

    # Khởi chạy bộ UR MoveIt Fake (Mô phỏng)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'use_fake_hardware': 'true',
            'launch_rviz': 'true'
        }.items()
    )

    # 1. Tự động parse file URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']), ' ',
        'name:=ur', ' ',
        'ur_type:=', ur_type
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 2. Tự động parse file SRDF
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'srdf', 'ur.srdf.xacro']), ' ',
        'name:=ur', ' ',
        'ur_type:=', ur_type
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # 3. Load file Kinematics (để tính toán động học nghịch Inverse Kinematics)
    kinematics_yaml = load_yaml('ur_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Node vẽ chữ truyền các thông số vào
    draw_node = Node(
        package='ur3_draw_letter',
        executable='draw_letter_node',
        name='draw_letter_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ]
    )

    return LaunchDescription([
        ur_moveit_launch,
        # Chờ 10 giây để RViz mở lên và tải cảnh xong trước khi ra lệnh vẽ
        TimerAction(period=10.0, actions=[draw_node])
    ])
