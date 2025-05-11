from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arduinobot_description_dir = get_package_share_directory('arduinobot_description')
    model_args = DeclareLaunchArgument(
            name="model", 
            default_value=os.path.join(
                arduinobot_description_dir, "urdf", "arduinobot.urdf.xacro"),
            description="Absolute path to the robot URDF file."
    )

    gazebo_resource_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[str(Path(arduinobot_description_dir).parent.resolve())]
    )

    gazebo_physic_engines_path = SetEnvironmentVariable(
        name="GZ_SIM_PHYSICS_ENGINE_PATH",
        value="/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins"
    )

    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine = "--physics-engine gz-physics-bullet-featherstone-plugin"
    gz_args = f"-v 4 -r empty.sdf {physics_engine}"

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 
                         'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch"
            ), "/gz_sim.launch.py"
            ]),
        launch_arguments=[("gz_args", [gz_args])]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "arduinobot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    return LaunchDescription([
        model_args,
        gazebo_resource_path,
        gazebo_physic_engines_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity, 
        gz_ros2_bridge  
    ])
