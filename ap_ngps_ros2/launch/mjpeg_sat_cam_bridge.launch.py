from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mjpeg_url',
                default_value='http://127.0.0.1:8090/video',
                description='sat_cam_emulator MJPEG URL (use --http-mjpeg-port on emulator)',
            ),
            DeclareLaunchArgument(
                'output_topic',
                default_value='/camera/image_raw',
                description='sensor_msgs/Image topic for NGPS',
            ),
            DeclareLaunchArgument('frame_id', default_value='camera'),
            DeclareLaunchArgument('capture_fps', default_value='15.0'),
            Node(
                package='ap_ngps_ros2',
                executable='mjpeg_sat_cam_bridge.py',
                name='mjpeg_sat_cam_bridge',
                output='screen',
                parameters=[
                    {
                        'mjpeg_url': ParameterValue(LaunchConfiguration('mjpeg_url'), value_type=str),
                        'output_topic': ParameterValue(LaunchConfiguration('output_topic'), value_type=str),
                        'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                        'capture_fps': ParameterValue(LaunchConfiguration('capture_fps'), value_type=float),
                    }
                ],
            ),
        ]
    )
