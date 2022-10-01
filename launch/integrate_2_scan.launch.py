#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    #general parameter for the integrated laserscan
    pointCloudTopic = LaunchConfiguration('integratedTopic', default="scan")
    pointCloutFrameId = LaunchConfiguration('integratedFrameId', default="base_link")
    
    #parameter for the first laserscan, feel free to duplicate and rename for other laserscans
    scanTopic1 = LaunchConfiguration('scanTopic1', default="front_lrf/laserscan")
    laser1XOff = LaunchConfiguration('laser1XOff', default=0.0)
    laser1YOff = LaunchConfiguration('laser1YOff', default=0.0)
    laser1Alpha = LaunchConfiguration('laser1Alpha', default=0.0)
    show1 = LaunchConfiguration('show1', default=True)

    #parameter for the second laserscan, feel free to duplicate and rename for other laserscans
    scanTopic2 = LaunchConfiguration('scanTopic2', default="rear_lrf/laserscan")
    laser2XOff = LaunchConfiguration('laser2XOff', default=0.0)
    laser2YOff = LaunchConfiguration('laser2YOff', default=0.0)
    laser2Alpha = LaunchConfiguration('laser2Alpha', default=0.0)
    show2 = LaunchConfiguration('show2', default=True)

    robotFrontEnd = LaunchConfiguration('robotFrontEnd', default=0.1)
    robotRearEnd = LaunchConfiguration('robotRearEnd', default=0.1)
    robotRightEnd = LaunchConfiguration('robotRightEnd', default=0.1)
    robotLeftEnd = LaunchConfiguration('robotLeftEnd', default=0.1)

    return LaunchDescription([
        DeclareLaunchArgument(
            'integratedTopic',
            default_value=pointCloudTopic,
            description='desc',
        ),
        DeclareLaunchArgument(
            'integratedFrameId',
            default_value=pointCloutFrameId,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic1',
            default_value=scanTopic1,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1XOff',
            default_value=laser1XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1YOff',
            default_value=laser1YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser1Alpha',
            default_value=laser1Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show1',
            default_value=show1,
            description='desc',
        ),

        DeclareLaunchArgument(
            'scanTopic2',
            default_value=scanTopic2,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2XOff',
            default_value=laser2XOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2YOff',
            default_value=laser2YOff,
            description='desc',
        ),
        DeclareLaunchArgument(
            'laser2Alpha',
            default_value=laser2Alpha,
            description='desc',
        ),
        DeclareLaunchArgument(
            'show2',
            default_value=show2,
            description='desc',
        ),
        DeclareLaunchArgument(
            'robotFrontEnd',
            default_value=robotFrontEnd,
            description='desc',
        ),
        DeclareLaunchArgument(
            'robotRearEnd',
            default_value=robotRearEnd,
            description='desc',
        ),
        DeclareLaunchArgument(
            'robotRightEnd',
            default_value=robotRightEnd,
            description='desc',
        ),
        DeclareLaunchArgument(
            'robotLeftEnd',
            default_value=robotLeftEnd,
            description='desc',
        ),
        
        
        launch_ros.actions.Node(
            package='laser_scan_integrator',
            executable='laser_scan_integrator',
            parameters=[{
                'integratedTopic' : pointCloudTopic,
                'integratedFrameId' : pointCloutFrameId,
                'scanTopic1' : scanTopic1,
                'laser1XOff' : laser1XOff,
                'laser1YOff' : laser1YOff,
                'laser1Alpha' : laser1Alpha,
                'show1' : show1,
                'scanTopic2' : scanTopic2,
                'laser2XOff' : laser2XOff,
                'laser2YOff' : laser2YOff,
                'laser2Alpha' : laser2Alpha,
                'show2' : show2,
                'robotFrontEnd' : robotFrontEnd,
                'robotRearEnd' : robotRearEnd,
                'robotRightEnd' : robotRightEnd,
                'robotLeftEnd' : robotLeftEnd,
            }],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
    ])
