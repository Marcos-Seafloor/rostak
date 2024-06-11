from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cot_url',
            default_value='',
            description='TAK server URL'),
        
        DeclareLaunchArgument(
            'cot_params',
            default_value='',
            description='Path to CoT config file'),
        
        DeclareLaunchArgument(
            'rate',
            default_value='0.2',
            description='Rate to publish CoT status messages'),

        SetEnvironmentVariable(name='DEBUG', value='false'),

        Node(
            package='rostak',
            executable='rostak_bridge',
            name='rostak_bridge',
            output='screen',
            parameters=[{
                'COT_URL': LaunchConfiguration('cot_url'),
                'PYTAK_TLS_CLIENT_CERT': '',
                'PYTAK_TLS_CLIENT_KEY': '',
                'PYTAK_TLS_CLIENT_CAFILE': '',
                'PYTAK_TLS_CLIENT_CIPHERS': '',
                'PYTAK_TLS_DONT_VERIFY': '',
                'PYTAK_TLS_DONT_CHECK_HOSTNAME': ''
            }]
        ),

        DeclareLaunchArgument(
            'fix_topic',  
            default_value='/mavros/global_position/global',
            description='Topic for NavSatFix messages'),

        Node(
            package='rostak',
            executable='roscot_fix',
            name='roscot_fix',
            output='screen', 
            parameters=[{
                'cot_params': LaunchConfiguration('cot_params'),
                'rate': LaunchConfiguration('rate')  
            }],
            remappings=[('fix', 'fix_throttle')]
        ),

        Node(
            package='topic_tools',
            executable='throttle',
            name='fix_throttle',
            parameters=[{
                'messages': LaunchConfiguration('fix_topic'),
                'rate': LaunchConfiguration('rate')
            }],
            remappings=[('messages', 'fix')]
        )
    ])
from launch.actions import SetEnvironmentVariable
