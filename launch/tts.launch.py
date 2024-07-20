from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'url',
            default_value='http://localhost:5002',
            description='Set Coqui TTS server url'
        ),
        DeclareLaunchArgument(
            'addStopChar',
            default_value='true',
            description='Add period at the end of a sentence'
        ),
        DeclareLaunchArgument(
            'filename',
            default_value='output.wav',
            description='Set result sound filename'
        ),
        DeclareLaunchArgument(
            'style_wav',
            default_value='',
            description='Set input style_wav if sample voice is given'
        ),
        DeclareLaunchArgument(
            'speaker_id',
            default_value='p225',
            description='Set Speaker ID if multi-speaker model is being used'
        ),
        DeclareLaunchArgument(
            'language_id',
            default_value='',
            description='Set Language if multi-language model is being used'
        ),
        DeclareLaunchArgument(
            'sound_audio',
            default_value='true',
            description='Set sound_audio to true if you want to play the sound'
        ),

        Node(
            package='coqui_tts_ros',
            executable='tts',
            name='coqui_tts_ros',
            output='screen',
            parameters=[{
                'url': LaunchConfiguration('url'),
                'addStopChar': LaunchConfiguration('addStopChar'),
                'filename': LaunchConfiguration('filename'),
                'speaker_id': LaunchConfiguration('speaker_id'),
                'language_id': LaunchConfiguration('language_id'),
            }]
        )
    ])
