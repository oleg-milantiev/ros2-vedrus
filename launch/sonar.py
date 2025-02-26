from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vedrus',
            executable='sonar',
            name='sonar',
            output='screen',
            parameters=[
                {'pins_trig': [127, 125, 124, 144]},
                {'pins_echo': [122, 120, 123, 121]},
#                 {'pins_trig': [127]},
#                 {'pins_echo': [122]},
                {'topic_name': '/vedrus/sonar'},
                {'names': ['left', 'left-rear', 'right', 'right-rear']},
                {'azimuths': [270.0, 200.0, 90.0, 160.0]},
            ]
        ),
    
    ])