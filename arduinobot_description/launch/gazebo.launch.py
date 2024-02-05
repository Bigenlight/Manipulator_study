import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

# 저번에 했던 것과 같이 런치 라이브러리, 지반번에 비해 몇개의 패키지가 추가됨
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 런치 로스 라이브러리
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

##### 기본적으로 가져와야하는 것은 같기 때문에 디스플레이 런치 파일에서 복붙
def generate_launch_description():
 
    arduinobot_description_dir = get_package_share_directory('arduinobot_description')
    # New os으로 디스크립션 share 경로 가져오고 변수에 저장
    arduinobot_description_share = os.path.join(get_package_prefix('arduinobot_description'), 'share')
    # New os으로 gazebo_ros를 가져와 저장
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        arduinobot_description_dir, 'urdf', 'arduinobot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    # New 환경 설정
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', arduinobot_description_share)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # New 시뮬레이션 연산이 일어나는 곳
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( # os로 시작 되는 위치 가져오기, 위에 저장된 경로 변수 넣고 런치 파일로 서버 
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )
    
    # New 시각화 인터페이스
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )
    # 여기까지만 하면 빈 공간은 생성 됨, 로봇을 환경에 소환하려면 아래 코드 작성
    
    # New
    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'arduinobot',
                                   '-topic', 'robot_description',
                                  ],
                        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])