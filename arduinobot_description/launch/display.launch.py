import os # 경로 생성에 필요함
from ament_index_python.packages import get_package_share_directory

# 런치 라이브러리(주로 로스 제외 외부 활동)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument #모델 선언을 위해 필요한 함수
from launch.substitutions import Command, LaunchConfiguration # xacro 변환을 위한 command

# 런치로스 라이브러리(주로 로스 담당)
from launch_ros.actions import Node # 노드 가져오기
from launch_ros.parameter_descriptions import ParameterValue

# 아 함수는 런치파일을 명령어로 부르면 바로 실행 됨 (메인 같은거 없음)
def generate_launch_description():
    # 우리 arduinobot_description에 접근하기 위한 경로, 아래에 두 곳에서 쓰임.
    arduinobot_description_dir = get_package_share_directory('arduinobot_description')

    # urdf 모델 다이렉토리, name은 argument 이름, os를 사용해서 urdf xacro 가져오고(위치는 위에 변수로 지정됨), 폴도명, 파일명
    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        arduinobot_description_dir, 'urdf', 'arduinobot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file')

    # 파라미터에 넣을 로봇 모델 위치 변수, 바로 아래에 쓰임
    # LaunchConfiguration을 통해 launch 클래스에 있는(위에서 지정한), 'model'의 경로를 가져옴
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # 시작하고 싶은 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher', # 패키지 지정
        executable='robot_state_publisher', # 노드 지정
        # 필요 파라미터 지정, 이거는 로봇 설명이 어디있는지 지정. 위치는 일단 변수고 어디에 있는지는 위에 지정 돼있음.
        parameters=[{'robot_description': robot_description}]
    )

    # 조인트 인터페이스 노드
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RVIZ 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',# 노드가 실행되는지 확인하기 위해 output 표시
        # 저장했던 설정을 불러오기 위해 argument 저장 필요
        # 이번에도 os 경로 찾기 사용, 위에 변수에 저장된 경로 이용, arduinobot_description 속에 rviz 폴더에, 파일명
        arguments=['-d', os.path.join(arduinobot_description_dir, 'rviz', 'display.rviz')],
    )

    # 시작하는 모든 노드 및 파일 반환
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
    # 쓰고 씨메이크로