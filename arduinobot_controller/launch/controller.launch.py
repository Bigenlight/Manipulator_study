import os # 경로 생성에 필요함

# 런치 라이브러리(주로 로스 제외 외부 활동)
from launch import LaunchDescription
from launch.substitutions import Command

# 런치로스 라이브러리(주로 로스 담당)
from launch_ros.actions import Node # 노드 가져오기
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 저번에 model_arg로 가져오는게 아니라 바로 urdf로 가기
    robot_description = ParameterValue(
        Command( # 리스트로 command 실행
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("arduinobot_description"),
                    "urdf",
                    "arduinobot.urdf.xacro",
                ),
            ]
        ),
        value_type=str, # 위 파일을 스트링으로 로드해야함
    )
    
    # 시작하고싶은 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher', # 패키지 지정
        executable='robot_state_publisher', # 노드 지정
        # 필요 파라미터 지정, 이거는 로봇 설명이 어디있는지 지정. 위치는 일단 변수고 어디에 있는지는 위에 지정 돼있음.
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
    # 