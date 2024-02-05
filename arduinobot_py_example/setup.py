from setuptools import find_packages, setup

package_name = 'arduinobot_py_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theo',
    maintainer_email='theo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 추가!
            'simple_publisher = arduinobot_py_example.simple_publisher:main',
            # 노드명은 왼쪽과 같고 오른쪽 경로에 있으며 main 함수가 시작임
            # 이거하고 package.xml 수정
            
            # 추가2!
            'simple_subscriber = arduinobot_py_example.simple_subscriber:main',
        
            # 또 추가
            'simple_parameter = arduinobot_py_example.simple_parameter:main',
            # 하고 package로
            
            # 추가!
            'simple_service_server = arduinobot_py_example.simple_service_server:main',
        ]
    },
)
