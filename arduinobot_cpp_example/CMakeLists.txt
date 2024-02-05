# 우리의 파일들은 어떻게 실제 실행가능한 파일로 번역할건지 정하는 것.

cmake_minimum_required(VERSION 3.8)
project(arduinobot_cpp_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

## 추가
find_package(rclcpp REQUIRED) # 필요 패키지 추가
find_package(std_msgs REQUIRED) # 몰랐는데 std는 standard임
# 추가
find_package(rcl_interfaces REQUIRED) 

# 컴파일러에게 빌드 지시
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs) # Add rclcpp to dependencies
# executable 디펜던시

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs) 

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter rclcpp rcl_interfaces) 
# 하고 씨메이크

install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter

  DESTINATION lib/${PROJECT_NAME}
  )
# 이거 쓰고 패키지로


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
