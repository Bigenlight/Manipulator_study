cmake_minimum_required(VERSION 3.8)
project(arduinobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

# 추가!
# 변경!
install(
  # 런치 파일에 추가한 launch와 rviz 추가, 쓰고 패키지
  DIRECTORY meshes urdf launch rviz# meshes와 urdf 폴더를 빌드가 읽을 수 있게
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
