cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  carla_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 원본 실행 파일 선언
add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)

# 새로운 최종 실행 파일 선언
add_executable(pure_pursuit_controller_advanced src/pure_pursuit_controller_advanced.cpp)

# [핵심 수정] catkin이 메시지 헤더를 먼저 생성하도록 빌드 순서 의존성을 명시합니다.
# 이것이 'No such file or directory' 오류를 해결하는 핵심입니다.
add_dependencies(pure_pursuit_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pure_pursuit_controller_advanced ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 각 실행 파일에 필요한 라이브러리를 정확히 링크합니다.
target_link_libraries(pure_pursuit_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(pure_pursuit_controller_advanced
  ${catkin_LIBRARIES}
)