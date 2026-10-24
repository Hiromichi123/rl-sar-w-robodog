cmake_minimum_required(VERSION 3.15)
project(rl_nav_examples)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/simple_goal_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
