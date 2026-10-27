cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# Python node started from launch/navigation.launch.py
install(PROGRAMS scripts/cmd_vel_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
