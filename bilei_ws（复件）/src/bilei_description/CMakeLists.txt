cmake_minimum_required(VERSION 3.8)
project(bilei_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/range_bridge.py
  scripts/occupancy_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
