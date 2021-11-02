cmake_minimum_required(VERSION 3.0.2)
project(sim2real)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(TEAM_NAME "RLLAB")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  ackermann_msgs
  message_generation
  message_filters
)

add_message_files(
   FILES
   Query.msg
   Result.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  project/IS_${TEAM_NAME}/project/${TEAM_NAME}_project1.py
  scripts/eval_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
