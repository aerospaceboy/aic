cmake_minimum_required(VERSION 3.22)
project(mybot)

find_package(ament_cmake REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS auto_grasp_moveit.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_grasp_moveit
)

install(
  PROGRAMS pick_place_moveit.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_moveit
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pick_place_math
    test/test_pick_place_math.py
  )
endif()

ament_package()

