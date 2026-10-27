cmake_minimum_required(VERSION 3.8)
project(delivery_planner)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/cost_matrix_builder.py
  scripts/calibrate_time_matrix.py
  scripts/evaluate_routes.py
  scripts/freeze_experiment.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md EXPERIMENT_PROTOCOL.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(planner_tests test)
endif()
ament_package()
