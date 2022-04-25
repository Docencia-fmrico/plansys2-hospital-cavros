cmake_minimum_required(VERSION 3.5)
project(plansys2-hospital-cavros)

# Para compilar en C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

# Se incluyen los paquetes usados
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(pick_object_node src/pick_object_node.cpp)
ament_target_dependencies(pick_object_node ${dependencies})

add_executable(place_object_node src/place_object_node.cpp)
ament_target_dependencies(place_object_node ${dependencies})

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

install(TARGETS
  move_action_node
  pick_object_node
  place_object_node
  controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Se deben instalar los directorios
install(DIRECTORY launch models worlds pddl
    DESTINATION share/${PROJECT_NAME}
)

# Para los test
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
