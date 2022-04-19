cmake_minimum_required(VERSION 3.5)
project(plansys2-hospital-cavros)

# Para compilar en C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

# Se incluyen los paquetes usados
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Se deben instalar los directorios
install(DIRECTORY launch models worlds
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
