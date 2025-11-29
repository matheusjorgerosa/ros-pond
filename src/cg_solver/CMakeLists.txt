cmake_minimum_required(VERSION 3.8)
project(cg_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# --- Executável da Parte 1 (Navegação) ---
add_executable(parte1 src/parte1.cpp)
ament_target_dependencies(parte1 rclcpp cg_interfaces)

# --- Executável da Parte 2 (Mapeamento) ---
add_executable(parte2 src/parte2.cpp)
ament_target_dependencies(parte2 rclcpp cg_interfaces)

install(TARGETS
  parte1
  parte2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
