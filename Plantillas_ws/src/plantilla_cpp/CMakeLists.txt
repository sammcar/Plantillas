cmake_minimum_required(VERSION 3.8)
project(cambiar_nombre)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# Install Python scripts
#install(PROGRAMS
#  src/nombre_nodo.py
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install necessary files
#install(FILES 
#	src/archivos_random.gif
#    DESTINATION lib/${PROJECT_NAME}
#)

# Install Directories
#install(DIRECTORY config launch meshes urdf etc
#  DESTINATION lib/${PROJECT_NAME})

ament_package()
