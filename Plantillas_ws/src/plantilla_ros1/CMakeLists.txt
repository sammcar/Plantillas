cmake_minimum_required(VERSION 3.0.2)
project(cambiar_nombre)

## Compilación estricta para GCC o Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Encuentra catkin y los paquetes requeridos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Declarar este paquete como un paquete catkin
catkin_package()

## Incluye directorios si fuera necesario
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Instalación de scripts Python
# install(PROGRAMS
#   scripts/nombre_nodo.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Instalación de archivos adicionales (por ejemplo, imágenes, URDF, config)
# install(FILES
#   src/archivos_random.gif
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY config launch meshes urdf etc
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
