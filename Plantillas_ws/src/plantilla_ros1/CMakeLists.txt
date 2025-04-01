cmake_minimum_required(VERSION 3.0.2)
project(nombre_del_paquete)

## Opciones de compilación (opcional pero recomendado)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Buscar dependencias necesarias
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  turtlesim
  genmsg
  message_generation
)

## Dependencias externas (ej. OpenCV)
find_package(OpenCV REQUIRED)

## Declarar mensajes y servicios personalizados (si los tienes)
## Descomenta y personaliza los siguientes bloques si aplican

# add_message_files(
#   FILES
#   MiMensaje.msg
# )

# add_service_files(
#   FILES
#   MiServicio.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

## Declarar el paquete catkin
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs
  # message_runtime  # ← si usas mensajes/servicios personalizados
)

## Incluir headers de catkin y OpenCV
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Ejecutables en C++
add_executable(nodo_1 src/nodo_1.cpp)
add_executable(nodo_2 src/nodo_2.cpp)
add_executable(nodo_3 src/nodo_3.cpp)

## Dependencias de generación de mensajes
add_dependencies(nodo_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nodo_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nodo_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Vincular bibliotecas
target_link_libraries(nodo_1 ${catkin_LIBRARIES})
target_link_libraries(nodo_2 ${catkin_LIBRARIES})
target_link_libraries(nodo_3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Scripts Python (si los hay)
catkin_install_python(PROGRAMS
  scripts/mi_script.py
  scripts/otro_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Archivos de configuración, launch, etc.
install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
