#Archivo tomado de los ejemplos de "A gentle introduction to ROS" de Jason O'Kane 
#(http://www.cse.sc.edu/~jokane)

# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(guardar_imagenes)

# Find the catkin build system, and any other packages on which we depend.
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs)


# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(guardar_imagenes guardar_imagenes.cpp)

# Specify libraries against which to link.
target_link_libraries(guardar_imagenes ${catkin_LIBRARIES})
target_link_libraries(guardar_imagenes ${OpenCV_LIBS})

