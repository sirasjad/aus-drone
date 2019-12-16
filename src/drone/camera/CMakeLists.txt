cmake_minimum_required(VERSION 2.8.3)
project(camera)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp asd_msg cv_bridge image_transport)
find_package(OpenCV REQUIRED)
#find_package(raspicam REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/camera.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Werror)

## Specify libraries to link a library or executable target against
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBS})
if (${ARCHITECTURE} STREQUAL "armv7l")
  set(LIBS ${LIBS} -lraspicam -lraspicam_cv)
  message("Compiling on raspberry pi")
  add_definitions(-DRASPI)
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS})
