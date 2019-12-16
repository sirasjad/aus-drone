cmake_minimum_required(VERSION 3.10.2)
project(gui)

find_package(catkin REQUIRED COMPONENTS roscpp asd_msg image_transport)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
find_package(Qt5Widgets REQUIRED)

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
#  LIBRARIES gui
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


set(header_directory ${PROJECT_SOURCE_DIR}/include)

set(
  gui_hpp
  ${header_directory}/detectedobjectwindow.hpp
  ${header_directory}/mainwindow.hpp
  ${header_directory}/roslink/roslink.hpp
)

set(ui_directory ${PROJECT_SOURCE_DIR}/ui)

set(
  gui_ui
  ${ui_directory}/detectedobjectwindow.ui
  ${ui_directory}/mainwindow.ui
)

qt5_wrap_cpp(gui_hpp_moc ${gui_hpp})
qt5_wrap_ui(gui_ui_moc ${gui_ui})

add_executable(
  gui
  src/detectedobjectwindow.cpp
  src/main.cpp
  src/mainwindow.cpp
  src/roslink/roslink.cpp
  ${gui_hpp_moc}
  ${gui_ui_moc}
)

add_dependencies(gui ${gui_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_options(gui PRIVATE -Wall -Wextra -Werror)
## Specify libraries to link a library or executable target against

target_link_libraries(
  gui
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
