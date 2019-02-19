cmake_minimum_required(VERSION 2.8.0)
project(qt_ros)
set(CMAKE_BUILD_TYPE Release) #RelWithDebInfo
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )
include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL 1.7.2 EXACT)
#find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

catkin_package()

include_directories(src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Qt5Widgets REQUIRED)
add_definitions(-DQT_NO_KEYWORDS -g)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(QT_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/images.qrc)
set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/ui/main_window.ui)

set(HEADER_FILES
  src/qnode.h
  src/main_window.h
  )
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

set(SOURCE_FILES
  src/qnode.cpp
  src/main_window.cpp
  ${QT_RESOURCES_CPP}
  ${QT_FORMS_HPP}
  ${MOC_FILES}
  )

add_executable(qt_ros src/main.cpp ${SOURCE_FILES})
add_dependencies(qt_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(qt_ros PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()
target_link_libraries(qt_ros Qt5::Widgets ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS qt_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
