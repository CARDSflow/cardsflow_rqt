cmake_minimum_required(VERSION 2.8.3)
project( cardsflow_rqt )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS common_utilities roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_communication_middleware )
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_communication_middleware common_utilities 
        DEPENDS YAML_CPP)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(cardsflow_rqt_MOCS include/cardsflow_rqt/cardsflow_rqt.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(cardsflow_rqt_UIS_H src/cardsflow_rqt.ui)
else()
  qt4_wrap_cpp(cardsflow_rqt_MOCS include/cardsflow_rqt/cardsflow_rqt.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(cardsflow_rqt_UIS_H src/cardsflow_rqt.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library( cardsflow_rqt src/cardsflow_rqt.cpp ${cardsflow_rqt_MOCS} ${cardsflow_rqt_UIS_H}
        qcustomplot/qcustomplot.cpp)
add_dependencies(cardsflow_rqt roboy_communication_middleware_gencpp)
target_link_libraries(cardsflow_rqt ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} -ldl)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(cardsflow_rqt Qt5::Widgets)
else()
  target_link_libraries(cardsflow_rqt ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS cardsflow_rqt
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )