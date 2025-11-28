#cmake_minimum_required(VERSION 2.8.11)

 #project(QtController)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH /opt/Qt/5.4/gcc_64)
find_package(Qt5Widgets REQUIRED)

#include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
#set(RTCONTROLLER_BIN_PATH ${PROJECT_BINARY_DIR}/rtController)
#configure_file( "QtControllerConfig.h.in"
#                "QtControllerConfig.h")
set(    QTPROTECT_SOURCES
        main.cpp
        sRobot.cpp
        mainwindow.cpp
        DialogInchWormActuator.cpp
        dialogflatinchwormctrl.cpp
        dialogSPIRITControl.cpp
        DialogAdvancedControl.cpp
        DialogWren.cpp
        qledindicator.cpp
        sRTFifo.cpp
        sEtherCAT.cpp
        sRTController.cpp
        sRemoteControl.cpp)

set(    QTPROTECT_HEADERS
	sRTFifo.h
        sEtherCAT.h
        sRTController.h
        sRemoteControl.h
        sRobot.h
        DialogInchWormActuator.h
        dialogSPIRITControl.h
        dialogflatinchwormctrl.h
        DialogAdvancedControl.h
        DialogWren.h
        mainwindow.h
        master_parameters_interface.h
        qledindicator.h)

set(    QTPROTECT_MOC
        mainwindow.h
        DialogInchWormActuator.h
        dialogSPIRITControl.h
        dialogflatinchwormctrl.h
        DialogAdvancedControl.h
        DialogWren.h
	qledindicator.h
        sEtherCAT.h
        sRTCOntroller.h
        sRemoteControl.h)

set(QTPROTECT_FORMS
    mainwindow.ui
        dialogflatinchwormctrl.ui
        DialogWren.ui
        DialogAdvancedControl.ui
    DialogInchWormActuator.ui
        dialogSPIRITControl.ui)

qt5_wrap_ui(QT_UI_FORMS ${QTPROTECT_FORMS})

add_executable(QtController ${QTPROTECT_SOURCES}  ${QT_UI_FORMS})
target_link_libraries(QtController Qt5::Widgets)
QT5_USE_MODULES(QtController Core Network)
