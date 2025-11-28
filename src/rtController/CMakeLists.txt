file(GLOB_RECURSE INC "*.h")
file(GLOB_RECURSE SRC "*.c")

include_directories(${ETHERCAT_INCLUDE_DIR})
set(EXTRA_LIBS
    ${XENOMAI_LIBRARY_NATIVE} ${XENOMAI_LIBRARY_XENOMAI}
    ${ECAT_LIBRARY}
#    ${BECKHOFF_MODULES_PATH}
    rt
    pthread
    m
    ec_el1018
    ec_el3104
    ec_el4134
    ec_el5101
    ec_el3164
    ec_el4104
    ec_el4132
    ec_el1008
    ec_el2212
    ec_el2004
    ec_el2828
    ec_el2124
    rtJPosPlanner
    rtJVelPlanner
    rtPIDControl
    rtAuxeticInchwormActuator
)

add_executable(rtController ${INC} ${SRC})
target_link_libraries(rtController ${EXTRA_LIBS})

