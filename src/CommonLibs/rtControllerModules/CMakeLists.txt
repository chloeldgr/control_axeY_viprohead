set(EXTRA_LIBS
    ${XENOMAI_LIBRARY_NATIVE} ${XENOMAI_LIBRARY_XENOMAI}
    ${ECAT_LIBRARY}
    rt
    pthread
    m
)
SET(RTCONTROLLER_LIBS rtAuxeticInchwormActuator rtJPosPlanner rtJVelPlanner rtPIDControl )
foreach(rtController_libs ${RTCONTROLLER_LIBS})
	add_library(${rtController_libs} ${rtController_libs}.c)
	target_include_directories(${rtController_libs} PUBLIC include)
endforeach(rtController_libs)
