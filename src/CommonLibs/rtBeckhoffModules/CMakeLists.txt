set(EXTRA_LIBS
    ${ECAT_LIBRARY}
    rt
    pthread
    m
)
SET(BECKHOFF_LIBS   ec_el1018
                    ec_el1084
                    ec_el2004
                    ec_el2124 ec_el3102 ec_el3104 ec_el3204 ec_el4132 ec_el4134 ec_el5101 ec_el1144 ec_el3164 ec_el4104 ec_el1008 ec_el2212 ec_el2828)
foreach(beckhoff_libs ${BECKHOFF_LIBS})
	add_library(${beckhoff_libs} ${beckhoff_libs}.c)
	target_include_directories(${beckhoff_libs} PUBLIC include)
endforeach(beckhoff_libs)
