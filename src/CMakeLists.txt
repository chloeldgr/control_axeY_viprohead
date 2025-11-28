cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


set(CMAKE_DISABLE_SOURCE_CHANGES ON)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
# Extra cmake modules path (must be set by toolchain file passed from command line)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Install path for relocatable copy (under the build directory)

set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/install/usr/local/${PROJECT_NAME})
# Xenomai libraries

# Libraries: ${XENOMAI_LIBRARY_NATIVE} ${XENOMAI_LIBRARY_XENOMAI}

find_package(Xenomai REQUIRED)
find_package(EtherCAT REQUIRED)


include_directories(${INCLUDE_DIRECTORIES} ${XENOMAI_INCLUDE_DIR} ${ETHERCAT_INCLUDE_DIR}) # ${QT_USE_FILE})
add_subdirectory(${PROJECT_SOURCE_DIR}/CommonLibs/rtBeckhoffModules)
add_subdirectory(${PROJECT_SOURCE_DIR}/CommonLibs/rtControllerModules)


add_subdirectory(${PROJECT_SOURCE_DIR}/rtController)
#add_subdirectory(${PROJECT_SOURCE_DIR}/hal)




set(CMAKE_DISABLE_SOURCE_CHANGES OFF)

set(CMAKE_DISABLE_IN_SOURCE_BUILD OFF)
add_subdirectory(${PROJECT_SOURCE_DIR}/UIController)


