cmake_minimum_required(VERSION 3.10)
 project(rtController C)

# set(CMAKE_C_STANDARD 11)

# # Use pthread
# find_package(Threads REQUIRED)

# add_executable(rt_example
#     rtController/rtController.c
# )

# target_link_libraries(rtController
#     Threads::Threads
# )

# # Enable compile flags useful for RT
# target_compile_options(rtController PRIVATE -Wall -Wextra -O2)



set(CMAKE_DISABLE_SOURCE_CHANGES ON)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
# Extra cmake modules path (must be set by toolchain file passed from command line)
message(${PROJECT_SOURCE_DIR})

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Install path for relocatable copy (under the build directory)

set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/install/usr/local/${PROJECT_NAME})

find_package(EtherCAT REQUIRED)


include_directories(${INCLUDE_DIRECTORIES} ${ETHERCAT_INCLUDE_DIR}) # ${QT_USE_FILE})
add_subdirectory(${PROJECT_SOURCE_DIR}/CommonLibs/rtBeckhoffModules)
add_subdirectory(${PROJECT_SOURCE_DIR}/CommonLibs/rtControllerModules)


add_subdirectory(${PROJECT_SOURCE_DIR}/rtController)

set(CMAKE_DISABLE_SOURCE_CHANGES OFF)

set(CMAKE_DISABLE_IN_SOURCE_BUILD OFF)
#add_subdirectory(${PROJECT_SOURCE_DIR}/UIController)


