cmake_minimum_required(VERSION 2.8.10 FATAL_ERROR)
project(power-control CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH}
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
include(GNUInstallDirs)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(DBUS_OBJECT_NAME "xyz/openbmc_project/Chassis/Control/Power")
set(DBUS_INTF_NAME "xyz.openbmc_project.Chassis.Control.Power")

add_definitions(-DDBUS_OBJECT_NAME="/${DBUS_OBJECT_NAME}0")
add_definitions(-DDBUS_INTF_NAME="${DBUS_INTF_NAME}")
set(SRC_FILES
    src/power_control.cpp
    src/timer.cpp
    src/main.cpp
)

# import sdbusplus
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDBUSPLUSPLUS sdbusplus REQUIRED)
include_directories(${SDBUSPLUSPLUS_INCLUDE_DIRS})
link_directories(${SDBUSPLUSPLUS_LIBRARY_DIRS})
find_program(SDBUSPLUSPLUS sdbus++)

# import phosphor-logging
find_package(PkgConfig REQUIRED)
pkg_check_modules(LOGGING phosphor-logging REQUIRED)
include_directories(${LOGGING_INCLUDE_DIRS})
link_directories(${LOGGING_LIBRARY_DIRS})

# phosphor-dbus-interfaces
find_package(PkgConfig REQUIRED)
pkg_check_modules(DBUSINTERFACE phosphor-dbus-interfaces REQUIRED)
include_directories(${DBUSINTERFACE_INCLUDE_DIRS})
link_directories(${DBUSINTERFACE_LIBRARY_DIRS})

option (EXTERNAL_PROJECT "Enable Cloning External Projects" OFF)
if (EXTERNAL_PROJECT)
    include_directories (${CMAKE_CURRENT_SOURCE_DIR}/inc/non-yocto)
endif()

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${DBUSINTERFACE_LIBRARIES}
                      chassisgpio chassisi2c)
target_link_libraries(${PROJECT_NAME} "${SDBUSPLUSPLUS_LIBRARIES} -lstdc++fs -lphosphor_dbus")

install (TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
