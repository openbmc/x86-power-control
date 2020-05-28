cmake_minimum_required(VERSION 2.8.10 FATAL_ERROR)
project(power-control CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions(-DBOOST_ALL_NO_LIB)
add_definitions(-DBOOST_NO_RTTI)
add_definitions(-DBOOST_NO_TYPEID)
add_definitions(-DBOOST_ASIO_DISABLE_THREADS)

set(SRC_FILES src/power_control.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} -lstdc++fs)
target_link_libraries(${PROJECT_NAME} chassisi2c)
target_link_libraries(${PROJECT_NAME} i2c)
target_link_libraries(${PROJECT_NAME} gpiodcxx)
target_link_libraries(${PROJECT_NAME} systemd)
target_link_libraries(${PROJECT_NAME} sdbusplus)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")

set(
  SERVICE_FILES
  ${PROJECT_SOURCE_DIR}/service_files/xyz.openbmc_project.Chassis.Control.Power@.service
  ${PROJECT_SOURCE_DIR}/service_files/xyz.openbmc_project.Chassis.Control.Power.service
  )
install(FILES ${SERVICE_FILES} DESTINATION /lib/systemd/system/)
