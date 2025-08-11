cmake_minimum_required(VERSION 3.10)
project(serial_library)

set(CMAKE_CXX_STANDARD 14)

set(SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${SERIAL_DIR}/include)

set(SERIAL_SRC
  ${SERIAL_DIR}/src/serial.cc
  ${SERIAL_DIR}/src/impl/unix.cc
  ${SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc  # 根据平台选择
)

add_library(serial SHARED ${SERIAL_SRC})
target_include_directories(serial PUBLIC ${SERIAL_DIR}/include)
target_link_libraries(serial PUBLIC pthread)  # Linux 需加线程库
