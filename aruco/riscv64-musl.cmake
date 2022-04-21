set(CMAKE_SYSTEM_NAME "Linux")
set(CMAKE_C_COMPILER  riscv64-linux-musl-gcc)
set(CMAKE_CXX_COMPILER riscv64-linux-musl-g++)

set(CMAKE_CXX_FLAGS ""    CACHE STRING "")
set(CMAKE_C_FLAGS ""    CACHE STRING "")

set(CMAKE_C_FLAGS "-march=rv64gc ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-march=rv64gc ${CXX_FLAGS}")
