# Standalone build for the Chrono Pac02 reference generator.
#
# This is INTENTIONALLY separate from VDSim's CMake tree — it links Project Chrono,
# which VDSim's core never depends on. Configure it pointing at an existing Chrono
# build (or install):
#
#   cmake -B build -DChrono_DIR=<chrono-build>/cmake
#   cmake --build build -j
#   ./build/gen_reference "$(pwd)"     # writes reference/pac02_reference.csv
#
# See README.md.
cmake_minimum_required(VERSION 3.18)
project(chrono_parity_gen CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Chrono COMPONENTS Vehicle CONFIG REQUIRED)

add_executable(gen_reference gen_reference.cpp)
target_include_directories(gen_reference PRIVATE ${CHRONO_INCLUDE_DIRS})
target_compile_options(gen_reference PRIVATE ${CHRONO_CXX_FLAGS})
target_link_libraries(gen_reference PRIVATE ${CHRONO_LIBRARIES})
