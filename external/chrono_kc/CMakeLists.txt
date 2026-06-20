# Standalone build for the Chrono KC (suspension kinematics) reference generator.
#
# INTENTIONALLY separate from VDSim's CMake tree — it links Project Chrono::Vehicle,
# which VDSim's core never depends on. Configure it pointing at an existing Chrono build:
#
#   cmake -B build -DChrono_DIR=<chrono-build>/cmake
#   cmake --build build -j
#   ./build/gen_kc_reference "$(pwd)/../.."   # writes reference/kc_dw_front_reference.csv
#
# See README.md (hardpoint mapping + CSV format).
cmake_minimum_required(VERSION 3.16)
project(chrono_kc_gen CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Chrono COMPONENTS Vehicle CONFIG REQUIRED)

add_executable(gen_kc_reference gen_kc_reference.cpp)
target_include_directories(gen_kc_reference PRIVATE ${CHRONO_INCLUDE_DIRS})
target_compile_options(gen_kc_reference PRIVATE ${CHRONO_CXX_FLAGS})
target_link_libraries(gen_kc_reference PRIVATE ${CHRONO_LIBRARIES})
