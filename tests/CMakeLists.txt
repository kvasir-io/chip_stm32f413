cmake_minimum_required(VERSION 3.28)

# Host tests of this chip's clock arithmetic (src/chip/ClockMath.hpp): the PLL solver, the flash and regulator tables,
# the timer, SPI, ADC and watchdog dividers. Most checks are static_asserts: building is the test. Needs Kvasir_SDK's
# Util/Prescaler.hpp - the checkout next to this one (KVASIR_ROOT), else fetched:
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
project(chip_stm32f413_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

if(NOT KVASIR_ROOT AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK/src/kvasir/Util/Prescaler.hpp)
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
endif()
if(NOT KVASIR_ROOT)
    # Headers of src/ and the svd_converter submodule (the write-only guard test); no configure of the SDK.
    include(FetchContent)
    FetchContent_Populate(
        kvasir_sdk_headers
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SUBMODULES "svd_converter")
    set(KVASIR_ROOT ${kvasir_sdk_headers_SOURCE_DIR})
endif()
get_filename_component(_chip_src "${CMAKE_CURRENT_LIST_DIR}/../src/chip" ABSOLUTE)

add_executable(clock_math clock_math_test.cpp)
target_include_directories(clock_math PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(clock_math PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_math COMMAND clock_math)

# core_cortex_m4's core_prio4.svd must be what its script makes from core.svd (needs the core submodule or the sibling
# checkout and python3).
find_package(Python3 COMPONENTS Interpreter)
foreach(core_dir ${CMAKE_CURRENT_LIST_DIR}/../core ${CMAKE_CURRENT_LIST_DIR}/../../core_cortex_m4)
    if(Python3_FOUND AND EXISTS ${core_dir}/scripts/make_prio4_svd.py)
        add_test(NAME core_prio4_svd_up_to_date COMMAND ${Python3_EXECUTABLE} -B ${core_dir}/scripts/make_prio4_svd.py
                                                        --check)
        break()
    endif()
endforeach()

# The write-only guard: every write-only field of this chip's SVD(s) is classified (oneToSet, a key, or <!-- Kvasir:
# write-only accepted -->), so svd_converter --write-only-guard=error must exit 0. Needs the SDK's svd_converter (the
# sibling checkout's submodule); left out without it.
if(EXISTS ${KVASIR_ROOT}/svd_converter/CMakeLists.txt)
    add_subdirectory(${KVASIR_ROOT}/svd_converter svd_converter)
    foreach(svd chip.svd)
        string(MAKE_C_IDENTIFIER ${svd} _svd_name)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/write_only_guard/${_svd_name})
        add_test(
            NAME svd_write_only_guard_${_svd_name}
            COMMAND svd_converter --write-only-guard=error --write-only-registers=derived
                    ${CMAKE_CURRENT_LIST_DIR}/../${svd} ${CMAKE_CURRENT_BINARY_DIR}/write_only_guard/${_svd_name} json)
    endforeach()
else()
    message(STATUS "svd_write_only_guard left out: no ${KVASIR_ROOT}/svd_converter")
endif()
