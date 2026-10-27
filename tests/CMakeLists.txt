cmake_minimum_required(VERSION 3.28)

# Host tests of this chip's clock arithmetic (src/chip/ClockMath.hpp): the PLL solver, the flash and regulator tables,
# the timer, SPI, ADC and watchdog dividers, the CAN bit timing; of the instance table (src/chip/Instances.hpp); and of
# the reset-cause decode (src/chip/ResetCause.hpp), of the flash sector map (src/chip/FlashLayout.hpp), of the GPIO
# toggle (src/chip/OdrToggle.hpp, on the SDK's register mock) and of the USART byte queues (src/chip/UsartQueue.hpp).
# Most checks are static_asserts: building is the test. Needs Kvasir_SDK's Util/Prescaler.hpp, Port/ResetKind.hpp and
# Atomic/Queue.hpp - the checkout next to this one (KVASIR_ROOT), else fetched:
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
project(chip_stm32f413_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(SDK_SUBMODULES svd_converter)
get_filename_component(_chip_src "${CMAKE_CURRENT_LIST_DIR}/../src/chip" ABSOLUTE)

add_executable(clock_math clock_math_test.cpp)
target_include_directories(clock_math PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(clock_math PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_math COMMAND clock_math)

# Instances.hpp: the bus of every peripheral instance (RM0430 Table 1) and the SPI limits (DS11581 Table 67)
add_executable(instances instances_test.cpp)
target_include_directories(instances PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(instances PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME instances COMMAND instances)

# busOfBase refuses a block on no APB (GPIOA)
add_executable(instances_must_fail EXCLUDE_FROM_ALL instances_must_fail.cpp)
target_include_directories(instances_must_fail PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
add_test(NAME instances_must_fail COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target instances_must_fail)
set_tests_properties(instances_must_fail PROPERTIES PASS_REGULAR_EXPRESSION "baseIsOnNoApbBus")

# ClockMath.hpp's bxCAN bit timing: exact, in range, the closest sample point (exhaustive), the conformance bus's value
add_executable(can_timing can_timing_test.cpp)
target_include_directories(can_timing PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(can_timing PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME can_timing COMMAND can_timing)

# UsartQueue.hpp: the port's byte queues hold exactly TxSize / RxSize; send and the receive drop count at exactly N
add_executable(usart_queue usart_queue_test.cpp)
target_include_directories(usart_queue PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(usart_queue PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME usart_queue COMMAND usart_queue)

add_executable(reset_cause reset_cause_test.cpp)
target_include_directories(reset_cause PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(reset_cause PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME reset_cause COMMAND reset_cause)

# FlashLayout.hpp: the sector map (RM0430 Table 5) and Nvm::Region's geometry checks; the must-fail cases are built by
# ctest and must stop at their static_assert
add_executable(flash_layout flash_layout_test.cpp)
target_include_directories(flash_layout PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(flash_layout PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME flash_layout COMMAND flash_layout)
foreach(case_and_text "1;one size" "2;starts on a sector" "3;sectors exist")
    list(GET case_and_text 0 _case)
    list(GET case_and_text 1 _text)
    add_executable(flash_layout_must_fail_${_case} EXCLUDE_FROM_ALL flash_layout_must_fail.cpp)
    target_include_directories(flash_layout_must_fail_${_case} PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
    target_compile_definitions(flash_layout_must_fail_${_case} PRIVATE FLASH_MUST_FAIL=${_case})
    add_test(NAME flash_layout_must_fail_${_case} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target
                                                          flash_layout_must_fail_${_case})
    set_tests_properties(flash_layout_must_fail_${_case} PROPERTIES PASS_REGULAR_EXPRESSION "${_text}")
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

    # OdrToggle.hpp: a GPIO toggle is an ODR read and one BSRR write (RM0430 7.3.5), on the SDK's register mock. Io.hpp
    # names the generated GPIO registers: this chip's SVD converted here, as an image does (cmake/chip.cmake).
    include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake) # kvasir::test_support
    svd_convert(chip_registers SVD_FILE ${CMAKE_CURRENT_LIST_DIR}/../chip.svd OUTPUT_DIRECTORY peripherals)
    add_executable(odr_toggle odr_toggle_test.cpp)
    target_link_libraries(odr_toggle PRIVATE kvasir::test_support chip_registers)
    # the seam as an image gets it (cmake/chip.cmake): Register/Exec.hpp includes OdrToggle.hpp at its end
    target_compile_definitions(odr_toggle PRIVATE KVASIR_REGISTER_MOCK [[KVASIR_CHIP_REGISTER_SEAM="OdrToggle.hpp"]])
    target_include_directories(odr_toggle PRIVATE ${_chip_src})
    target_compile_options(odr_toggle PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME odr_toggle COMMAND odr_toggle)
else()
    message(STATUS "svd_write_only_guard and odr_toggle left out: no ${KVASIR_ROOT}/svd_converter")
endif()
