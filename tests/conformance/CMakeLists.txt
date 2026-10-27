cmake_minimum_required(VERSION 3.28)

# The chip's conformance tree (Kvasir_SDK tests/conformance/conformance.cmake): this package's headers compiled for the
# target with a firmware's flags, against the generated register headers. Building is the test - the contract rows
# (conformance.cpp), the dependent register writes (instantiate.cpp) and every public header alone. The chip and core
# SVDs convert with the write-only guard on the way.
#
# cmake -S tests/conformance -B tests/conformance/build -G Ninja [-DCOMPILE_TARGET=arm_gcc] && cmake --build
# tests/conformance/build
#
# Kvasir_SDK: the checkout next to this one, else fetched (FETCHCONTENT_SOURCE_DIR_KVASIR_SDK points a lone checkout at
# a local copy). No kvasir_devices: no header of this chip includes it.

get_filename_component(CHIP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../Kvasir_SDK" ABSOLUTE)
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
kvasir_locate(SDK_SUBMODULES ALL)

include(${KVASIR_ROOT}/tests/conformance/conformance.cmake)
kvasir_conformance_begin()

# a version: the must-fail fixture lists the boot report, which prints the project version
project(
    chip_stm32f413_conformance
    VERSION 0.0.0
    LANGUAGES CXX C ASM)

include(${KVASIR_ROOT}/cmake/util.cmake)

kvasir_conformance_project(
    NAME
    chip_stm32f413_conformance
    SOURCES
    conformance.cpp
    INSTANTIATE
    instantiate.cpp
    CONFIG_HEADER
    kvasir_config.hpp
    HEADERS
    ${CHIP_ROOT}/src/chip/*.hpp
    NOT_ALONE
    not_alone.txt
    MAIN_TU
    main_tu.cpp)

# The Startup resource checks (each driver claims what it configures and owns, the clock settings provide the bus clocks
# they program, an instance row names its enable register's bus): resources_must_fail.cpp with RESOURCES_FAIL=0 compiles
# in `all`, each case below must fail with its diagnostic (ctest in this tree, both compilers).
kvasir_conformance_must_fail(
    NAME
    resources_must_fail
    PROJECT
    chip_stm32f413_conformance
    SOURCE
    resources_must_fail.cpp
    DEFINE
    RESOURCES_FAIL
    CASES
    "1|provided [(]configured, owned[)] by two peripherals: EXTI line 4"
    "2|provided [(]configured, owned[)] by two peripherals: Io pin 2[.]13"
    "3|provided [(]configured, owned[)] by two peripherals: timer [(]TIMx, PSC, ARR[)] 3[.]"
    "4|provided [(]configured, owned[)] by two peripherals: timer channel [(]TIMx, channel[)] 3[.]2"
    "5|provided [(]configured, owned[)] by two peripherals: timer [(]TIMx, PSC, ARR[)] 2[.]"
    "6|provided [(]configured, owned[)] by two peripherals: peripheral block at 0x40012000"
    "7|nothing in any Startup list provides: Kvasir::Clocks::ClkApb2 84000000"
    "8|nothing in any Startup list provides: Kvasir::Clocks::ClkApb2 50000000"
    "9|this register set is on no APB bus")

# The watchdog's gate: watchdog_must_fail.cpp with WATCHDOG_FAIL=0 compiles in `all`, each case below must fail (a gated
# feed or arm without the supervisor's key, a disarm of the IWDG).
kvasir_conformance_must_fail(
    NAME
    watchdog_must_fail
    PROJECT
    chip_stm32f413_conformance
    SOURCE
    watchdog_must_fail.cpp
    DEFINE
    WATCHDOG_FAIL
    CASES
    "1|no matching function for call to .*[':]feed['(]"
    "2|no matching function for call to .*[':]arm['(]"
    "3|no matching function for call to .*[':]disarm['(]")
