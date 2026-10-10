cmake_minimum_required(VERSION 3.10)
project(spline_planner LANGUAGES CXX Fortran)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -std=legacy -fdefault-real-8")
set(FORTRAN_SOURCES
    fortran_src/curfit.f
    fortran_src/fpader.f
    fortran_src/fpback.f
    fortran_src/fpbspl.f
    fortran_src/fpchec.f
    fortran_src/fpcurf.f
    fortran_src/fpdisc.f
    fortran_src/fpgivs.f
    fortran_src/fpknot.f
    fortran_src/fprati.f
    fortran_src/fprota.f
    fortran_src/spalde.f
    fortran_src/splev.f
)

find_package(Eigen3 REQUIRED)
find_package(GSL REQUIRED)
find_package(ur_rtde REQUIRED)

# find_program(CLANG_TIDY_EXEC NAMES "clang-tidy")

# if(CLANG_TIDY_EXEC)
#     message(STATUS "clang-tidy found: ${CLANG_TIDY_EXEC}")
#     set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXEC}")
# else()
#     message(WARNING "clang-tidy not found.")
# endif()

add_library(SPL INTERFACE)
target_include_directories(SPL INTERFACE include)
target_link_libraries(SPL INTERFACE Eigen3::Eigen)

add_library(SPLPLANNER INTERFACE)
target_include_directories(SPLPLANNER INTERFACE include)
target_link_libraries(SPLPLANNER INTERFACE SPL GSL::gsl)

add_library(SPLEXECUTOR INTERFACE)
target_include_directories(SPLEXECUTOR INTERFACE include)
target_link_libraries(SPLEXECUTOR INTERFACE SPL ur_rtde::rtde)

add_executable(${PROJECT_NAME} src/main.cpp ${FORTRAN_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME_DEBUG "${PROJECT_NAME}_debug"
    OUTPUT_NAME_RELEASE "${PROJECT_NAME}"
)
target_link_libraries(${PROJECT_NAME} PUBLIC SPLPLANNER SPLEXECUTOR)

include(CheckCXXCompilerFlag)
include(CheckIPOSupported)

check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
    set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
check_cxx_compiler_flag("-ffast-math" COMPILER_SUPPORTS_FAST_MATH)

if(COMPILER_SUPPORTS_MARCH_NATIVE)
    target_compile_options(${PROJECT_NAME} PRIVATE 
        $<$<COMPILE_LANGUAGE:CXX>:-march=native>
        $<$<COMPILE_LANGUAGE:Fortran>:-march=native>
    )
elseif(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE "/arch:AVX2")
endif()

if(COMPILER_SUPPORTS_FAST_MATH)
    target_compile_options(${PROJECT_NAME} PRIVATE 
        $<$<COMPILE_LANGUAGE:CXX>:-ffast-math>
        $<$<COMPILE_LANGUAGE:Fortran>:-ffast-math>
    )
elseif(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE "/fp:fast")
endif()