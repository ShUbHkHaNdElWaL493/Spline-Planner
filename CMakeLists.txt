cmake_minimum_required(VERSION 3.10)
project(SPLINE_PLANNER LANGUAGES CXX Fortran)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
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

add_library(SPL INTERFACE)
target_include_directories(SPL INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(SPL INTERFACE Eigen3::Eigen)

add_library(SPLPLANNER INTERFACE)
target_include_directories(SPLPLANNER INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(SPLPLANNER INTERFACE SPL GSL::gsl)

add_library(SPLEXECUTOR INTERFACE)
target_include_directories(SPLEXECUTOR INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(SPLEXECUTOR INTERFACE SPL ur_rtde::rtde)

add_library(SPLVISUALIZER INTERFACE)
target_include_directories(SPLVISUALIZER INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(SPLVISUALIZER INTERFACE SPL)

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp ${FORTRAN_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC gfortran SPLPLANNER SPLVISUALIZER)