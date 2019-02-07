cmake_minimum_required(VERSION 3.9)
project(KillingFusion C CXX)

set(CMAKE_CXX_STANDARD 14)

# https://stackoverflow.com/q/12399422/1874627
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

find_package(PkgConfig REQUIRED)

# Display of Marching Cubes
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

## Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

## OpenCV
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## FreeImage - Compile yourself
include_directories(/home/sk/windows/workspace/Libraries/FreeImage3170/build/include/)
link_directories(/home/sk/windows/workspace/Libraries/FreeImage3170/build/lib/)

## Definitions
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_DIR="${PROJECT_SOURCE_DIR}/data/")
add_definitions(-DOUTPUT_DIR="${PROJECT_SOURCE_DIR}/output/")

# Set files to be compiled
include_directories(${PROJECT_SOURCE_DIR}/include)

set(HEADER_FILES
        include/config.h
        include/utils.h
        include/Timer.h
        include/DatasetReader.h
        include/KillingFusion.h
        include/SDF.h
        include/MarchingCubes.h
        include/DisplacementField.h)

set(SOURCE_FILES
        src/config.cpp
        src/KillingFusion.cpp
        src/DatasetReader.cpp
        src/SDF.cpp
        src/DisplacementField.cpp)

# To Check if in debug mode. Disables OpenMP and printing a lot of Fusion Info.
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMY_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDISABLE_OPENMP")

## Targets
add_executable(KillingFusion src/main.cpp ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries(KillingFusion PRIVATE OpenMP::OpenMP_CXX libfreeimage-3.17.0.so ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

## Targets
add_executable(KillingFusionGui src/mainGui.cpp ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries(KillingFusionGui PRIVATE OpenMP::OpenMP_CXX libfreeimage-3.17.0.so ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()