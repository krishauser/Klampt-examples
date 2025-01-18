# Make the Klamp't examples 
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT(KlamptExamples)

SET(KLAMPT_ROOT ../../../Klampt CACHE PATH "Klampt root directory")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
MESSAGE(STATUS "Looking for FindKlampt.cmake in ${CMAKE_MODULE_PATH}")
# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

ADD_EXECUTABLE(GeometryTest geometrytest.cpp)
ADD_EXECUTABLE(GeometryStats geometrystats.cpp)
ADD_EXECUTABLE(PrimitiveTest primitivetest.cpp)
ADD_EXECUTABLE(HeightmapTest heightmaptest.cpp)
ADD_EXECUTABLE(StabilityTest stabilitytest.cpp)
ADD_EXECUTABLE(WorldIOTest worldiotest.cpp)
SET(TESTS GeometryTest GeometryStats PrimitiveTest HeightmapTest StabilityTest WorldIOTest) 

FOREACH(f ${TESTS})
  TARGET_LINK_LIBRARIES(${f} ${KLAMPT_LIBRARIES})
ENDFOREACH( )

ADD_CUSTOM_TARGET(tests ALL
		DEPENDS ${TESTS})

MESSAGE (STATUS "Compile tests using: make tests OR make [testname]")
MESSAGE (STATUS "  Possible values for [testname]:")
FOREACH(f ${TESTS})
  MESSAGE (STATUS "    " ${f})
ENDFOREACH ()
