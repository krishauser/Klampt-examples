# Make the Klamp't examples 
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4...3.20)
PROJECT(KlamptExamples)

SET(KLAMPT_ROOT ../../Klampt CACHE PATH "Klampt root directory")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
MESSAGE(STATUS "Looking for FindKlampt.cmake in ${CMAKE_MODULE_PATH}")
# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

#IKDemo 
ADD_EXECUTABLE(CartPole cartpole.cpp)
ADD_EXECUTABLE(ContactPlan contactplan.cpp)
#ADD_EXECUTABLE(IKDemo ikdemo.cpp)
ADD_EXECUTABLE(PlanDemo plandemo.cpp)
ADD_EXECUTABLE(DynamicPlanDemo dynamicplandemo.cpp)
ADD_EXECUTABLE(TSDFReconstruction tsdfreconstruction.cpp)
SET(EXAMPLES  CartPole ContactPlan PlanDemo DynamicPlanDemo TSDFReconstruction)
IF(GLUI_FOUND)
  ADD_EXECUTABLE(PlanGUI plangui.cpp)
  ADD_EXECUTABLE(RealTimePlanning realtimeplanning.cpp)
  ADD_EXECUTABLE(SafeSerialClient safeserialclient.cpp)
  ADD_EXECUTABLE(UserTrials usertrials.cpp)
  ADD_EXECUTABLE(UserTrialsSerial usertrials_serial.cpp)
  SET(EXAMPLES ${EXAMPLES} PlanGUI RealTimePlanning SafeSerialClient UserTrials UserTrialsSerial)
ENDIF(GLUI_FOUND)
FOREACH(f ${EXAMPLES})
  TARGET_LINK_LIBRARIES(${f} ${KLAMPT_LIBRARIES})
ENDFOREACH( )

ADD_CUSTOM_TARGET(examples ALL
		DEPENDS ${EXAMPLES})

MESSAGE (STATUS "Compile examples using: make examples OR make [examplename]")
MESSAGE (STATUS "  Possible values for [examplename]:")
FOREACH(f ${EXAMPLES})
  MESSAGE (STATUS "    " ${f})
ENDFOREACH ()
