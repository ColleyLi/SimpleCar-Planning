cmake_minimum_required(VERSION 2.4)

project(simple_car)

# Set the CMake build type: Release for users
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 support
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

find_package(ompl)
MESSAGE("Your OMPL version: " ${OMPL_VERSION})
find_package(ACADO REQUIRED)

# Set header file directory
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

# Set output library directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Add library of myself
add_library(${PROJECT_NAME} SHARED
  src/SimpleCarPlanning.cpp
  src/ValidityChecker.cpp
)
# Link other libraries if necessary
target_link_libraries(${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  ${OMPLAPP_LIBRARIES}
)

# Set output executable directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exec)

# Add executable files
add_executable(SimpleCarMain src/SimpleCarMain.cpp)
target_link_libraries(SimpleCarMain
  ${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  ${OMPLAPP_LIBRARIES}
)
# add_executable(RigidBodyPlanning 
#   src/RigidBodyPlanning.cpp
# )
# target_link_libraries(RigidBodyPlanning
#   ${OMPL_LIBRARIES}
# )
# add_executable(RigidBodyPlanningWithODESolverAndControls
#   src/RigidBodyPlanningWithODESolverAndControls.cpp
# )
# target_link_libraries(RigidBodyPlanningWithODESolverAndControls
#   ${OMPL_LIBRARIES}
# )
add_executable(MPC src/MPC.cpp)
target_link_libraries(MPC ${ACADO_SHARED_LIBRARIES})