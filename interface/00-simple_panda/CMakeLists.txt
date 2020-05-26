# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SAI2-EXAMPLES_BINARY_DIR}/00-simple_panda)
ADD_EXECUTABLE (controller00 controller.cpp)
ADD_EXECUTABLE (simviz00 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller00 ${SAI2-EXAMPLES_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz00 ${SAI2-EXAMPLES_COMMON_LIBRARIES})

# copy example interface to output
FILE(COPY 00-simple_panda.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_panda_gripper.urdf panda_arm_hand.urdf spatula.urdf mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
