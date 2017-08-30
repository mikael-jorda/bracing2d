# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BRACING2D_BINARY_DIR}/01-gravity_compensation)
ADD_EXECUTABLE (simulation01 simulation_main.cpp ${BRACING2D_COMMON_SOURCE})
ADD_EXECUTABLE (visualizer01 visualizer_main.cpp ${BRACING2D_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simulation01 ${BRACING2D_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer01 ${BRACING2D_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BRACING2D_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/01-gravity_compensation)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY 4pbot_floating.urdf DESTINATION ${APP_RESOURCE_DIR})