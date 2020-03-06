# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BRACING2D_BINARY_DIR}/04-free_space_2d_bracing)
ADD_EXECUTABLE (app04 app.cpp ${BRACING2D_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (app04 ${BRACING2D_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY 4rbot_fixed.urdf DESTINATION ${APP_RESOURCE_DIR})