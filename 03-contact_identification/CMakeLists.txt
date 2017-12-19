# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BRACING2D_BINARY_DIR}/03-contact_identification)
ADD_EXECUTABLE (analytical analytical.cpp ${BRACING2D_COMMON_SOURCE})
ADD_EXECUTABLE (pfilter particle_filter.cpp ${BRACING2D_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (analytical ${BRACING2D_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (pfilter ${BRACING2D_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BRACING2D_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/03-contact_identification)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY 4pbot_fixed.urdf DESTINATION ${APP_RESOURCE_DIR})