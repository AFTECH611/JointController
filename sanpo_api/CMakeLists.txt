# ================================
# Get current folder name (optional)
# ================================
string(REGEX REPLACE ".*/(.*)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ================================
# Target name
# ================================
set(CUR_TARGET_NAME sanpo_controller)

# ================================
# File collection
# ================================
file(GLOB_RECURSE HEAD_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# ================================
# Add library
# ================================
add_library(${CUR_TARGET_NAME} STATIC
    ${SRC_FILES}
)

# ================================
# Include directories
# ================================
target_include_directories(${CUR_TARGET_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${CUR_TARGET_NAME}>
)

# ================================
# Install library
# ================================
install(
    TARGETS ${CUR_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# ================================
# Install headers
# ================================
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/${CUR_TARGET_NAME}
    PATTERN "internal" EXCLUDE
)
