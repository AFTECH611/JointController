cmake_minimum_required(VERSION 3.22)
message("Configuring Sanpo Controller project")

project(sanpo_controller LANGUAGES CXX C)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wl,--disable-new-dtags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wl,--disable-new-dtags")

add_subdirectory(sanpo_api)
add_subdirectory(example)

# CPack
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "SanpoController")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A brief description of SanpoController: version: v0.1.x")
set(CPACK_PACKAGE_VENDOR "AgibotTech")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Long")

include(CPack)
