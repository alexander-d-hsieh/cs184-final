cmake_minimum_required(VERSION 2.8)

# Object simulation source
set(SHATTERSIM_VIEWER_SOURCE
    # Shatter simulation objects
    brittleObject.cpp
    brittleObjectMesh.cpp

    # Collision objects
    collision/sphere.cpp
    collision/plane.cpp

    # Application
    main.cpp
    shatterSimulator.cpp

    # Miscellaneous
    # png.cpp
    misc/sphere_drawing.cpp

    # Camera
    camera.cpp
)

#-------------------------------------------------------------------------------
# Embed resources
#-------------------------------------------------------------------------------

# embed_resources(SHATTERSIM_SHADERS
#   shaders/vert.glsl
#   shaders/normalFrag.glsl
#   shaders/wireframeFrag.glsl
# )

#-------------------------------------------------------------------------------
# Set definitions
#-------------------------------------------------------------------------------
add_definitions(${NANOGUI_EXTRA_DEFS})

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${CGL_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
  ${NANOGUI_EXTRA_INCS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${CGL_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_executable(shattersim ${SHATTERSIM_VIEWER_SOURCE})

target_link_libraries(shattersim
    CGL ${CGL_LIBRARIES}
    nanogui ${NANOGUI_EXTRA_LIBS}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET shattersim APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Put executable in build directory root
set(EXECUTABLE_OUTPUT_PATH ..)

# Install to project root
install(TARGETS shattersim DESTINATION ${ShatterSim_SOURCE_DIR})
