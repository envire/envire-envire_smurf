rock_library(envire_smurf
    SOURCES GraphLoader.cpp
    HEADERS GraphLoader.hpp
    DEPS_PKGCONFIG envire_core smurf base-lib urdfdom_headers)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)

# These two libraries are for registering the collidable and visual objects.
# Currently used for visualization only.

rock_library(envire_smurf_collidable_vis
    SOURCES Collidable.cpp
    HEADERS Collidable.hpp
    DEPS_PKGCONFIG envire_smurf smurf)

rock_library(envire_smurf_visual_vis
    SOURCES Visual.cpp
    HEADERS Visual.hpp
    DEPS_PKGCONFIG envire_smurf smurf)


install_plugin_info(envire_smurf_collidable_vis)
install_plugin_info(envire_smurf_visual_vis)

