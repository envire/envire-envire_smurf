rock_library(envire_smurf
    SOURCES GraphLoader.cpp Visual.cpp Collidable.cpp
    HEADERS GraphLoader.hpp Visual.hpp Collidable.hpp
    DEPS_PKGCONFIG envire_core smurf base-lib)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)


install_plugin_info(envire_smurf)
