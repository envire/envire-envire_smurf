rock_library(envire_smurf
    SOURCES GraphLoader.cpp
            VisualRegistration.cpp
            CollidableRegistration.cpp
    HEADERS GraphLoader.hpp
    DEPS_PKGCONFIG envire_core smurf base-lib urdfdom_headers)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)

install_plugin_info(envire_smurf)
