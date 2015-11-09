rock_library(envire_smurf
    SOURCES Robot.cpp
    HEADERS Robot.hpp
    DEPS_PKGCONFIG envire_core smurf mars_interfaces)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)

