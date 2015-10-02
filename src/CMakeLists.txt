rock_library(envire_smurf
    SOURCES Robot.cpp
    HEADERS Robot.hpp
    DEPS_PKGCONFIG envire_core smurf)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)

