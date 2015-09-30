rock_library(envire_smurf
  SOURCES Robot.cpp
  HEADERS Robot.hpp)

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf)

