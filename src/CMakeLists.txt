rock_library(robot
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(robot_bin Main.cpp
    DEPS robot)

