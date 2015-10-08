rock_testsuite(test_suite suite.cpp
    test_robot.cpp
    DEPS envire_smurf
    DEPS_PKGCONFIG 
        envire_core
        orocos_cpp)
