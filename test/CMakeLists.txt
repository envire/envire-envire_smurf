find_package(Boost)

rock_testsuite(test_suite_graphLoader GraphLoader_test.cpp suite.cpp
    DEPS envire_smurf
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG 
        envire_core)

target_link_libraries(test_suite_graphLoader ${Boost_LIBRARIES})

file(COPY sample_smurfs DESTINATION ./)
