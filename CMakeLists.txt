# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
project(envire_smurf)
cmake_minimum_required(VERSION 3.3)

find_package(Rock)

find_package(Poco COMPONENTS Foundation)

find_package(Boost COMPONENTS thread system filesystem program_options)

find_package(PluginManager)

set(ROCK_TEST_ENABLED ON)
rock_init()
rock_standard_layout()
