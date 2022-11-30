# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
project(envire_smurf)
cmake_minimum_required(VERSION 2.6)
set(ROCK_USE_CXX11 TRUE)
find_package(Rock)

#find_package(Poco COMPONENTS Foundation)

find_package(PluginManager)

set(ROCK_TEST_ENABLED ON)
rock_init()
rock_standard_layout()
