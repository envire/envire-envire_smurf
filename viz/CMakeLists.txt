rock_vizkit_plugin(envire_smurf-viz
    EnvireSmurfVisualization.cpp
    DEPS envire_smurf
    DEPS_PKGCONFIG osgViz 
    MOC EnvireSmurfVisualization.hpp
    HEADERS EnvireSmurfVisualization.hpp)

