rock_vizkit_plugin(envire_smurf-viz
    PluginLoader.cpp
    DEPS envire_smurf
    DEPS_PKGCONFIG osgViz 
    MOC 
        EnvireSmurfVisualization.cpp
        EnvireSmurfCollidableVisualization.cpp
    HEADERS 
        EnvireSmurfVisualization.hpp
        EnvireSmurfCollidableVisualization.hpp)

#rock_vizkit_plugin(envire_smurf-collidable-viz
#    EnvireSmurfCollidableVisualization.cpp
#    DEPS envire_smurf
#    DEPS_PKGCONFIG osgViz 
#    MOC EnvireSmurfCollidableVisualization.cpp
#    HEADERS EnvireSmurfCollidableVisualization.hpp)
