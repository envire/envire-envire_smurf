rock_library(envire_smurf
    SOURCES GraphLoader.cpp
            FrameRegistration.cpp
            InertialRegistration.cpp
            VisualRegistration.cpp
            CollidableRegistration.cpp
            StaticTransformationRegistration.cpp
            JointRegistration.cpp
            MotorRegistration.cpp
            SensorRegistration.cpp
    HEADERS GraphLoader.hpp
    DEPS_PKGCONFIG envire_core smurf base-lib urdfdom_headers)


#find_package(Boost COMPONENTS thread system filesystem)
find_package( Boost COMPONENTS program_options thread system filesystem REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

rock_executable(envire_smurf_bin Main.cpp
    DEPS envire_smurf
    DEPS_PKGCONFIG
        envire_core)

target_link_libraries(envire_smurf_bin ${Boost_LIBRARIES})

install_plugin_info(envire_smurf)
