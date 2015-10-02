#include "Robot.hpp"
#include <iostream>
#include <smurf/Smurf.hpp>
#include <envire_core/items/Transform.hpp>
#include <base/Time.hpp>

//using namespace std;
//using namespace envire::smurf;
//using namespace envire::core;

void envire::smurf::Robot::welcome()
{
    std::cout << "You successfully compiled and executed the envire_smurf Project. Welcome!" << std::endl;
}

void envire::smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph, const std::string& path)
{
  // This method should be very similar to
  // tools/smurf::Robot::loadFromSmurf but shoul populate a envireTransformGraph
  // Other similar methods are to be found the simulation libraries that instantiate objects in the simulation from SMURFS
  // - mars/entity_generation/smurf/src/ which generates SimEntities
  // - mars/sim/src/core/simEntity.h
  //
    ::smurf::Robot robot;
    robot.loadFromSmurf(path);
    // From this object dump into a graph
    std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    std::cout << "Iterate over the static transformations:" << std::endl;
    for(std::vector<::smurf::StaticTransformation *>::iterator it = staticTfs.begin(); it != staticTfs.end(); ++it) {
        ::smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = (*it) -> getTransformation();
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);
    }   
    std::vector<::smurf::Joint *> joints = robot.getJoints();
    std::cout << "Iterate over the joints:" << std::endl;
    for(std::vector<::smurf::Joint *>::iterator it = joints.begin(); it != joints.end(); ++it) {
        std::cout << "There is some joint:" << std::endl;
        ::smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        //This transformation is in the smurf but tools::Smurf seems to drop it (I guess there is a initial value)
        Eigen::Affine3d tf_smurf = (*it) -> getAxisTransformation(); 
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);
    }   
    //std::vector<::smurf::DynamicTransformation *> dynamicTfs= robot.getDynamicTransforms();
    //std::cout << "Iterate over the dynamic transformations:" << std::endl;
    //for(std::vector<::smurf::DynamicTransformation *>::iterator it = dynamicTfs.begin(); it != dynamicTfs.end(); ++it) {
    //    ::smurf::Frame source = (*it) -> getSourceFrame();
    //    envire::core::FrameId sourceId = source.getName();
    //    ::smurf::Frame target = (*it) -> getTargetFrame();
    //    envire::core::FrameId targetId = target.getName();
    //    //This transformation is in the smurf but tools::Smurf seems to drop it (I guess there is a initial value)
    //    Eigen::Affine3d tf_smurf = (*it) -> getTransformation(); 
    //    base::Time time = base::Time::now();
    //    base::TransformWithCovariance tf_cov(tf_smurf);
    //    envire::core::Transform envire_tf(time, tf_cov);
    //    graph.addTransform(sourceId, targetId, envire_tf);
    //}   
}
