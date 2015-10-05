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
    // Frames
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    std::cout << "Iterate over the frames:" << std::endl;
    for(std::vector<::smurf::Frame *>::iterator it = frames.begin(); it != frames.end(); ++it) {
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov;
        envire::core::Transform envire_tf(time, tf_cov);
        std::cout << "Include the following frame in the graph: " << (*it)->getName() << std::endl;
        //graph.addFrame(frame->getName());
    }
    // Static Transformations:
    std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    std::cout << "All transformations are considered static initially " << std::endl;
    for(std::vector<::smurf::StaticTransformation *>::iterator it = staticTfs.begin(); it != staticTfs.end(); ++it) {
        ::smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = (*it) -> getTransformation();
        std::cout << "Transformation from " << sourceId <<" to " << targetId << " is " << tf_smurf.matrix() << std::endl;
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);
    }
    std::vector<::smurf::DynamicTransformation *> dynamicTfs = robot.getDynamicTransforms();
    std::cout << "Iterate over the dynamic Tfs" << std::endl;
    for(std::vector<::smurf::DynamicTransformation *>::iterator it = dynamicTfs.begin(); it != dynamicTfs.end(); ++it) {
        ::smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        // GET THE CONFIG MAP OF THE TARGET NODE AND PUT IT IN THE TREE
        // MAYBE WHAT YOU CAN ADD IS THE DYNAMIC TRANSFORMATION
        // Include in the frames the objects from the robot
        std::cout << "The node " << targetId << " is dynamic." << std::endl;
    } 
}
