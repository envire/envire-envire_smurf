#include "Robot.hpp"
#include <iostream>
#include <envire_core/items/Transform.hpp>
#include <base/Time.hpp>
#include <base/Logging.hpp>
#include <envire_core/items/Item.hpp>
#include <configmaps/ConfigData.h>
#include <mars/sim/ConfigMapItem.h>
#include <mars/interfaces/NodeData.h>
#include <mars/utils/Vector.h>


envire::envire_smurf::Robot::Robot(){};

envire::envire_smurf::Robot::Robot(envire::core::Transform pose)
{
    iniPose = pose;
}

void envire::envire_smurf::Robot::loadVisuals(envire::core::TransformGraph &graph)
{
    using visualsItemPtr = envire::core::Item<std::vector<smurf::Visual>>::Ptr;
    std::vector<smurf::Frame *> frames= robot.getFrames();
    for(smurf::Frame* frame : frames)
    {
	visualsItemPtr visuals_itemPtr (new  envire::core::Item<std::vector<smurf::Visual>>(frame->getVisuals()));
	LOG_DEBUG("[envire_smurf::Robot]Added a vector of smurf::Visual to the frame");
        graph.addItemToFrame(frame -> getName(), visuals_itemPtr);
    }
}

void envire::envire_smurf::Robot::loadFrames(envire::core::TransformGraph &graph)
{
    envire::core::FrameId frame_id;
    std::vector<smurf::Frame *> frames= robot.getFrames();
    for(smurf::Frame* frame : frames)
    {
      frame_id = frame->getName();
      graph.addFrame(frame_id);
      LOG_DEBUG_S << "[envire_smurf::Robot]Frame Added: " << frame_id;
    }
}

void envire::envire_smurf::Robot::loadTfs(envire::core::TransformGraph &graph)
{
    using staticTransPtr = boost::shared_ptr<envire::core::Item<smurf::StaticTransformation*  > >;
    // Static Transformations: All transformations are considered static initially
    std::vector<smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    for(std::vector<smurf::StaticTransformation *>::iterator it = staticTfs.begin(); it != staticTfs.end(); ++it) {
        smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = (*it) -> getTransformation();
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);
	// Smurf::StaticTransformation: Not yet used
        //staticTransPtr joint_itemPtr (new  envire::core::Item< smurf::StaticTransformation* > );
        //joint_itemPtr->setData(*it);
        //graph.addItemToFrame(sourceId,joint_itemPtr);
    }
}

void envire::envire_smurf::Robot::loadPhysics(envire::core::TransformGraph& graph)
{
    // Add Physics (The stuff the simulator reacts to) Better outside this method
    using linkItemPtr = envire::core::Item<smurf::Frame>::Ptr;
    envire::core::FrameId frame_id;
    std::vector<smurf::Frame *> frames= robot.getFrames();
    for(smurf::Frame* frame : frames)
    {
	linkItemPtr link_itemPtr (new  envire::core::Item<smurf::Frame>(*frame));
        graph.addItemToFrame(frame -> getName(), link_itemPtr);
	LOG_DEBUG("Added an smurf::frame to the frame");
    }
}

void envire::envire_smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph, const std::string& path, envire::core::vertex_descriptor linkTo)
{
    LOG_DEBUG("[envire_smurf::Robot]loadFromSmurf with a given frame to link to");
    LOG_DEBUG_S << "[envire_smurf::Robot]Transform to linkTo added: " << graph.getFrameId(linkTo) << " and " << rootName;
    // Load the robot
    loadFromSmurf(graph, path); // This one should not add the simulation reactive stuff
    // Create the transform between the linkTo and the robot Root
    envire::core::FrameId frame_id = rootName;
    graph.addTransform(graph.getFrameId(linkTo), rootName, iniPose);
}


void envire::envire_smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph, const std::string& path)
{
    // TODO Glossary
    // Frame: Envire Frame
    // Link: URDF link, which in SMURF are called frames too
    // Should we just not mention the link?
    robot.loadFromSmurf(path);
    // Add Frames (no physical or visual stuff
    loadFrames(graph);
    // Add Transformations
    loadTfs(graph);
    /*
    // Add Sensors 
    using sensorItemPtr = boost::shared_ptr<envire::core::Item< smurf::Sensor*  > >;
    std::vector<smurf::Sensor *> robot_Sensors= robot.getSensors();
    for(std::vector<smurf::Sensor *>::iterator it = robot_Sensors.begin(); it != robot_Sensors.end(); ++it)
    {
        frame_id=(*it)->getattachmentPoint()->getName();
//        std::cout<<"------------------------------------------" <<std::endl;
//        std::cout<<"frame_id: "<<frame_id <<std::endl;
        sensorItemPtr sensor_itemPtr (new  envire::core::Item< smurf::Sensor* > );
        sensor_itemPtr-> setData(*it);
        graph.addItemToFrame(frame_id, sensor_itemPtr);
    }
    */

    /*
    // Static Transformations: All transformations are considered static initially
    std::vector<smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    for(std::vector<smurf::StaticTransformation *>::iterator it = staticTfs.begin(); it != staticTfs.end(); ++it) {
        smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = (*it) -> getTransformation();
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);
        staticTransPtr joint_itemPtr (new  envire::core::Item< smurf::StaticTransformation* > );
        joint_itemPtr->setData(*it);
        graph.addItemToFrame(sourceId,joint_itemPtr);
    }
    return robotRoot;
    */
}
    /*
     * This method creates all the required simulation objects in the envire graph that are required to simulate the robot. The created objects are detected by the envire_physics plugin
     * 
     * Why not generate directly the PhysicsConfigMapItem in the frame in loadFromSmurf? Because you may not want to simulate
     */
void envire::envire_smurf::Robot::simulationReady(envire::core::TransformGraph &graph){
    // Get all the smurf::Frame objects and generate the correspondent PhysicsConfigMapItem in the same frames
    std::vector<smurf::Frame *> frames= robot.getFrames();
    for(std::vector<smurf::Frame *>::iterator it = frames.begin(); it != frames.end(); ++it)
    {
        std::string frame_id = (*it)->getName();
        graph.getFrame(frame_id);
	//mars::sim::PhysicsConfigMapItem simFrame;
	mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);
        graph.addItemToFrame(frame_id, item);
	// What has to go in a PhysicsConfigMap?
    }
    // Get all the smurf::StaticTransformation objects and generate the correspondent JointConfigMapItem in the same frames
    // Get all the smurf::Sensor objecst and generate the correspondet SensorConfigMapItem in the same frames (The SensorConfigMapItem) is not defined yet
}
bool envire::envire_smurf::Robot::frameHas(envire::core::TransformGraph &graph,FRAME_ITEM_TYPE itemType, envire::core::FrameId frameID)
{
    //envire::core::Frame frame=graph.getFrame(frameID);
    using namespace boost;
    bool has_item=false;
    switch (itemType)
    {
        case SENSOR :
        {
//            std::cout << "item is sensor";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<smurf::Sensor*>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<smurf::Sensor*>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;

        }

        case JOINT:
        {
//            std::cout << "item is joint";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<smurf::StaticTransformation*>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<smurf::StaticTransformation*>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;
        }

        case LINK:
        {
//            std::cout << "item is link";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<smurf::Frame *>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<smurf::Frame *>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;
        }


    }

    return has_item;
}

std::vector<envire::core::FrameId>  envire::envire_smurf::Robot::getTransformFrames(envire::core::FrameId &sourceFrame,envire::core::FrameId &targetFrame, envire::core::TransformGraph &graph)
{
    return graph.getPath(sourceFrame, targetFrame);
}
