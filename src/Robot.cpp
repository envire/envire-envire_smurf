#include "Robot.hpp"
#include <iostream>
#include <base/Time.hpp>
#include <base/Logging.hpp>
#include <mars/interfaces/NodeData.h>
#include <mars/utils/Vector.h>
#include <configmaps/ConfigData.h>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

void envire::smurf::Robot::loadFrames(envire::core::TransformGraph &graph)
{
    // Load the frame of each frame in the smurf model
    LOG_DEBUG_S << "[envire_smurf::Robot] LoadFrames start";
    envire::core::FrameId frame_id;
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        frame_id = frame->getName();
        graph.addFrame(frame_id);
        LOG_DEBUG_S << "[envire_smurf::Robot] Frame Added: " << frame_id;
    }
    // Load the frame of each dynamic transformation in the smurf model
    std::vector<::smurf::DynamicTransformation *> dynamicTfs= robot.getDynamicTransforms();
    for(::smurf::DynamicTransformation* dynamicTf : dynamicTfs)
    {
        frame_id = dynamicTf -> getName();
        graph.addFrame(frame_id);
        LOG_DEBUG_S << "[envire_smurf::Robot] Frame Added for a dynamic transformation: " << frame_id;
    }
}

void envire::smurf::Robot::loadDynamicJoints(envire::core::TransformGraph &graph)
{
    using jointsPtr = envire::core::Item<::smurf::Joint>::Ptr ;
    std::vector<::smurf::Joint *> joints= robot.getJoints();
    if (joints.empty())
    {
        LOG_DEBUG_S << "[Envire Smurf] There is no joint in the model";
    }
    for(::smurf::Joint* joint : joints) 
    {
        //::smurf::Frame jointFrame = joint->getName();
        envire::core::FrameId frame_id= joint -> getName();
        jointsPtr joint_itemPtr (new envire::core::Item<::smurf::Joint>(*joint));
        graph.addItemToFrame(frame_id, joint_itemPtr);
        LOG_DEBUG_S << "[Envire Smurf] There is a joint in the frame " << joint -> getName() << " from " << joint->getSourceFrame().getName() << " to " << joint->getTargetFrame().getName();
    } 
}

void envire::smurf::Robot::loadDynamicTfs(envire::core::TransformGraph &graph)
{
    // Joints need to be loaded to the graph
    loadDynamicJoints(graph);
    using dynamicTransPtr = boost::shared_ptr<envire::core::Item<::smurf::DynamicTransformation  > >;
    std::vector<::smurf::DynamicTransformation *> dynamicTfs= robot.getDynamicTransforms();
    for(::smurf::DynamicTransformation* dynamicTf : dynamicTfs)
    {
        // First part: identity transformation between dynamic joint and child (Target)
        ::smurf::Frame target = dynamicTf-> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        envire::core::FrameId dynamicId = dynamicTf -> getName();
        envire::core::Transform staticTf(base::Time::now(), base::TransformWithCovariance::Identity());
        graph.addTransform(dynamicId, targetId, staticTf);
        // Second part: getParentToJointOrigin transformation is set between parent and dynamic joint
        using Iterator = envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::Joint>::Ptr>;
        Iterator begin, end;
        boost::tie(begin, end) = graph.getItems<envire::core::Item<::smurf::Joint>::Ptr>(dynamicId);
        envire::core::Transform parent2Joint;
        if (begin == end)
        {
            LOG_DEBUG_S << "[Envire Smurf] No joint given for the dynamic transformation between " << dynamicId << " and " << targetId;
            parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance::Identity());
        }
        else
        {
            LOG_DEBUG_S << "[Envire Smurf] Found joint for the dynamic transformation between " << dynamicId << " and " << targetId;
            ::smurf::Joint joint = (*begin)->getData();
            Eigen::Affine3d parentToJoint = joint.getParentToJointOrigin();
            // If it is named parent to joint it should go from the parent to the joint origin and not after the joint
            parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance(parentToJoint)); 
        }
        ::smurf::Frame source = dynamicTf-> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        graph.addTransform(sourceId, dynamicId, parent2Joint);
    }
}

void envire::smurf::Robot::loadStaticTfs(envire::core::TransformGraph &graph)
{
    using staticTransPtr = boost::shared_ptr<envire::core::Item<::smurf::StaticTransformation*  > >;
    std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    for(::smurf::StaticTransformation* tf : staticTfs) {
        ::smurf::Frame source = tf -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = tf -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = tf -> getTransformation();
        envire::core::Transform envire_tf(base::Time::now(), base::TransformWithCovariance(tf_smurf));
        graph.addTransform(sourceId, targetId, envire_tf);
    }

}

void envire::smurf::Robot::loadTfs(envire::core::TransformGraph &graph)
{
    loadDynamicTfs(graph);
    loadStaticTfs(graph);
}

void envire::smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph)
{
    loadFrames(graph);
    loadTfs(graph);
}

void envire::smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph, envire::core::vertex_descriptor linkTo)
{
    LOG_DEBUG("[envire_smurf::Robot]loadFromSmurf with a given frame to link to");
    loadFromSmurf(graph);
    // Create the transform between the linkTo and the robot Root
    //envire::core::FrameId robotRoot = robot.getRootFrame()->getName(); // FIXME This method fails
    envire::core::FrameId robotRoot = "root";
    LOG_DEBUG_S << "[envire_smurf::Robot]Transform to linkTo added: " << graph.getFrameId(linkTo) << " and " << robotRoot;
    iniPose.time = base::Time::now();
    graph.addTransform(graph.getFrameId(linkTo), robotRoot, iniPose);
}

void envire::smurf::Robot::loadStaticJoints(envire::core::TransformGraph &graph)
{
    // TODO: Visualize the joints on the simulator visualizer (envire_graphics)
    using staticTransPtr = boost::shared_ptr<envire::core::Item<::smurf::StaticTransformation  > >;
    std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    for(::smurf::StaticTransformation* tf : staticTfs) {
        ::smurf::Frame source = tf -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        ::smurf::Frame target = tf -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        staticTransPtr joint_itemPtr (new  envire::core::Item< ::smurf::StaticTransformation > (*tf));
        graph.addItemToFrame(sourceId, joint_itemPtr);
        LOG_DEBUG_S << "[Envire Smurf] Added a new Item< ::smurf::StaticTransformation >";
    }
}

void envire::smurf::Robot::loadSensors(envire::core::TransformGraph &graph)
{
    // Add Sensors 
    using sensorItemPtr = boost::shared_ptr<envire::core::Item< ::smurf::Sensor > >;
    std::vector<::smurf::Sensor*> sensors= robot.getSensors();
    for(::smurf::Sensor* sensor : sensors)
    {
        std::string frameName = sensor->getAttachmentPoint()->getName();
        sensorItemPtr sensor_itemPtr (new  envire::core::Item< ::smurf::Sensor>(*sensor) );
        graph.addItemToFrame(frameName, sensor_itemPtr);
        LOG_DEBUG_S << "[Envire SMURF] Attached sensor " << sensor->getName() << " to frame " << frameName;
    }
}

// TODO: [Refactor] This and loadVisuals are very similar, only the naming differs

void envire::smurf::Robot::loadCollisions(envire::core::TransformGraph& graph)
{
    robot.loadCollisions();
    // Add Physic objects of which the simulator generated simple objects
    using collisionsVector = std::vector<urdf::Collision>;
    using collisionItem = envire::core::Item<urdf::Collision>;
    using collisionsItemPtr = collisionItem::Ptr;
    envire::core::FrameId frame_id;
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        const collisionsVector& collisions = frame->getCollisions();
        for(urdf::Collision collision : collisions)
        {
            const base::Vector3d translation(collision.origin.position.x, collision.origin.position.y, collision.origin.position.z); 
            const base::Quaterniond rotation(collision.origin.rotation.w, collision.origin.rotation.x, collision.origin.rotation.y, collision.origin.rotation.z); 
            collisionsItemPtr collision_itemPtr(new collisionItem(collision));
            //check if the offset is an identity transform
            if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
            {
                //if yes, just add the visual to the existing frame
                graph.addItemToFrame(frame->getName(), collision_itemPtr);
                LOG_DEBUG_S << "[envire::smurf::loadCollidables] Added an urdf::collision to the frame " << frame->getName();
            }
            else
            {
                //otherwise, create a new transformation in the graph to encode the offset
                base::TransformWithCovariance tfCv(translation, rotation);
                envire::core::Transform tf(base::Time::now(), tfCv);
                const envire::core::FrameId collisionFrame(frame->getName() + "_collision_" + boost::lexical_cast<envire::core::FrameId>(collision.name));
                graph.addTransform(frame->getName(), collisionFrame, tf);
                graph.addItemToFrame(collisionFrame, collision_itemPtr);
                LOG_DEBUG_S << "[envire::smurf::loadCollidables] Added an urdf::collision to the frame " << collisionFrame;
            }
        }
    }
}

void envire::smurf::Robot::loadPhysics(envire::core::TransformGraph& graph)
{
    // Add Physic objects of which the simulator generated simple objects
    using linkItemPtr = envire::core::Item<::smurf::Frame>::Ptr;
    envire::core::FrameId frame_id;
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        linkItemPtr link_itemPtr (new  envire::core::Item<::smurf::Frame>(*frame));
        graph.addItemToFrame(frame -> getName(), link_itemPtr);
        LOG_DEBUG_S << "Added an smurf::frame to the frame" << frame->getName();
    }
}
void envire::smurf::Robot::loadVisuals(envire::core::TransformGraph &graph)
{
    using VisualsItemPtr = envire::core::Item<envire::smurf::Visual>::Ptr;
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        const std::vector<urdf::Visual>& visuals = frame->getVisuals();
        int visualNo = 0;//used to create unique frame names for the visuals
        for(const urdf::Visual& visual : visuals)
        {
            const base::Vector3d translation(visual.origin.position.x, visual.origin.position.y, 
                                             visual.origin.position.z);
            const base::Quaterniond rotation(visual.origin.rotation.w, visual.origin.rotation.x,
                                             visual.origin.rotation.y, visual.origin.rotation.z);
            
            VisualsItemPtr visual_itemPtr(new envire::core::Item<envire::smurf::Visual>(visual));
            
            //check if the visual offset is an identity transform
            if(translation == base::Vector3d::Zero() && 
               (rotation.coeffs() == base::Quaterniond::Identity().coeffs() ||
               rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
            {
                //if yes, just add the visual to the existing frame
                graph.addItemToFrame(frame->getName(), visual_itemPtr);
            }
            else
            {
                //otherwise, create a new transformation in the graph to encode the visual offset
                envire::core::Transform tf(translation, rotation);
                const envire::core::FrameId visualFrame(frame->getName() + "_visual_" + boost::lexical_cast<envire::core::FrameId>(visualNo) );
                ++visualNo;
                graph.addTransform(frame->getName(), visualFrame, tf);
                graph.addItemToFrame(visualFrame, visual_itemPtr);
            }
        }
        LOG_DEBUG("[envire_smurf::Robot]Added smurf::Visuals to the frame");
    }
}

bool envire::smurf::Robot::frameHas(envire::core::TransformGraph &graph,FRAME_ITEM_TYPE itemType, envire::core::FrameId frameID)
{
    //envire::core::Frame frame=graph.getFrame(frameID);
    using namespace boost;
    bool has_item=false;
    switch (itemType)
    {
        case SENSOR :
        {
//            std::cout << "item is sensor";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::Sensor*>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::Sensor*>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;

        }

        case JOINT:
        {
//            std::cout << "item is joint";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::StaticTransformation*>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::StaticTransformation*>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;
        }

        case LINK:
        {
//            std::cout << "item is link";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::Frame *>::Ptr> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::Frame *>::Ptr>(frameID);
            if(begin!=end)
                has_item=true;
            break;
        }


    }

    return has_item;
}

std::vector<envire::core::FrameId>  envire::smurf::Robot::getTransformFrames(envire::core::FrameId &sourceFrame,envire::core::FrameId &targetFrame, envire::core::TransformGraph &graph)
{
    return graph.getPath(sourceFrame, targetFrame);
}

envire::smurf::Visual::Visual(const urdf::Visual& urdfVisual)
{
    geometry = urdfVisual.geometry;
    material = urdfVisual.material;
    material_name = urdfVisual.material_name;
    name = urdfVisual.name;
}

