#include "Robot.hpp"
#include <iostream>
#include <base/Time.hpp>
#include <base/Logging.hpp>
#include <mars/interfaces/NodeData.h>
#include <mars/utils/Vector.h>
#include <configmaps/ConfigData.h>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

void envire::smurf::Robot::initGraph(envire::core::TransformGraph &graph)
{
    initFrames(graph);
    initTfs(graph);
    initialized = true;
}

void envire::smurf::Robot::initGraph(envire::core::TransformGraph &graph, envire::core::vertex_descriptor linkTo)
{
    if (debug) {LOG_DEBUG("[Robot::initRobotGraph] LoadFromSmurf with a given frame to link to");}
    if (not initialized)
    {
        initGraph(graph);
    }
    // FIXME This method fails:
    //envire::core::FrameId robotRoot = robot.getRootFrame()->getName(); 
    envire::core::FrameId robotRoot = "root";
    if (debug) {LOG_DEBUG_S << "[Robot::initRobotGraph] Transform to linkTo added: " << graph.getFrameId(linkTo) << " and " << robotRoot;}
    iniPose.time = base::Time::now();
    graph.addTransform(graph.getFrameId(linkTo), robotRoot, iniPose);
}

void envire::smurf::Robot::loadLinks(envire::core::TransformGraph& graph, int& nextGroupId)
{
    using linkItemPtr = envire::core::Item<::smurf::Frame>::Ptr;
    std::vector<::smurf::Frame *> frames = robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        frame->setGroupId(nextGroupId);
        nextGroupId ++;
        linkItemPtr link_itemPtr (new  envire::core::Item<::smurf::Frame>(*frame));
        envire::core::FrameId frameId = frame->getName();
        graph.addItemToFrame(frameId, link_itemPtr);  
        if (debug){ LOG_DEBUG_S << " [Robot::loadLinks] Added an smurf::Frame to the frame *** " << frame->getName() << " ***";}        
    }
    linksLoaded = true;
}

void envire::smurf::Robot::loadFixedJoints(envire::core::TransformGraph &graph)
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
        if (debug) { LOG_DEBUG_S << "[Robot::LoadFixedJoints] Added a new Item< ::smurf::StaticTransformation > to frame *** " + sourceId + "***"; }
    }
}

void envire::smurf::Robot::loadCollidables(envire::core::TransformGraph& graph)
{
    // Without the linksLoaded the id of the frame is not set
    if (linksLoaded)
    {
        using linkItemPtr = envire::core::Item<::smurf::Frame>::Ptr;
        using collidablesVector = std::vector<::smurf::Collidable>;
        using collidableItem = envire::core::Item<::smurf::Collidable>;
        using collidableItemPtr = collidableItem::Ptr;
        std::vector<::smurf::Frame *> frames = robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            int groupId = frame->getGroupId(); // Sharing same GroupId means simulated objects are linked (move together)
            const collidablesVector& collidables  = frame->getCollidables();
            for(::smurf::Collidable collidable : collidables)
            {
                collidable.setGroupId(groupId);
                urdf::Collision collision = collidable.getCollision();
                const base::Vector3d translation(collision.origin.position.x, collision.origin.position.y, collision.origin.position.z); 
                const base::Quaterniond rotation(collision.origin.rotation.w, collision.origin.rotation.x, collision.origin.rotation.y, collision.origin.rotation.z); 
                collidableItemPtr collidable_itemPtr(new collidableItem(collidable));
                //check if the offset is an identity transform
                if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                {
                    //if yes, just add the collision to the existing frame
                    graph.addItemToFrame(frame->getName(), collidable_itemPtr); 
                    if (debug) { LOG_DEBUG_S << "[Robot::loadCollidables] Added a smurf::Collidable to the frame ***" << frame->getName() +" ***";}
                }
                else
                {
                    //otherwise, create a new transformation in the graph to encode the offset
                    base::TransformWithCovariance tfCv(translation, rotation);
                    envire::core::Transform tf(base::Time::now(), tfCv);
                    const envire::core::FrameId collisionFrame(frame->getName()  + "_" + collidable.getName());
                    graph.addTransform(frame->getName(), collisionFrame, tf);
                    graph.addItemToFrame(collisionFrame, collidable_itemPtr);
                    if (debug) {LOG_DEBUG_S << "[Robot::loadCollidables] Added a smurf::Collidable to the frame *** " << collisionFrame << " ***";}
                }
            }
        }
    }
    else
    {
        LOG_ERROR_S << "[Robot::loadCollidables] Robot links are not loaded: You need to call loadLinks before calling loadCollidables";
    }
}

void envire::smurf::Robot::loadInertials(envire::core::TransformGraph& graph)
{
    // Without the linksLoaded the id of the frame is not set
    if (linksLoaded)
    {
        using linkItemPtr = envire::core::Item<::smurf::Frame>::Ptr;
        using inertialItem = envire::core::Item<::smurf::Inertial>;
        using inertialItemPtr = inertialItem::Ptr;
        std::vector<::smurf::Frame *> frames = robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            int groupId = frame->getGroupId();
            if (frame -> getHasInertial())
            {
                ::smurf::Inertial inertialSMURF = frame -> getInertial();
                urdf::Inertial inertial = inertialSMURF.getUrdfInertial();
                inertialSMURF.setGroupId(groupId);
                const base::Vector3d translation(inertial.origin.position.x, inertial.origin.position.y, inertial.origin.position.z); 
                const base::Quaterniond rotation(inertial.origin.rotation.w, inertial.origin.rotation.x, inertial.origin.rotation.y, inertial.origin.rotation.z); 
                inertialItemPtr inertial_itemPtr(new inertialItem(inertialSMURF));
                //check if the offset is an identity transform
                if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                {
                    //if yes, just add the inertial to the existing frame
                    graph.addItemToFrame(frame->getName(), inertial_itemPtr);
                    if (debug) {LOG_DEBUG_S << "[Robot::loadInertials] Added a smurf::Inertial to the frame *** " << frame->getName() << " ***";}
                }
                else
                {
                    //otherwise, create a new transformation in the graph to encode the offset
                    base::TransformWithCovariance tfCv(translation, rotation);
                    envire::core::Transform tf(base::Time::now(), tfCv);
                    const envire::core::FrameId inertialFrame(frame->getName() + "_inertial");
                    graph.addTransform(frame->getName(), inertialFrame, tf);
                    graph.addItemToFrame(inertialFrame, inertial_itemPtr);
                    if (debug) {LOG_DEBUG_S << "[Robot::loadInertials] Added a smurf::Inertial to the frame *** " << inertialFrame << " ***";}
                }
            }
        }
    }
    else
    {
        LOG_ERROR_S << "[Robot::LoadInertials] Robot links are not loaded: You need to call loadLinks before calling loadInertiasls";
    }
}

void envire::smurf::Robot::loadDynamicJoints(envire::core::TransformGraph &graph)
{
    if (initialized)
    {
        using jointsPtr = envire::core::Item<::smurf::Joint>::Ptr ;
        std::vector<::smurf::Joint *> joints= robot.getJoints();
        if (joints.empty())
        {
            if (debug) { LOG_DEBUG_S << "[Robot::loadDynamicJoints] There is no joint in the model";}
        }
        for(::smurf::Joint* joint : joints) 
        {
            envire::core::FrameId frame_id = joint -> getName();
            jointsPtr joint_itemPtr (new envire::core::Item<::smurf::Joint>(*joint));
            graph.addItemToFrame(frame_id, joint_itemPtr);
            if (debug) { LOG_DEBUG_S << "[Robot::loadDynamicJoints] There is a joint in the frame " << joint -> getName() << " from " << joint->getSourceFrame().getName() << " to " << joint->getTargetFrame().getName();}
            if (debug) { LOG_DEBUG_S << "[Robot::loadDynamicJoints] Added a smurf::Joint to the frame ***" << frame_id << "***";}
        }
    }
    else
    {
        LOG_ERROR_S << "[Robot::LoadDynamicJoints] The Robot graph is not initialized, before calling this method initGraph must be called.";
    }
}

/* OLD, DOING MORE THAN JUST SETTING THE TRANSFORMATIONS
void envire::smurf::Robot::initDynamicTfs(envire::core::TransformGraph &graph)
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
            if (debug) { LOG_DEBUG_S << "[Robot::LoadDynamicTfs] No joint given for the dynamic transformation between " << dynamicId << " and " << targetId;}
            parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance::Identity());
        }
        else
        {
            if (debug) { LOG_DEBUG_S << "[Robot::LoadDynamicTfs] Found joint for the dynamic transformation between " << dynamicId << " and " << targetId;}
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
*/





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
        if (debug) { LOG_DEBUG_S << "[Robot::LoadSensors] Attached sensor " << sensor->getName() << " to frame " << frameName;}
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
        if (debug) LOG_DEBUG("[Robot::loadVisuals] Added smurf::Visuals" );
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
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::Sensor*>> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::Sensor*>>(frameID);
            if(begin!=end)
                has_item=true;
            break;

        }

        case JOINT:
        {
//            std::cout << "item is joint";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::StaticTransformation*>> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::StaticTransformation*>>(frameID);
            if(begin!=end)
                has_item=true;
            break;
        }

        case LINK:
        {
//            std::cout << "item is link";
            envire::core::TransformGraph::ItemIterator<envire::core::Item<::smurf::Frame *>> begin, end;
            tie(begin, end) = graph.getItems<envire::core::Item<::smurf::Frame *>>(frameID);
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

// Private
void envire::smurf::Robot::initFrames(envire::core::TransformGraph &graph)
{
    envire::core::FrameId frame_id;
    std::vector<::smurf::Frame *> frames= robot.getFrames();
    for(::smurf::Frame* frame : frames)
    {
        frame_id = frame->getName();
        graph.addFrame(frame_id);
        if (debug) { LOG_DEBUG_S << "[Robot::LoadFrames] Frame Added: " << frame_id;}
    }
    std::vector<::smurf::DynamicTransformation *> dynamicTfs= robot.getDynamicTransforms();
    for(::smurf::DynamicTransformation* dynamicTf : dynamicTfs)
    {
        frame_id = dynamicTf -> getName();
        graph.addFrame(frame_id);
        if (debug) { LOG_DEBUG_S << "[Robot::loadFrames] Frame Added for a dynamic transformation: " << frame_id;}
    }
}

void envire::smurf::Robot::initTfs(envire::core::TransformGraph &graph)
{
    initStaticTfs(graph);
    initDynamicTfs(graph);
}


void envire::smurf::Robot::initStaticTfs(envire::core::TransformGraph &graph)
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

void envire::smurf::Robot::initDynamicTfs(envire::core::TransformGraph &graph)
{
    std::vector<::smurf::Joint *> joints = robot.getJoints();
    for(::smurf::Joint* joint : joints)
    {
        ::smurf::Frame target = joint -> getTargetFrame();
        envire::core::FrameId jointId = joint -> getName();
        // First part: ParentToJointOrigin transformation is set between parent and joint frame
        Eigen::Affine3d parentToJoint = joint->getParentToJointOrigin();
        envire::core::Transform parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance(parentToJoint)); 
        ::smurf::Frame source = joint-> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        graph.addTransform(sourceId, jointId, parent2Joint);
        // Second part: Identity transformation between joint and target frame
        envire::core::FrameId targetId = target.getName();
        envire::core::Transform staticTf(base::Time::now(), base::TransformWithCovariance::Identity());
        graph.addTransform(jointId, targetId, staticTf);
        if (debug) { LOG_DEBUG_S << "[Robot::initDynamicTfs] Transformations between " << sourceId << "and" << jointId << " and " << targetId <<" set.";}
    }
}