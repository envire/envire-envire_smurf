#include "GraphLoader.hpp"
#include "Visual.hpp"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <base/Logging.hpp>

//TODO Instead of having the robot as attribute, receive it as parameter (the groupId will have to be kept in the graph, or we modify the robot)
//TODO Fix the getRoot method of the smurf::robot
//TODO Template the second part of loadCollidables, loadInertials and loadVisuals and put in an auxiliar method because they do the same
//TODO We want to store the dynamic joints in the source frame not in the target. Or is there some reason to put it in the target?
//TODO Put at least an error message or warning: We assume that there is a frame with the same name of the motor! This will fail in multiple cases (e.g. motor name is set to joint name, multiple robots with same frame names...). 
//TODO We assume that there is a frame with the name of the attachment point of the sensor. This will fail in multiple cases (e.g. attachment name does not correspond to any frame, multiple robots with same frame names...) 
//TODO We decided to remove the frame in between for the dynamic transformations, therefore they shall be almost equal to the static ones
//TODO The current implementation loads inertials and colllions each one in a separate frame even if they have the same position. Maybe they should be improved
//TODO Delete the Robot stuff
//TODO Implement a method that loads all at once

// NOTE The dynamic joints are currently loaded in the target frame    
// This does not affect the behavior as long as the simulated motors and joints are linked properly by the plugin. For that they have to be stored in the same frame.
// What are the conventions Joint-Motor-Frame in the SMURF?
// In the smurf the target corresponds normaly to the name of the joint (but there is no enforcement AFAIK)
// The name that is in the motor is the name of the target (also the name of the joint)

namespace envire { namespace smurf {


    void GraphLoader::loadStructure(const ::smurf::Robot& robot)
    {
        initFrames(robot);
        initTfs(robot);
        initialized = true;
    }

    void GraphLoader::loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo, const ::smurf::Robot& robot)
    {
        if (debug) {LOG_DEBUG("[GraphLoader::loadStructure] LoadFromSmurf with a given frame to link to");}
        if (not initialized) { loadStructure(robot); }
        // FIXME This method fails:
        //envire::core::FrameId robotRoot = robot.getRootFrame()->getName(); 
        envire::core::FrameId robotRoot = "root";
        if (debug) {LOG_DEBUG_S << "[GraphLoader::loadStructure] Transform to linkTo added: " << graph->getFrameId(linkTo) << " and " << robotRoot;}
        iniPose.time = base::Time::now();
        graph->addTransform(graph->getFrameId(linkTo), robotRoot, iniPose);
    }

    void GraphLoader::loadFrames(int& nextGroupId, const ::smurf::Robot& robot)
    {
        // NOTE The Frames in smurf correspond to the links in Urdf. The urdf link object is not loaded
        using FrameItemPtr = envire::core::Item<::smurf::Frame>::Ptr;
        std::vector<::smurf::Frame *> frames = robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            frame->setGroupId(nextGroupId);
            nextGroupId ++;
            FrameItemPtr frameItemPtr (new envire::core::Item<::smurf::Frame>(*frame));
            envire::core::FrameId frameId = frame->getName();
            graph->addItemToFrame(frameId, frameItemPtr);  
            if (debug){ LOG_DEBUG_S << " [GraphLoader::loadFrames] Added an smurf::Frame to the frame *" << frame->getName() << "*";}        
        }
        framesLoaded = true;
    }
    
    void GraphLoader::loadFixedJoints(const ::smurf::Robot& robot)
    {
        using StaticTransPtr = boost::shared_ptr<envire::core::Item<::smurf::StaticTransformation  > >;
        std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
        for(::smurf::StaticTransformation* tf : staticTfs) {
            ::smurf::Frame source = tf -> getSourceFrame();
            envire::core::FrameId sourceId = source.getName();
            ::smurf::Frame target = tf -> getTargetFrame();
            envire::core::FrameId targetId = target.getName();
            StaticTransPtr joint_itemPtr (new  envire::core::Item< ::smurf::StaticTransformation > (*tf));
            graph->addItemToFrame(sourceId, joint_itemPtr);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::loadFixedJoints] Added a new Item< ::smurf::StaticTransformation > to frame *" + sourceId + "*"; }
        }
    }
    
    void GraphLoader::loadDynamicJoints(const ::smurf::Robot& robot)
    {
        if (initialized)
        {
            using JointsPtr = envire::core::Item<::smurf::Joint>::Ptr ;
            std::vector<::smurf::Joint *> joints= robot.getJoints();
            if (joints.empty())
            {
                if (debug) { LOG_DEBUG_S << "[GraphLoader::loadDynamicJoints] There is no joint in the model";}
            }
            for(::smurf::Joint* joint : joints) 
            {
                //TODO We want to store the dynamic joints in the source frame not in the target
                if (debug) { LOG_DEBUG_S << "[GraphLoader::loadDynamicJoints] There is a joint with name " << joint -> getName() << " from " << joint->getSourceFrame().getName() << " to " << joint->getTargetFrame().getName();}
                envire::core::FrameId frame_id = joint -> getTargetFrame().getName();
                JointsPtr joint_itemPtr (new envire::core::Item<::smurf::Joint>(*joint));
                graph->addItemToFrame(frame_id, joint_itemPtr);
                if (debug) { LOG_DEBUG_S << "[GraphLoader::loadDynamicJoints] Added a smurf::Joint to the frame *" << frame_id << "*";}
            }
        }
        else
        {
            LOG_ERROR_S << "[GraphLoader::loadDynamicJoints] The Robot graph is not initialized, before calling this method initGraph must be called.";
        }
    }

    void GraphLoader::loadCollidables(const ::smurf::Robot& robot)
    {
        // NOTE Without the Frames Loaded the GroupId of the frames are not set. By sharing same GroupId, the simulated objects are linked (move together)
        if (framesLoaded)
        {
            using CollidablesVector = std::vector<::smurf::Collidable>;
            using CollidableItem = envire::core::Item<::smurf::Collidable>;
            using CollidableItemPtr = CollidableItem::Ptr;
            std::vector<::smurf::Frame *> frames = robot.getFrames();
            for(::smurf::Frame* frame : frames)
            {
                int groupId = frame->getGroupId(); 
                const CollidablesVector& collidables  = frame->getCollidables();
                for(::smurf::Collidable collidable : collidables)
                {
                    collidable.setGroupId(groupId);
                    urdf::Collision collision = collidable.getCollision();
                    const base::Vector3d translation(collision.origin.position.x, collision.origin.position.y, collision.origin.position.z); 
                    const base::Quaterniond rotation(collision.origin.rotation.w, collision.origin.rotation.x, collision.origin.rotation.y, collision.origin.rotation.z); 
                    CollidableItemPtr collidable_itemPtr(new CollidableItem(collidable));
                    //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                    if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                    {
                        graph->addItemToFrame(frame->getName(), collidable_itemPtr); 
                        if (debug) { LOG_DEBUG_S << "[GraphLoader::loadCollidables] Added a smurf::Collidable to the frame *" << frame->getName() +"*";}
                    }
                    else
                    {
                        base::TransformWithCovariance tfCv(translation, rotation);
                        envire::core::Transform tf(base::Time::now(), tfCv);
                        const envire::core::FrameId collisionFrame(frame->getName()  + "_" + collidable.getName());
                        graph->addTransform(frame->getName(), collisionFrame, tf);
                        graph->addItemToFrame(collisionFrame, collidable_itemPtr);
                        if (debug) {LOG_DEBUG_S << "[GraphLoader::loadCollidables] Added a smurf::Collidable to the frame *" << collisionFrame << "*";}
                    }
                }
            }
        }
        else
        {
            LOG_ERROR_S << "[GraphLoader::loadCollidables] Robot frames are not loaded: loadFrames must be executed before loadCollidables";
        }
    }
    void GraphLoader::loadInertials(const ::smurf::Robot& robot)
    {
        // NOTE Without the Frames Loaded the GroupId of the frames are not set. By sharing same GroupId, the simulated objects are linked (move together)
        if (framesLoaded)
        {
            using InertialItem = envire::core::Item<::smurf::Inertial>;
            using InertialItemPtr = InertialItem::Ptr;
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
                    InertialItemPtr inertial_itemPtr(new InertialItem(inertialSMURF));
                    //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                    if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                    {
                        graph->addItemToFrame(frame->getName(), inertial_itemPtr);
                        if (debug) {LOG_DEBUG_S << "[GraphLoader::loadInertials] Added a smurf::Inertial to the frame *" << frame->getName() << "*";}
                    }
                    else
                    {
                        base::TransformWithCovariance tfCv(translation, rotation);
                        envire::core::Transform tf(base::Time::now(), tfCv);
                        const envire::core::FrameId inertialFrame(frame->getName() + "_inertial");
                        graph->addTransform(frame->getName(), inertialFrame, tf);
                        graph->addItemToFrame(inertialFrame, inertial_itemPtr);
                        if (debug) {LOG_DEBUG_S << "[GraphLoader::loadInertials] Added a smurf::Inertial to the frame *" << inertialFrame << "*";}
                    }
                }
            }
        }
        else
        {
            LOG_ERROR_S << "[GraphLoader::LoadInertials] Robot frames are not loaded: loadFrames has to be executed before loadInertiasls";
        }
    }
    
    void GraphLoader::loadVisuals(const ::smurf::Robot& robot)
    {
        using VisualsItemPtr = envire::core::Item<envire::smurf::Visual>::Ptr;
        std::vector<::smurf::Frame *> frames= robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            const std::vector<urdf::Visual>& visuals = frame->getVisuals();
            //NOTE used to create unique frame names for the visuals
            int visualNo = 0;
            for(const urdf::Visual& visual : visuals)
            {
                const base::Vector3d translation(visual.origin.position.x, visual.origin.position.y,                                              visual.origin.position.z);
                const base::Quaterniond rotation(visual.origin.rotation.w, visual.origin.rotation.x,                                                visual.origin.rotation.y, visual.origin.rotation.z);            
                VisualsItemPtr visual_itemPtr(new envire::core::Item<envire::smurf::Visual>(visual));            
                //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                if(translation == base::Vector3d::Zero() && 
                    (rotation.coeffs() == base::Quaterniond::Identity().coeffs() ||
                    rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                {
                    graph->addItemToFrame(frame->getName(), visual_itemPtr);
                }
                else
                {
                    envire::core::Transform tf(translation, rotation);
                    const envire::core::FrameId visualFrame(frame->getName() + "_visual_" + boost::lexical_cast<envire::core::FrameId>(visualNo) );
                    ++visualNo;
                    graph->addTransform(frame->getName(), visualFrame, tf);
                    graph->addItemToFrame(visualFrame, visual_itemPtr);
                }
            }
            if (debug) LOG_DEBUG("[GraphLoader::loadVisuals] Added smurf::Visuals" );
        }
    }
    
    void GraphLoader::loadMotors(smurf::Robot robot)
    {
        using MotorItemPtr = boost::shared_ptr<envire::core::Item< ::smurf::Motor > >;
        std::vector<::smurf::Motor*> motors= robot.getMotors();
        for(::smurf::Motor* motor : motors)
        {
            // TODO We assume that there is a frame with the same name of the motor! This will fail in multiple cases (e.g. motor name is set to joint name, multiple robots with same frame names...) 
            std::string frameName = motor -> getName();
            MotorItemPtr motor_itemPtr (new  envire::core::Item< ::smurf::Motor>(*motor) );
            graph->addItemToFrame(frameName, motor_itemPtr);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::LoadMotors] Attached motor " << motor->getName() << " to frame *" << frameName<<"*";}
        }
    }

    void GraphLoader::loadSensors(smurf::Robot robot)
    {
        using SensorItemPtr = boost::shared_ptr<envire::core::Item< ::smurf::Sensor > >;
        std::vector<::smurf::Sensor*> sensors = robot.getSensors();
        for(::smurf::Sensor* sensor : sensors)
        {
            // TODO We assume that there is a frame with the name of the attachment point of the sensor. This will fail in multiple cases (e.g. attachment name does not correspond to any frame, multiple robots with same frame names...) 
            std::string frameName = sensor->getAttachmentPoint()->getName();
            SensorItemPtr sensor_itemPtr (new  envire::core::Item< ::smurf::Sensor>(*sensor) );
            graph->addItemToFrame(frameName, sensor_itemPtr);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::LoadSensors] Attached sensor " << sensor->getName() << " to frame *" << frameName << "*";}
        }
    }
    
    void GraphLoader::loadRobot(int& nextGroupId, const envire::core::GraphTraits::vertex_descriptor& linkTo, const envire::core::Transform& pose, const ::smurf::Robot& robot)
    {
        // NOTE The only smurf object supported by now are robots, but we should implement it to be compatible with smurfs in general. Will other smurf objects provide joints and so on? maybe yes
        iniPose = pose;
        loadStructure(linkTo, robot);
        loadFrames(nextGroupId, robot);
        loadFixedJoints(robot);
        loadDynamicJoints(robot);
        loadCollidables(robot);
        loadVisuals(robot);
        loadInertials(robot);
        loadMotors(robot);
        loadSensors(robot);
        return nextGroupId;
    }
        
    void GraphLoader::initFrames(const ::smurf::Robot& robot)
    {
        envire::core::FrameId frame_id;
        std::vector<::smurf::Frame *> frames= robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            frame_id = frame->getName();
            graph->addFrame(frame_id);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::initFrames] Frame Added: " << frame_id;}
        }
        std::vector<::smurf::DynamicTransformation *> dynamicTfs= robot.getDynamicTransforms();
        for(::smurf::DynamicTransformation* dynamicTf : dynamicTfs)
        {
            frame_id = dynamicTf -> getName();
            graph->addFrame(frame_id);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::initFrames] Frame Added for a dynamic transformation: " << frame_id;}
        }
    }
    
    void GraphLoader::initTfs(const ::smurf::Robot& robot)
    {
        initStaticTfs(robot);
        initDynamicTfs(robot);
    }
    
    void GraphLoader::initStaticTfs(const ::smurf::Robot& robot)
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
            graph->addTransform(sourceId, targetId, envire_tf);
        }
    }
    
    void GraphLoader::initDynamicTfs(const ::smurf::Robot& robot)
    {
        std::vector<::smurf::Joint *> joints = robot.getJoints();
        for(::smurf::Joint* joint : joints)
        {
            // TODO We decided to remove the frame in between for the dynamic transformations, therefore they shall be almost equal to the static ones
            ::smurf::Frame target = joint -> getTargetFrame();
            envire::core::FrameId jointId = joint -> getName();
            // NOTE First part: ParentToJointOrigin transformation is set between parent and joint frame
            Eigen::Affine3d parentToJoint = joint->getParentToJointOrigin();
            envire::core::Transform parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance(parentToJoint)); 
            ::smurf::Frame source = joint-> getSourceFrame();
            envire::core::FrameId sourceId = source.getName();
            graph->addTransform(sourceId, jointId, parent2Joint);
            // NOTE Second part: Identity transformation between joint and target frame
            envire::core::FrameId targetId = target.getName();
            envire::core::Transform staticTf(base::Time::now(), base::TransformWithCovariance::Identity());
            graph->addTransform(jointId, targetId, staticTf);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::initDynamicTfs] Transformations between " << sourceId << "and" << jointId << " and " << targetId <<" set.";}
        }
    }
            
} } // envire::smurf
