//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "GraphLoader.hpp"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <base-logging/Logging.hpp>
#include <boost/lexical_cast.hpp>

//NOTE We store the dynamic joints in the source frame. The name in the smurf for the motor has to be the source of the corresponden dynamic joint, otherwise it won't be found by envire_motors. How is the naming done in Phobos? I guess that the target is the name and therefore it will not work. Motors should have source and target in their description.
//TODO 1. Introduce a prefix to be able to load multiple robots of the same model or with same naming for the frames
//TODO 2. The current implementation loads inertials, visuals and colllions each one in a separate frame even if they have the same position. This should be improved
//TODO 3. Template the second part of loadCollidables, loadInertials and loadVisuals and put in an auxiliar method because they do the same

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
        const envire::core::FrameId robotRoot = robot.getRootFrame()->getName();
        iniPose.time = base::Time::now();
        graph->addTransform(graph->getFrameId(linkTo), robotRoot, iniPose);
        if (debug) {LOG_DEBUG_S << "[GraphLoader::loadStructure] Transform to linkTo added: " << graph->getFrameId(linkTo) << " and " << robotRoot;}
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
        using StaticTransPtr = envire::core::Item<::smurf::StaticTransformation  >::Ptr;
        std::vector<::smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
        for(::smurf::StaticTransformation* tf : staticTfs) {
            ::smurf::Frame source = tf->getSourceFrame();
            envire::core::FrameId sourceId = source.getName();
            ::smurf::Frame target = tf->getTargetFrame();
            envire::core::FrameId targetId = target.getName();
            StaticTransPtr joint_itemPtr (new  envire::core::Item< ::smurf::StaticTransformation > (*tf));
            graph->addItemToFrame(sourceId, joint_itemPtr);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::loadFixedJoints] Added a new Item< ::smurf::StaticTransformation > with name *" + tf->getName() +  "* to frame *" + sourceId + "*"; }
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
                envire::core::FrameId frame_id = joint -> getSourceFrame().getName();
                JointsPtr joint_itemPtr (new envire::core::Item<::smurf::Joint>(*joint));
                graph->addItemToFrame(frame_id, joint_itemPtr);
                if (debug) { LOG_DEBUG_S << "[GraphLoader::loadDynamicJoints] Added a smurf::Joint with name *" << joint -> getName() << "* to frame *" << frame_id << "*";}
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
                int collisionNo = 0;
                const CollidablesVector& collidables  = frame->getCollidables();
                for(::smurf::Collidable collidable : collidables)
                {
                    collidable.groupId = groupId;
                    CollidableItemPtr collidable_itemPtr(new CollidableItem(collidable));
                    //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                    //if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                    //{
                    //    graph->addItemToFrame(frame->getName(), collidable_itemPtr);
                    //    if (debug) { LOG_DEBUG_S << "[GraphLoader::loadCollidables] Added a smurf::Collidable to the frame *" << frame->getName() +"*";}
                    //}
                    //else
                    //{
                    base::TransformWithCovariance tfCv(collidable.origin.position, collidable.origin.orientation);
                    envire::core::Transform tf(base::Time::now(), tfCv);
                    envire::core::FrameId collisionFrame;
                    if (collidable.name.empty())
                    {
                        collisionFrame = envire::core::FrameId(frame->getName() + "_collision_" + boost::lexical_cast<envire::core::FrameId>(collisionNo) );
                        ++collisionNo;
                    } else
                        collisionFrame = envire::core::FrameId(frame->getName()  + "_" + collidable.name);

                    graph->addTransform(frame->getName(), collisionFrame, tf);
                    graph->addItemToFrame(collisionFrame, collidable_itemPtr);
                    if (debug) {LOG_DEBUG_S << "[GraphLoader::loadCollidables] Added a smurf::Collidable to the frame *" << collisionFrame << "*";}
                    //}
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
                    //if(translation == base::Vector3d::Zero() && (rotation.coeffs() == base::Quaterniond::Identity().coeffs() || rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                    //{
                    //    graph->addItemToFrame(frame->getName(), inertial_itemPtr);
                    //    if (debug) {LOG_DEBUG_S << "[GraphLoader::loadInertials] Added a smurf::Inertial to the frame *" << frame->getName() << "*";}
                    //}
                    //else
                    //{
                    base::TransformWithCovariance tfCv(translation, rotation);
                    envire::core::Transform tf(base::Time::now(), tfCv);
                    const envire::core::FrameId inertialFrame(frame->getName() + "_inertial");
                    graph->addTransform(frame->getName(), inertialFrame, tf);
                    graph->addItemToFrame(inertialFrame, inertial_itemPtr);
                    if (debug) {LOG_DEBUG_S << "[GraphLoader::loadInertials] Added a smurf::Inertial to the frame *" << inertialFrame << "*";}
                    //}
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
        using VisualsItemPtr = envire::core::Item<::smurf::Visual>::Ptr;
        std::vector<::smurf::Frame *> frames= robot.getFrames();
        for(::smurf::Frame* frame : frames)
        {
            const std::vector<::smurf::Visual>& visuals = frame->getVisuals();

            //NOTE used to create unique frame names for the visuals
            int visualNo = 0;
            int groupId = frame->getGroupId();
            for(const ::smurf::Visual& visual : visuals)
            {
                VisualsItemPtr visual_itemPtr(new envire::core::Item<::smurf::Visual>(visual));
                visual_itemPtr->getData().groupId = groupId;
                //NOTE Checks if the offset is an identity transform. If yes, just add the collision to the existing frame otherwise, create a new transformation in the graph to encode the offset.
                //if(translation == base::Vector3d::Zero() &&
                //    (rotation.coeffs() == base::Quaterniond::Identity().coeffs() ||
                //    rotation.coeffs() == -base::Quaterniond::Identity().coeffs()))
                //{
                //    graph->addItemToFrame(frame->getName(), visual_itemPtr);
                //}
                //else
                //{
                base::TransformWithCovariance tfCv(visual.origin.position, visual.origin.orientation);
                envire::core::Transform tf(base::Time::now(), tfCv);
                envire::core::FrameId visualFrame;
                if (visual.name.empty())
                {
                    visualFrame = envire::core::FrameId(frame->getName() + "_visual_" + boost::lexical_cast<envire::core::FrameId>(visualNo) );
                    ++visualNo;
                } else
                    visualFrame = envire::core::FrameId(frame->getName()  + "_" + visual.name);

                graph->addTransform(frame->getName(), visualFrame, tf);
                graph->addItemToFrame(visualFrame, visual_itemPtr);
                //}
            }
            if (debug) LOG_DEBUG("[GraphLoader::loadVisuals] Added smurf::Visuals" );
        }
    }

    void GraphLoader::loadMotors(const ::smurf::Robot& robot)
    {
        using MotorItemPtr = envire::core::Item< ::smurf::Motor >::Ptr;
        std::vector<::smurf::Motor*> motors= robot.getMotors();
        std::vector<::smurf::Joint*> joints = robot.getJoints();
        for(::smurf::Motor* motor : motors)
        {
            configmaps::ConfigMap motorMap = motor -> getMotorMap();
            std::string jointName = static_cast<std::string>(motorMap["joint"]);
            std::string frameName = "";
            size_t i = 0;
            while ((frameName == "") && (i<joints.size()))
            {
                if (joints[i]->getName() == jointName)
                {
                    frameName = joints[i]->getSourceFrame().getName();
                }
                i+=1;
            }
            if (graph->containsFrame(frameName))
            {
                MotorItemPtr motor_itemPtr (new  envire::core::Item< ::smurf::Motor>(*motor) );
                graph->addItemToFrame(frameName, motor_itemPtr);
                if (debug) { LOG_DEBUG_S << "[GraphLoader::LoadMotors] Attached motor " << motor->getName() << " to frame *" << frameName<<"*";}
            }
            else
            {
                LOG_ERROR_S << "[GraphLoader::LoadMotors] The motor "<< motor->getName() << " does not have a joint with the same target/child name. Therefore, the motor is not added into the graph.";
            }
        }
    }

    void GraphLoader::loadSensors(const ::smurf::Robot& robot)
    {
        using SensorItemPtr = envire::core::Item< ::smurf::Sensor >::Ptr;
        std::vector<::smurf::Sensor*> sensors = robot.getSensors();
        for(::smurf::Sensor* sensor : sensors)
        {
            std::string frameName = sensor->getAttachmentPoint()->getName();
            SensorItemPtr sensor_itemPtr (new  envire::core::Item< ::smurf::Sensor>(*sensor) );
            if (graph->containsFrame(frameName))
            {
                graph->addItemToFrame(frameName, sensor_itemPtr);
                if (debug) { LOG_DEBUG_S << "[GraphLoader::LoadSensors] Attached sensor " << sensor->getName() << " to frame *" << frameName << "*";}
            }
            else
            {
                LOG_ERROR_S << "[GraphLoader::LoadSensors] The specified frame for the sensor "<< sensor->getName() << " does not exist. A frame (urdf link) with the same name of the sensor attachemnt's point is missing (link field). Therefore, the sensor is not added into the graph.";
            }
        }
    }

    void GraphLoader::loadRobot(int& nextGroupId, const envire::core::GraphTraits::vertex_descriptor& linkTo, const envire::core::Transform& pose, const ::smurf::Robot& robot)
    {
        // NOTE The only smurf object supported by now are robots, but we should implement it to be compatible with smurfs in general. Will other smurf objects provide joints and so on? maybe yes
        // TODO The order in which the elements are loaded in the graph can be problematic, somehow the commented out order produces a strange behavior in which the fixed joints pull together the objects joined
        /*
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
        */
        iniPose = pose;
        loadStructure(linkTo, robot);
        loadFrames(nextGroupId, robot);
        loadCollidables(robot);
        loadInertials(robot);
        loadFixedJoints(robot);
        loadDynamicJoints(robot);
        loadVisuals(robot);
        loadMotors(robot);
        loadSensors(robot);
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
    }

    void GraphLoader::initTfs(const ::smurf::Robot& robot)
    {
        initStaticTfs(robot);
        initDynamicTfs(robot);
    }

    void GraphLoader::initStaticTfs(const ::smurf::Robot& robot)
    {
        using staticTransPtr = envire::core::Item<::smurf::StaticTransformation*  >::Ptr;
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
            ::smurf::Frame target = joint -> getTargetFrame();
            // NOTE ParentToJointOrigin transformation is set between parent and child frame
            Eigen::Affine3d parentToJoint = joint->getParentToJointOrigin();
            envire::core::Transform parent2Joint = envire::core::Transform(base::Time::now(), base::TransformWithCovariance(parentToJoint));
            ::smurf::Frame source = joint-> getSourceFrame();
            envire::core::FrameId sourceId = source.getName();
            envire::core::FrameId targetId = target.getName();
            graph->addTransform(sourceId, targetId, parent2Joint);
            if (debug) { LOG_DEBUG_S << "[GraphLoader::initDynamicTfs] Transformation between " << sourceId << " and " << targetId <<" set.";}
        }
    }

} } // envire::smurf
