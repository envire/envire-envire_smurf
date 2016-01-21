#ifndef _ENVIRE_SMURF_ROBOT_HPP_
#define _ENVIRE_SMURF_ROBOT_HPP_
#include <envire_core/graph/TransformGraph.hpp>
#include <smurf/Robot.hpp>


// TODO Glossary

namespace envire
{ 
    namespace smurf
    {

        typedef enum {SENSOR,JOINT,LINK}FRAME_ITEM_TYPE;
        
        /**A replacement for urdf::Visual that hides the visual offset, because
         * in envire the offset is encoded by the structure of the
         * TransformGraph*/
        struct Visual
        {
            Visual(const urdf::Visual& urdfVisual);
            boost::shared_ptr<urdf::Geometry> geometry;
            std::string material_name;
            boost::shared_ptr<urdf::Material> material;
            std::string name;
        };
        
        class Robot
        {
        public: 
            //TODO: Shouldn't the Transform Graph be a class member? Seems to
            //be used in every method
            Robot(){};
            /**
             * Sets iniPose to the provided @param pose
             * 
             */
            Robot(envire::core::Transform pose):iniPose(pose){};
            /**
             *
             * @param path the path to the smurf to be loaded
             */
            Robot(const std::string& path)
            {
                robot.loadFromSmurf(path);
            };
            /**
             *
             * @pose The initial pose to set the robot's root
             * @param path the path to the smurf to be loaded
             */
            Robot(envire::core::Transform pose, 
                  const std::string& path):iniPose(pose)
            {
                robot.loadFromSmurf(path);
            };
            /**
             * Creates the frames and transformations of the robot in the
             * transformation graph from the information in the SMURF file.
             * - SMURF::Frames and URDF::Links -> Each generate a Frame in the
             *   Transformation Graph
             * - SMURF::Joints -> Correspond to dynamic transformations. A new
             *   frame, children of the Source of the transformation is created
             *   and father of the target of the transformation (This frame
             *   will later be the one storing the smurf::Joint). The
             *   transformation from the father or source is given by the
             *   parameter parentToJointOrigin of the joint. The transformation
             *   from the joint frame to the target frame is set to the
             *   identity.
             * - SMURF::StaticTransformation -> Generate the Transformation
             *   Graph correspondent transformation.  This method does not load
             *   any of the objects to which the simulator reacts, but is
             *   required before loading any of them.
             */
            void initRobotGraph( envire::core::TransformGraph& graph); 
            /**
             * Links the robot to the provided vertex with a transformation that
             * corresponst to the iniPose
             * 
             * For this to occur the root frame of the robot must have
             * same name that the constant attribute rootName of this class.
             */
            void initRobotGraph( envire::core::TransformGraph &graph, 
                                 envire::core::vertex_descriptor linkTo);
            /**
             * Loads in the vertex from where a static connection to other
             * frames exist a Smurf::StaticTransformation.  To this objects the
             * envire_physics plugin reacts by creating the simulation of a 
             * fixed joint between the connected links. The sourceFrame and the
             * targetFrame are specified in the smurf::StaticTransformation.
             */
            void loadFixedJoints(envire::core::TransformGraph &graph);


            

            
            

            /**
             * Loads the joints from the smurf in the correspondent frames,
             * this is needed to get the dynamic transformations.
             */
            void loadDynamicJoints(envire::core::TransformGraph &graph);
            void loadSensors(envire::core::TransformGraph &graph);
            /** 
             * This method includes in the frames the physical objects that
             * the simulator will react to
             */
            void loadPhysics(envire::core::TransformGraph& graph, 
                             int& nextGroupId);
            /**
             * 
             */
            void loadVisuals(envire::core::TransformGraph &graph);
            /**
             * 
             */
            void loadCollisions(envire::core::TransformGraph& graph);
            /**
             * 
             */
            //void loadCollidables(envire::core::TransformGraph& graph, int& nextGroupId);
            
            /** 
             * Checks if the given frame contains any object of the given
             * @itemType
             * 
             * TODO This method can be removed has the transformationGraph can
             * handle this
             * 
             * @returns true if the frame contains one or more objects of type
             * @itemType
             */
            bool frameHas(envire::core::TransformGraph &graph,
                          FRAME_ITEM_TYPE itemType, 
                          envire::core::FrameId frameID);
            /** 
             * @returns a vector with the FrameIds of the frames between two
             * given frames
             * 
             */
            std::vector<envire::core::FrameId> getTransformFrames(
                envire::core::FrameId &sourceFrame,
                envire::core::FrameId &targetFrame, 
                envire::core::TransformGraph &graph);

            ::smurf::Robot getRobot()
            {
                return robot;
            }
            // Members
            /** 
             * 
             */
            ::smurf::Robot robot;
        private:
          
            /** 
             * Adds the frames of the robot as vertex in the graph the @member
             * robot has to have loaded an SMURF document in advance.  Loads a
             * frame for each link in the smurf model. Additionally a frame is
             * added for each dynamic transformation where the joint will later
             * be stored.  This one is not adding the smurf::Frame object to
             * the frame, neither creating frames for the inertial or
             * collidable objects.
             */
            void initFrames(envire::core::TransformGraph &graph);
            /** 
             * Loads all the transformations on the graph. For the dynamic
             * transformations an identity transformation is set for the
             * children and the transformation to the parent is set to the
             * father.
             */
            void initTfs(envire::core::TransformGraph &graph);
            /**
             * This method set the transformations that correspond to static 
             * transformations of the robot in the graph.
             */
            void initStaticTfs(envire::core::TransformGraph &graph);
            /**
             * Sets the dynamic transformations from the smurf in the
             * correspondent edges. Some transformations are read from the
             * dynamic joint object.
             * 
             * We do it through the joints of the smurf, because they contain 
             * all the information for the dynamic transfrom. I think that we 
             * don't need any dynamic transform object indeed
             */
            void initDynamicTfs(envire::core::TransformGraph &graph);

            
            envire::core::Transform iniPose;
            bool initialized = false;
            const bool debug = true;
            void addJoint(envire::core::TransformGraph& graph, 
                          const ::smurf::Frame& source, 
                          const ::smurf::Frame& target, 
                          const ::smurf::StaticTransformation& smurfStaticTf);
        };
        class Environment
        {
        public:
            
            Environment(){};
            
            void loadFromSmurfs( envire::core::TransformGraph &graph, 
                                 const std::string &path);
            
            /**
             * 
             * Populates the transformation graph with the dynamic
             * transformations information provided in the file which path is
             * given. The dynamic tranformation include the task that provides
             * it.
             *
             */
            void loadDynamicTransforms(
                const envire::core::TransformGraph &graph, 
                const std::string &path);
            
            
        };
        
        
    } // end namespace smurf
} // end namespace envire

#endif // _ENVIRE_SMURF_ROBOT_HPP
