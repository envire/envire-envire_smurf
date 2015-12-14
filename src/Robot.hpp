#ifndef _ROBOTPROJECT_ROBOT_HPP_
#define _ROBOTPROJECT_ROBOT_HPP_
#include <envire_core/graph/TransformGraph.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <smurf/Robot.hpp>
typedef enum {SENSOR,JOINT,LINK}FRAME_ITEM_TYPE;

// TODO Glossary

namespace envire
{ 
    namespace smurf
    {
        
        /**A replacement for urdf::Visual that hides the visual offset, because
         * in envire the offset is encoded by the structure of the TransformGraph*/
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
            Robot();
            /**
             * Sets iniPose to the provided @param pose
             * 
             */
            Robot(envire::core::Transform pose);
            Robot(const std::string& path);
            Robot(envire::core::Transform pose, const std::string& path);
            /**
             * Adds the frames in the robot as vertex in the graph the @member robot has to have loaded an SMURF document in advance.
             * It adds a frame for each Frame in the smurf model and a frame also for each dynamic transformation.
             */
            void loadFrames(envire::core::TransformGraph &graph);
            /**
             * Loads the joints from the smurf in the correspondent frames, this is needed to get the dynamic transformations.
             */
            void loadDynamicJoints(envire::core::TransformGraph &graph);
            /**
             * Loads the dynamic transformations from the smurf in the correspondent edges. Some transformations are read from the dynamic joint object which is stored in the correspondent frame.
             * uses loadDynamicJoints
             */
            void loadDynamicTfs(envire::core::TransformGraph &graph);
            void loadStaticTfs(envire::core::TransformGraph &graph);
            /**
             * Loads all the transformations on the graph. For the dynamic transformations an identity transformation is set for the children and the transformation to the parent is set to the father.
             */
            void loadTfs(envire::core::TransformGraph &graph);
            /**
             * Loads the robot entity in the transformation graph from the
             * information in the SMURF file, it contains:
             * - Static and Dynamic transformations. The dynamic transformations are located after a dynamic joint. As initial value they have the identity
             * - Frames.
             * It will not contain:
             * - Links of the robot. 
             * - Static joints for the simulation of the robot.
             * - Sensors.
             * The components that are required for simulation are loaded in simulationReady
             */
            void loadFromSmurf( envire::core::TransformGraph& graph);
            /**
             * Links the robot to the provided vertex with a dummy transformation
             * 
             * The vertex_descriptor that corresponds to the root frame of the robot. For this to occur the root frame of the robot must have same name that the constant attribute rootName of this class.
             * 
             */
            void loadFromSmurf( envire::core::TransformGraph &graph, envire::core::vertex_descriptor linkTo);
            /**
             * Load in the vertex from where a static connection to other frames exist a Smurf::StaticTransformation.
             * To this objects the simulation reacts by creating the simulation joint between the connected links. The sourceFrame and the targetFrame are included in the tf.
             */
            void loadStaticJoints(envire::core::TransformGraph &graph);
            void loadSensors(envire::core::TransformGraph &graph);
            /**
             * This method includes in the frames the physical objects that the simulator will react to
             */
            void loadPhysics(envire::core::TransformGraph &graph);
            /**
             * 
             */
            void loadVisuals(envire::core::TransformGraph &graph);
            /**
             * 
             */
            void loadCollisions(envire::core::TransformGraph& graph);
            /**
             * Includes in the graph all the necessary information about the robot needed for the physical simulation.
             * Including:
             * - Links of the robot. 
             * - Static joints for the simulation of the robot.
             * - Sensors.
             * 
             */
            void simulationReady(envire::core::TransformGraph &graph);

            
            
            //void loadRotationalJoints(envire::core::TransformGraph &graph);
            //void loadTransationalJoints(envire::core::TransformGraph &graph);
            //void loadDynamicTransformations(envire::core::TransformGraph &graph);
            //void loadDynamicJoints(envire::core::TransformGraph &graph);

            /**
             * Checks if the given frame contains any object of the given @itemType
             * 
             * TODO This method can be removed has the transformationGraph can handle this
             * 
             * @returns true if the frame contains one or more objects of type @itemType
             */
            bool frameHas(envire::core::TransformGraph &graph,FRAME_ITEM_TYPE itemType, envire::core::FrameId frameID);
            /**
             * @returns a vector with the FrameIds of the frames between two given frames
             * 
             */
            std::vector<envire::core::FrameId> getTransformFrames(envire::core::FrameId &sourceFrame,envire::core::FrameId &targetFrame, envire::core::TransformGraph &graph);
            // Members
            /** 
             * 
             */
            ::smurf::Robot robot;
        private:
            envire::core::Transform iniPose;
        };
        class Environment
        {
        public:
            
            Environment(){};
            
            void loadFromSmurfs( envire::core::TransformGraph &graph, const std::string &path);
            
            /**
             * 
             * Populates the transformation graph with the dynamic
             * transformations information provided in the file which path is
             * given. The dynamic tranformation include the task that provides
             * it.
             *
             */
            void loadDynamicTransforms(const envire::core::TransformGraph &graph, const std::string &path);
            
            
        };
        
        
    } // end namespace smurf
} // end namespace envire

#endif // _ROBOTPROJECT_ROBOT_HPP_
