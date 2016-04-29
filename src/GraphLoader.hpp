#ifndef GRAPHLOADER_H
#define GRAPHLOADER_H
#include <smurf/Robot.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/GraphTypes.hpp>

//TODO Move the Visual struct to another file.

namespace envire
{ 
    namespace core 
    {
        class EnvireGraph;
    }
    
    namespace smurf
    {
            
        class GraphLoader
        {
            
        public: 
            // Constructors
            // NOTE A method to load with respect to a given frame could also be nice
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph): graph(targetGraph){};
            /**
             * Sets iniPose to the provided @param pose
             * 
             */
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, envire::core::Transform pose): graph(targetGraph), iniPose(pose){};
            /**
             *
             * @param path the path to the smurf to be loaded
             */
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, const std::string& path): graph(targetGraph)
            {
                robot.loadFromSmurf(path);
            };
            /**
             *
             * @targetGraph The graph on which the smurf will be loaded
             * @pose The initial pose to set the robot's root
             * @param path the path to the smurf to be loaded
             */
            GraphLoader(const std::shared_ptr<envire::core::EnvireGraph>& targetGraph, envire::core::Transform pose, const std::string& path): graph(targetGraph), iniPose(pose)
            {
                robot.loadFromSmurf(path);
            };
            // Getters and Setters
            std::shared_ptr<envire::core::EnvireGraph> getGraph(){return this->graph;};
            // Other public methods
            /**
             * Creates the frames and transformations of the robot in the
             * transformation graph from the information in the SMURF file 
             * and sets the graph as attribute of the object .
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
            void loadStructure();
            /**
             * Links the robot to the provided vertex with a transformation that
             * corresponst to the iniPose
             * 
             * For this to occur the root frame of the robot must have
             * same name that the constant attribute rootName of this class.
             */
            void loadStructure(envire::core::GraphTraits::vertex_descriptor linkTo);
            /** 
             * Loads in each vertex that corresponds to a link of the robot a
             * smurf::Frame object. This objects trigger in envire_physics the
             * creation of simple nodes that can be used to link connected 
             * structures through fixed joints.
             */
            void loadFrames(int& nextGroupId);
            /**
             * Loads in the vertex from where a static connection to other
             * frames exist a Smurf::StaticTransformation.  To this objects the
             * envire_physics plugin reacts by creating the simulation of a 
             * fixed joint between the connected links. The sourceFrame and the
             * targetFrame are specified in the smurf::StaticTransformation.
             */
            void loadFixedJoints();
            /**
             * Loads the joints from the smurf in the correspondent frames,
             * this is needed to get the dynamic transformations.
             * 
             * Before calling to this method the graph must be initialized.
             */
            void loadDynamicJoints();
            /**
             * Loads the smurf::collidables in the correspondent frames of the 
             * graph. If a collidable position does not correspond to the 
             * owner's link position then a new frame to allocate the 
             * collidable will be created. The new frame will be child of the 
             * link to which belongs according to the smurf/urdf model. 
             * 
             * Two nodes declared in the simulation with the same GroupId are
             * equivalent to two nodes fix-joined, but it is less expensive for
             * the simulator. Thus to keep the colidables moving along with the
             * frame, we set the same groupId of the link to collidables.
             * 
             * If one link has more than one collision object then they will 
             * be grouped in the simulation. In order to do so, the same group 
             * id is assigned for all collidables in the same frame. If one 
             * link has inertial data in the urdf model and collision data 
             * they will alos receive the same groupId.
             * 
             */
            void loadCollidables();
            /** 
             * This method includes in the frames the physical objects that
             * the simulator will react to
             */
            void loadVisuals();
            /**
             * Loads the smurf::inertials in the correspondent frames of the 
             * graph. If an inertial position does not correspond to the 
             * owner's link position then a new frame to allocate the 
             * inertial will be created. The new frame will be child of the 
             * frame that represents the link to which the inertial belongs 
             * according to the smurf/urdf model. 
             * 
             * Two nodes declared in the simulation with the same GroupId are 
             * equivalent to two nodes fix-joined, but it is less expensive 
             * for the simulator. Thus to keep the inertials moving along 
             * with the frame, we set the same groupId of the link to the
             * inertial.
             */
            void loadInertials();
            /**
             * Loads the smurf::Motor objects in the correspondent frame 
             * of the graph.
             * 
             */
            void loadMotors();
            /**
             * Loads the smurf::Sensor objects in the correspondent frame 
             * of the graph.
             * 
             */
            void loadSensors();
            /**
             * Returns the smurf robot which is being loaded in the graph
             */
            ::smurf::Robot getRobot()
            {
                return robot;
            }
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
            void initFrames();
            /** 
             * Loads all the transformations on the graph. For the dynamic
             * transformations an identity transformation is set for the
             * children and the transformation to the parent is set to the
             * father.
             */
            void initTfs();
            /**
             * This method set the transformations that correspond to static 
             * transformations of the robot in the graph.
             */
            void initStaticTfs();
            /**
             * Sets the dynamic transformations from the smurf in the
             * correspondent edges. Some transformations are read from the
             * dynamic joint object.
             * 
             * We do it through the joints of the smurf, because they contain 
             * all the information for the dynamic transfrom. I think that we 
             * don't need any dynamic transform object indeed
             */
            void initDynamicTfs();
            // Attributes
            std::shared_ptr<envire::core::EnvireGraph> graph;
            envire::core::Transform iniPose;
            bool initialized = false;
            const bool debug = false;
            bool framesLoaded = false;
            ::smurf::Robot robot;
        }; // Class
    } // Smurf
} // Envire

#endif // GRAPHLOADER_H
