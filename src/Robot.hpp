#ifndef _ROBOTPROJECT_ROBOT_HPP_
#define _ROBOTPROJECT_ROBOT_HPP_
#include <envire_core/graph/TransformGraph.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <smurf/Smurf.hpp>
typedef enum {SENSOR,JOINT,LINK}FRAME_ITEM_TYPE;

namespace envire{ namespace envire_smurf
{
    class Robot
    {
        public: 

            Robot();

	    Robot(envire::core::Transform pose);

	    /**
	     * Adds the frames in the robot as vertex in the graph the @member robot has to have loaded an SMURF document in advance
	     * 
	     * 
	     */
	    void loadFrames(envire::core::TransformGraph &graph);
	    
	    void loadTfs(envire::core::TransformGraph &graph);
	    
	    /**
	     * 
	     * This method includes in the frames the physical objects that the simulator will react to
	     * 
	     */
	    void loadPhysics(envire::core::TransformGraph &graph);
	    
            /**
             * Generates the robot entity in the transformation graph from the
             * information in the SMURF file, it will only contain the static
             * transformations and the information of each link and joint.
	     * 
	     * 
             */
            void loadFromSmurf( envire::core::TransformGraph& graph, const std::string& path);
	    
	    /**
	     * Links the robot to the provided vertex with a dummy transformation
	     * 
	     * The vertex_descriptor that corresponds to the root frame of the robot. For this to occur the root frame of the robot must have same name that the constant attribute rootName of this class.
	     * 
	     */
	    void loadFromSmurf( envire::core::TransformGraph &graph, const std::string &path, envire::core::vertex_descriptor linkTo);
	    
	    /**
	     * Includes in the graph all the necessary information about the robot needed for the physical simulation.
	     * 
	     */
	    void simulationReady(envire::core::TransformGraph &graph);
	    
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
          
	    /**
	     * 
	     */
	    void loadVisuals(envire::core::TransformGraph &graph);
	    
	    smurf::Robot robot;
	    
	private:
	    const envire::core::FrameId rootName = "root";
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
