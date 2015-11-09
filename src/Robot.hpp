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
            /**
             * Print a welcome to stdout
             * \return nothing
             */
            void welcome();

            Robot(){};

            /**
             * Generates the robot entity in the transformation graph from the
             * information in the SMURF file, it will only contain the static
             * transformations and the information of each link and joint.
             *
             */
            void loadFromSmurf( envire::core::TransformGraph &graph, const std::string &path);
	    void simulationReady(envire::core::TransformGraph &graph);
            bool frameHas(envire::core::TransformGraph &graph,FRAME_ITEM_TYPE itemType, envire::core::FrameId frameID);
            std::vector<envire::core::FrameId> getTransformFrames(envire::core::FrameId &sourceFrame,envire::core::FrameId &targetFrame, envire::core::TransformGraph &graph);
          
	    smurf::Robot robot;
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
