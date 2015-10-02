#ifndef _ROBOTPROJECT_ROBOT_HPP_
#define _ROBOTPROJECT_ROBOT_HPP_
#include <envire_core/graph/TransformGraph.hpp>

using namespace envire::core;

namespace envire{ namespace smurf
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

            void loadFromSmurf(const std::string &path, const envire::core::TransformGraph &graph); 
            /**
             * Generates the robot entity in the transformation graph from the
             * information in the SMURF file, it will only contain the static
             * transformations and the information of each link and joint.
             *
             */

            void loadDynamicTransforms(const std::string &path, const envire::core::TransformGraph &graph);
            /**
             *
             * Populates the transformation graph with the dynamic
             * transformations information provided in the file which path is
             * given. The dynamic tranformation include the task that provides
             * it.
             *
             */

          
    };

} // end namespace smurf
} // end namespace envire

#endif // _ROBOTPROJECT_ROBOT_HPP_
