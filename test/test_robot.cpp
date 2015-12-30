#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>
#include <smurf/Robot.hpp>

const std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const std::string robotPath = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");
BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::smurf::Robot robot;
}

BOOST_AUTO_TEST_CASE(load_frames)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    std::cout << "Load the Frames" << std::endl;
    robot.loadFrames(transformGraph);
    viz.write(transformGraph, "loadFramesTest.dot");
}

BOOST_AUTO_TEST_CASE(load_dynamic_joints)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    std::cout << "Load the Frames" << std::endl;
    robot.loadFrames(transformGraph);
    robot.loadDynamicJoints(transformGraph);
    viz.write(transformGraph, "loadJointsTest.dot");
    smurf::Robot smurfRobot;
    smurfRobot.loadFromSmurf(path);
    std::vector<smurf::Joint*> smurfJoints = smurfRobot.getJoints();
    for(smurf::Joint* joint : smurfJoints) 
    {
        envire::core::FrameId frame_id= joint -> getName();
        envire::core::Item<smurf::Joint>::Ptr itemJoint = transformGraph.getItem<envire::core::Item<smurf::Joint>::Ptr>(frame_id); // Can we avoid the Item twice here?
        smurf::Joint graphJoint = itemJoint->getData();
        std::cout << "[Envire Smurf Test] A joint was obtained from the graph " << graphJoint.getName() << " from " << graphJoint.getSourceFrame().getName() << " to " << graphJoint.getTargetFrame().getName() << std::endl;
    } 
}

BOOST_AUTO_TEST_CASE(load_dynamic_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadDynamicTfs(transformGraph);
    viz.write(transformGraph, "loadDynamicTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_static_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadStaticTfs(transformGraph);
    viz.write(transformGraph, "loadStaticTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadTfs(transformGraph);
    viz.write(transformGraph, "loadTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_from_smurf)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    viz.write(transformGraph, "loadFromSmurfTest.dot");
}
BOOST_AUTO_TEST_CASE(load_collisions)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    robot.loadCollisions(transformGraph);
    viz.write(transformGraph, "loadCollisionsTest.dot");
}
BOOST_AUTO_TEST_CASE(load_collidables)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    robot.loadCollidables(transformGraph);
    viz.write(transformGraph, "loadCollidablesTest.dot");
}
BOOST_AUTO_TEST_CASE(load_visuals)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    robot.loadVisuals(transformGraph);
    viz.write(transformGraph, "loadVisualsTest.dot");
}


BOOST_AUTO_TEST_CASE(load_from_smurf_2)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    envire::core::FrameId id = "NodeToLinkRoot";
    transformGraph.addFrame(id);
    envire::core::vertex_descriptor vertex = transformGraph.getVertex(id);
    robot.loadFromSmurf(transformGraph, vertex);
    viz.write(transformGraph, "loadFromSmurfTest2.dot");
}

BOOST_AUTO_TEST_CASE(load_static_joints)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadStaticJoints(transformGraph);
    viz.write(transformGraph, "loadStaticJointsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_physics)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadPhysics(transformGraph);
    viz.write(transformGraph, "loadPhysicsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_sensors)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadSensors(transformGraph);
    viz.write(transformGraph, "loadSensorsTest.dot");
}