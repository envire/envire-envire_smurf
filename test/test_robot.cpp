#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>
#include <smurf/Robot.hpp>

//const std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");
//const std::string path = "/home/dfki.uni-bremen.de/rdominguez/Entern/1505/models/robots/spaceclimber_just_two_pieces/smurf/spaceclimber.smurf";

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::smurf::Robot robot;
}

BOOST_AUTO_TEST_CASE(initRobotGraph_noPos)
{
    const std::string path=orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf");
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.initRobotGraph(transformGraph);
    viz.write(transformGraph, "initRobotGraph_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withPos)
{
    const std::string path=orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf");
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initRobotGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withDynamicJoint)
{   
    const std::string path=orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf");
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initRobotGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withDynamicJoint_test.dot");
}

envire::smurf::Robot getRobotWithInitGraph(const std::string path, envire::core::TransformGraph& transformGraph)
{

    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initRobotGraph(transformGraph, transformGraph.getVertex(center));
    return robot;
}

BOOST_AUTO_TEST_CASE(loadFixedJoints)
{
    const std::string path=orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf");
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    robot.loadFixedJoints(transformGraph);
    viz.write(transformGraph, "loadFixedJoints_Test.dot");
}

/*
BOOST_AUTO_TEST_CASE(load_dynamic_tfs)
{
    const std::string path=orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf");
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    robot.initRobotGraph(transformGraph);
    robot.loadDynamicTfs(transformGraph);
    viz.write(transformGraph, "loadDynamicTfsTest.dot");
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



*/


/*

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
*/

/*
BOOST_AUTO_TEST_CASE(load_collidables)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    int nextGroupId = 0;
    //robot.loadCollidables(transformGraph, nextGroupId);
    //viz.write(transformGraph, "loadCollidablesTest.dot");
}

BOOST_AUTO_TEST_CASE(load_physics)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::smurf::Robot robot(path);
    robot.initRobotGraph(transformGraph);
    int nextGroupId = 0;
    robot.loadPhysics(transformGraph, nextGroupId);
    viz.write(transformGraph, "loadPhysicsTest.dot");
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
*/