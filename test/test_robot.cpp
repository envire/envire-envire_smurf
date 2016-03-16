#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>
#include <smurf/Robot.hpp>


const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::smurf::Robot robot;
}

BOOST_AUTO_TEST_CASE(initRobotGraph_noPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot(path);
    robot.initGraph(transformGraph);
    viz.write(transformGraph, "initRobotGraph_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(initRobotGraph_withDynamicJoint)
{   
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    viz.write(transformGraph, "initRobotGraph_withDynamicJoint_test.dot");
}

envire::smurf::Robot getRobotWithInitGraph(const std::string path, envire::core::EnvireGraph& transformGraph)
{

    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::Robot robot(iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph.addFrame(center);
    robot.initGraph(transformGraph, transformGraph.getVertex(center));
    return robot;
}

BOOST_AUTO_TEST_CASE(loadLinks)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    int nextGroupId = 0;
    robot.loadLinks(transformGraph, nextGroupId);
    viz.write(transformGraph, "loadLinks_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadLinks_withDynamicJoint)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    int nextGroupId = 0;
    robot.loadLinks(transformGraph, nextGroupId);
    viz.write(transformGraph, "loadLinks_withDynamicJoint_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFixedJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    robot.loadFixedJoints(transformGraph);
    viz.write(transformGraph, "loadFixedJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(load_collidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    // An error  message should appear because the links where not loaded
    robot.loadCollidables(transformGraph);
    int nextGroupId = 0;
    robot.loadLinks(transformGraph, nextGroupId);
    robot.loadCollidables(transformGraph);
    viz.write(transformGraph, "loadCollidablesTest.dot");
}

BOOST_AUTO_TEST_CASE(load_inertials)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    // An error  message should appear because the links where not loaded
    robot.loadInertials(transformGraph);
    int nextGroupId = 0;
    robot.loadLinks(transformGraph, nextGroupId);
    robot.loadInertials(transformGraph);
    viz.write(transformGraph, "loadInertialsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_inertials_and_collidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    // TODO The current solution loads inertials and colllions each one in a separate frame even if they have the same position. Maybe they should be merged?
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    // An error  message should appear because the links where not loaded
    int nextGroupId = 0;
    robot.loadLinks(transformGraph, nextGroupId);
    robot.loadCollidables(transformGraph);
    robot.loadInertials(transformGraph);
    viz.write(transformGraph, "loadInertials_and_collidables_Test.dot");
}

BOOST_AUTO_TEST_CASE(load_dynamic_joints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    robot.loadDynamicJoints(transformGraph);
    viz.write(transformGraph, "loadDynamicJointsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_motors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    envire::core::EnvireGraph transformGraph;
    envire::core::GraphViz viz;
    envire::smurf::Robot robot = getRobotWithInitGraph(path, transformGraph);
    robot.loadDynamicJoints(transformGraph);
    robot.loadMotors(transformGraph);
    viz.write(transformGraph, "loadMotorsTest.dot");
}
