#include <boost/test/unit_test.hpp>
#include <envire_smurf/GraphLoader.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>
#include <smurf/Robot.hpp>

BOOST_AUTO_TEST_CASE(constructor_Test)
{
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::smurf::GraphLoader graphLoader(transformGraph);
}

BOOST_AUTO_TEST_CASE(loadStructure_noPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader(transformGraph);
    graphLoader.loadStructure(*robot);
    viz.write(*transformGraph, "loadStructure_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadStructure_withPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center), (*robot));
    viz.write(*transformGraph, "loadStructure_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadStructre_withDynamicJoint)
{   
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center), (*robot));
    viz.write(*transformGraph, "loadStructure_withDynamicJoint_test.dot");
}

envire::smurf::GraphLoader getLoaderWithStructuredGraph(const smurf::Robot & robot)
{   
    envire::core::Transform iniPose;
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center), robot);
    return graphLoader;
}

BOOST_AUTO_TEST_CASE(loadFrames)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    std::cout << "Initial Group ID: " << nextGroupId << std::endl;
    graphLoader.loadFrames(nextGroupId, *robot);
    std::cout << "Final Group ID: "<< nextGroupId << std::endl;
    viz.write(*(graphLoader.getGraph()), "loadFrames_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFrames_withDynamicJoint)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    viz.write(*(graphLoader.getGraph()), "loadFrames_withDynamicJoint_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFixedJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadFixedJoints(*robot);
    //NOTE Fixed joints are loaded in the source frame
    viz.write(*(graphLoader.getGraph()), "loadFixedJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadCollidables(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadCollidables(*robot);
    viz.write(*(graphLoader.getGraph()), "loadCollidables_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadInertials)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadInertials(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadInertials(*robot);
    viz.write(*(graphLoader.getGraph()), "loadInertials_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadInertialsAndCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadCollidables(*robot);
    graphLoader.loadInertials(*robot);
    viz.write(*(graphLoader.getGraph()), "loadInertialsAndCollidables_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadVisuals)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadVisuals(*robot);
    viz.write(*(graphLoader.getGraph()), "loadVisuals_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadDynamicJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadDynamicJoints(*robot);
    viz.write(*(graphLoader.getGraph()), "loadDynamicJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadMotors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadDynamicJoints(*robot);
    graphLoader.loadMotors(*robot);
    viz.write(*(graphLoader.getGraph()), "loadMotors_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadSensors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_sensor.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadSensors(*robot);
    viz.write(*(graphLoader.getGraph()), "loadSensors_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadRobot)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    std::shared_ptr<envire::core::EnvireGraph> targetGraph(new envire::core::EnvireGraph) ;
    envire::smurf::GraphLoader graphLoader(targetGraph);
    envire::core::FrameId center = "center";
    targetGraph->addFrame(center);
    int nextGroupId = 0;
    graphLoader.loadRobot(nextGroupId, targetGraph->getVertex(center), iniPose, *robot);
    envire::core::GraphViz viz;
    viz.write(*(graphLoader.getGraph()), "loadRobot_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadJoints(*robot);
    viz.write(*(graphLoader.getGraph()), "loadJoints_Test.dot");
}