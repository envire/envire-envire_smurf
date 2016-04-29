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
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader(transformGraph, path);
    graphLoader.loadStructure();
    viz.write(*transformGraph, "loadStructure_noPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadStructure_withPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center));
    viz.write(*transformGraph, "loadStructure_withPos_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadStructre_withDynamicJoint)
{   
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::GraphViz viz;

    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center));
    viz.write(*transformGraph, "loadStructure_withDynamicJoint_test.dot");
}

envire::smurf::GraphLoader getLoaderWithStructuredGraph(const std::string path)
{
    envire::core::Transform iniPose;
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose, path);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center));
    return graphLoader;
}

BOOST_AUTO_TEST_CASE(loadFrames)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId);
    viz.write(*(graphLoader.getGraph()), "loadFrames_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFrames_withDynamicJoint)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId);
    viz.write(*(graphLoader.getGraph()), "loadFrames_withDynamicJoint_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadFixedJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    graphLoader.loadFixedJoints();
    //NOTE Fixed joints are loaded in the source frame
    viz.write(*(graphLoader.getGraph()), "loadFixedJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadCollidables();
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId);
    graphLoader.loadCollidables();
    viz.write(*(graphLoader.getGraph()), "loadCollidables_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadInertials)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadInertials();
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId);
    graphLoader.loadInertials();
    viz.write(*(graphLoader.getGraph()), "loadInertials_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadInertialsAndCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId);
    graphLoader.loadCollidables();
    graphLoader.loadInertials();
    viz.write(*(graphLoader.getGraph()), "loadInertialsAndCollidables_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadDynamicJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    graphLoader.loadDynamicJoints();
    viz.write(*(graphLoader.getGraph()), "loadDynamicJoints_Test.dot");
}

BOOST_AUTO_TEST_CASE(loadMotors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    envire::core::GraphViz viz;
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(path);
    graphLoader.loadDynamicJoints();
    graphLoader.loadMotors();
    viz.write(*(graphLoader.getGraph()), "loadMotors_Test.dot");
}
