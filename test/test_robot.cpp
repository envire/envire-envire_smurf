#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>

const std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const std::string robotPath = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::envire_smurf::Robot robot;
}

BOOST_AUTO_TEST_CASE(load_frames)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::envire_smurf::Robot robot(path);
    std::cout << "Load the Frames" << std::endl;
    robot.loadFrames(transformGraph);
    viz.write(transformGraph, "loadFramesTest.dot");
}

BOOST_AUTO_TEST_CASE(load_dynamic_joints)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::envire_smurf::Robot robot(path);
    std::cout << "Load the Frames" << std::endl;
    robot.loadFrames(transformGraph);
    robot.loadDynamicJoints(transformGraph);
    viz.write(transformGraph, "loadJointsTest.dot");
    
}

BOOST_AUTO_TEST_CASE(load_dynamic_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadDynamicTfs(transformGraph);
    viz.write(transformGraph, "loadDynamicTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_static_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadStaticTfs(transformGraph);
    viz.write(transformGraph, "loadStaticTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_tfs)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadTfs(transformGraph);
    viz.write(transformGraph, "loadTfsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_from_smurf)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::envire_smurf::Robot robot(path);
    robot.loadFromSmurf(transformGraph);
    viz.write(transformGraph, "loadFromSmurfTest.dot");
}

BOOST_AUTO_TEST_CASE(load_from_smurf_2)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    std::cout << "Path to robot model " << path << std::endl;
    envire::envire_smurf::Robot robot(path);
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
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadStaticJoints(transformGraph);
    viz.write(transformGraph, "loadStaticJointsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_physics)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadPhysics(transformGraph);
    viz.write(transformGraph, "loadPhysicsTest.dot");
}

BOOST_AUTO_TEST_CASE(load_sensors)
{
    envire::core::TransformGraph transformGraph;
    envire::core::GraphViz viz;
    envire::envire_smurf::Robot robot(path);
    robot.loadFrames(transformGraph);
    robot.loadSensors(transformGraph);
    viz.write(transformGraph, "loadSensorsTest.dot");
}


//robot.loadJoints(transformGraph);
    //robot.loadDynamicTransformations(transformGraph);
    /*
     *    // The following lines are an example of how to add an item to a frame of the graph, they should be removed from the test ASAP
     *    std::string item_content = "example";
     *    boost::shared_ptr<envire::core::Item<std::string> >itemPtr (new  envire::core::Item<std::string> );
     *    itemPtr -> setData(item_content);
     *    std::string frame_id = "NewFrame";
     *    transformGraph.addFrame(frame_id);
     *    transformGraph.addItemToFrame(frame_id, itemPtr);
     *    viz.write(transformGraph, "withExtraFrame.dot");
     *    // The previous lines are an example of how to add an item to a frame of the graph, they should be removed from the test ASAP
     */
    // Load now the dynamic transformations information
    //envire::envire_smurf::Environment environment;
    //environment.loadDynamicTransforms(transformGraph, "<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/envire_smurf/test/dynamicTransformations/simulatedAsguard.yml");
    //viz.write(transformGraph, "simulatedAsguard.dot");
    
    

BOOST_AUTO_TEST_CASE(get_transform_frames)
{
    //envire::envire_smurf::Robot robot;
    //std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
    //envire::core::TransformGraph transformGraph;
    //robot.loadFromSmurf(transformGraph, path);
    //
    //envire::core::FrameId sourceFrame("front_left");
    //envire::core::FrameId targetFrame("front_right_2");
    //std::vector<envire::core::FrameId>  frames_path=robot.getTransformFrames(sourceFrame,targetFrame,transformGraph);
    //for(std::vector<envire::core::FrameId>::iterator it=frames_path.begin();it!=frames_path.end();it++)
    //{
    //    std::cout<<*it <<std::endl;
    //}
    
}

BOOST_AUTO_TEST_CASE(test_frameHas)
{
    //envire::envire_smurf::Robot robot;
    //std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
    //envire::core::TransformGraph transformGraph;
    //robot.loadFromSmurf(transformGraph, path);
    //FRAME_ITEM_TYPE itemType;
    //itemType=SENSOR;
    //envire::core::FrameId frameID("hokuyo_link");
    //if(robot.frameHas(transformGraph,itemType,frameID))
    //{
    //    std::cout<<"The frame " <<frameID<< " has sensor" <<std::endl;
    //}
    //frameID.clear();
    //frameID="velodyne_link";
    //if(robot.frameHas(transformGraph,itemType,frameID))
    //{
    //
    //    std::cout<<"The frame " <<frameID<< " has sensor" <<std::endl;
    //
    //}
    
}
