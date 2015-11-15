#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/items/Item.hpp>

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_created)
{
    envire::envire_smurf::Robot robot;
}

BOOST_AUTO_TEST_CASE(load_asguard_smurf)
{
    envire::envire_smurf::Robot robot;
    std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
    std::cout << "Path to robot model " << path << std::endl;
    envire::core::TransformGraph transformGraph;
    robot.loadFromSmurf(transformGraph, path);
    envire::core::GraphViz viz;
    viz.write(transformGraph, "load_asguard_smurf.dot");

    // The following lines are an example of how to add an item to a frame of the graph, they should be removed from the test ASAP
    std::string item_content = "example";
    boost::shared_ptr<envire::core::Item<std::string> >itemPtr (new  envire::core::Item<std::string> );
    itemPtr -> setData(item_content);
    std::string frame_id = "NewFrame";
    transformGraph.addFrame(frame_id);
    transformGraph.addItemToFrame(frame_id, itemPtr);
    viz.write(transformGraph, "withExtraFrame.dot");
    // The previous lines are an example of how to add an item to a frame of the graph, they should be removed from the test ASAP

    // Load now the dynamic transformations information
    //envire::envire_smurf::Environment environment;
    //environment.loadDynamicTransforms(transformGraph, "<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/envire_smurf/test/dynamicTransformations/simulatedAsguard.yml");
    //viz.write(transformGraph, "simulatedAsguard.dot");


}

BOOST_AUTO_TEST_CASE(get_transform_frames)
{
    envire::envire_smurf::Robot robot;
    std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
    envire::core::TransformGraph transformGraph;
    robot.loadFromSmurf(transformGraph, path);

    envire::core::FrameId sourceFrame("front_left");
    envire::core::FrameId targetFrame("front_right_2");
    std::vector<envire::core::FrameId>  frames_path=robot.getTransformFrames(sourceFrame,targetFrame,transformGraph);
    for(std::vector<envire::core::FrameId>::iterator it=frames_path.begin();it!=frames_path.end();it++)
    {
        std::cout<<*it <<std::endl;
    }

}

BOOST_AUTO_TEST_CASE(test_frameHas)
{
    envire::envire_smurf::Robot robot;
    std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
    envire::core::TransformGraph transformGraph;
    robot.loadFromSmurf(transformGraph, path);
    FRAME_ITEM_TYPE itemType;
    itemType=SENSOR;
    envire::core::FrameId frameID("hokuyo_link");
    if(robot.frameHas(transformGraph,itemType,frameID))
    {
        std::cout<<"The frame " <<frameID<< " has sensor" <<std::endl;
    }
    frameID.clear();
    frameID="velodyne_link";
    if(robot.frameHas(transformGraph,itemType,frameID))
    {

        std::cout<<"The frame " <<frameID<< " has sensor" <<std::endl;

    }

}
