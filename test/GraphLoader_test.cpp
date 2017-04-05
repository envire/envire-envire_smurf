//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include <boost/test/unit_test.hpp>
#include <envire_smurf/GraphLoader.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphDrawing.hpp>
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
    envire::smurf::GraphLoader graphLoader(transformGraph);
    graphLoader.loadStructure(*robot);
    envire::core::GraphDrawing::writeSVG(*transformGraph, "loadStructure_noPos_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadStructure_withPos)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center), (*robot));
    envire::core::GraphDrawing::writeSVG(*transformGraph, "loadStructure_withPos_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadStructre_withDynamicJoint)
{   
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    std::shared_ptr<envire::core::EnvireGraph> transformGraph(new envire::core::EnvireGraph) ;
    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(transformGraph, iniPose);
    envire::core::FrameId center = "center";
    transformGraph->addFrame(center);
    graphLoader.loadStructure(transformGraph->getVertex(center), (*robot));
    
    envire::core::GraphDrawing::writeSVG(*transformGraph, "loadStructure_withDynamicJoint_test.svg");
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
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    std::cout << "Initial Group ID: " << nextGroupId << std::endl;
    graphLoader.loadFrames(nextGroupId, *robot);
    std::cout << "Final Group ID: "<< nextGroupId << std::endl;
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadFrames_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadFrames_withDynamicJoint)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadFrames_withDynamicJoint_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadFixedJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadFixedJoints(*robot);
    //NOTE Fixed joints are loaded in the source frame
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadFixedJoints_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadCollidables(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadCollidables(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadCollidables_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadInertials)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    std::cout << "An error  message should appear because the Frames where not loaded " << path << std::endl;
    graphLoader.loadInertials(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadInertials(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadInertials_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadInertialsAndCollidables)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadCollidables(*robot);
    graphLoader.loadInertials(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadInertialsAndCollidables_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadVisuals)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadVisuals(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadVisuals_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadDynamicJoints)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadDynamicJoints(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadDynamicJoints_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadMotors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadDynamicJoints(*robot);
    graphLoader.loadMotors(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadMotors_Test.svg");
}

BOOST_AUTO_TEST_CASE(loadSensors)
{
    const std::string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_sensor.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::smurf::GraphLoader graphLoader = getLoaderWithStructuredGraph(*robot);
    graphLoader.loadSensors(*robot);
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadSensors_Test.svg");
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
    envire::core::GraphDrawing::writeSVG(*(graphLoader.getGraph()), "loadRobot_Test.svg");
}