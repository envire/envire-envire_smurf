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

#include <iostream>
#include <envire_smurf/GraphLoader.hpp>
#include <boost/program_options.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphDrawing.hpp>

void parse_args(int argc, char** argv, std::string& smurf_file, std::string& out)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help",      "Produce help message")
            ;
    //These arguments are positional and thuis should not be show to the user
    po::options_description hidden("Hidden");
    hidden.add_options()
            ("smurf_file", po::value<std::string>(&smurf_file)->required(), "Input SMURF file")
            ("out_dot_file",       po::value<std::string>(&out), "Output DOT file with the resulting information")
            ;

    //Collection of all arguments
    po::options_description all("Allowed options");
    all.add(desc).add(hidden);

    //Collection of only the visible arguments
    po::options_description visible("Allowed options");
    visible.add(desc);

    //Define th positional arguments (they refer to the hidden arguments)
    po::positional_options_description p;
    p.add("smurf_file", 1);
    p.add("out_dot_file", 1);

    //Parse all arguments, but ....
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(all).positional(p).run(), vm);
    //po::notify(vm);

    if (vm.count("help") || argc < 2)
    {
        //... but display only the visible arguments
        std::cout << "smurf_dump allows to dump a parts of a SMURF model, which is usually distributed over multiple files, to a single file or the terminal.\n" << std::endl;
        std::cout << "USAGE: \n\t" << argv[0] << " SMURF_FILE DOT_FILE_OUTPUT\n"<<std::endl;
        desc.print(std::cout);
        exit(EXIT_FAILURE);
    }
    po::notify(vm);
}

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

int main(int argc, char** argv)
{
    std::string smurf_file = "", out_dot_file = "";
    parse_args(argc, argv, smurf_file, out_dot_file);

    // load smurf representation
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(smurf_file);        

    // create graph with init frame CENTER
    std::shared_ptr<envire::core::EnvireGraph> graph(new envire::core::EnvireGraph());
    std::string center_frame = "CENTER";
    graph->addFrame(center_frame);    
    vertex_descriptor center = graph->getVertex(center_frame);

    envire::core::Transform iniPose;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 0.0, 0.0, 0.0;

    int nextGroupID = 1;

    envire::smurf::GraphLoader graphLoader(graph);
    graphLoader.loadRobot(nextGroupID, center, iniPose, *robot);
    envire::core::GraphDrawing::write(*graph, out_dot_file);

    return 0;
}
