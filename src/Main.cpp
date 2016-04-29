#include <iostream>
#include <envire_smurf/GraphLoader.hpp>

int main(int argc, char** argv)
{
    std::shared_ptr<envire::core::EnvireGraph> graph;
    envire::smurf::GraphLoader loader(graph);
    return 0;
}
