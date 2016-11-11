#include <iostream>
#include "EnvireSmurfVisualization.hpp"

using namespace vizkit3d;

struct EnvireSmurfVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    envire::smurf::Visual data;
};


EnvireSmurfVisualization::EnvireSmurfVisualization()
    : p(new Data)
{
}

EnvireSmurfVisualization::~EnvireSmurfVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> EnvireSmurfVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void EnvireSmurfVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void EnvireSmurfVisualization::updateDataIntern(envire::smurf::Visual const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvireSmurfVisualization)

