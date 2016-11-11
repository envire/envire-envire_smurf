#include <iostream>
#include "EnvireSmurfVisualization.hpp"
#include "SmurfItemVisual.hpp"
#include <memory>

using namespace vizkit3d;

struct EnvireSmurfVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::shared_ptr<envire::smurf::Visual> data;
    std::shared_ptr<envire::smurf::Visual> currentVisual;
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
    if(p->currentVisual != p->data)
    {
        p->currentVisual = p->data;
        osg::Group* group = static_cast<osg::Group*>(node);
        group->removeChildren(0, group->getNumChildren());
        group->addChild(new SmurfItemVisual(p->data));
    }
}

void EnvireSmurfVisualization::updateDataIntern(envire::smurf::Visual const& value)
{
    if(p->data && value != (*p->data.get()))
    {
        p->data.reset(new envire::smurf::Visual(value));
    }
        
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvireSmurfVisualization)

