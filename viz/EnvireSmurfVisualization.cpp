#include <iostream>
#include "EnvireSmurfVisualization.hpp"
#include "SmurfItemVisual.hpp"
#include <memory>
#include <osgViz/Object.h>

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
    return new osgviz::Object();
}

void EnvireSmurfVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    if(p->currentVisual != p->data)
    {
        p->currentVisual = p->data;
        osgviz::Object* obj = dynamic_cast<osgviz::Object*>(node);
        if(obj)
        {
            obj->removeChildren(0, obj->getNumChildren());
            obj->addChild(new SmurfItemVisual(p->data));
        }
    }
}

void EnvireSmurfVisualization::updateDataIntern(envire::smurf::Visual const& value)
{
//     std::cout << "UPDATE CALLLED" << std::endl;
    if(p->data && value != (*p->data.get()))
    {
        p->data.reset(new envire::smurf::Visual(value));
    }
    else if(!p->data)
    {
         p->data.reset(new envire::smurf::Visual(value));
    }
        
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvireSmurfVisualization)

