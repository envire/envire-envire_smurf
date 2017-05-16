#include <iostream>
#include "EnvireSmurfCollidableVisualization.hpp"
#include "SmurfItemCollidable.hpp"
#include <memory>
#include <osgViz/Object.h>

using namespace vizkit3d;

struct EnvireSmurfCollidableVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::shared_ptr<smurf::Collidable> data;
    std::shared_ptr<smurf::Collidable> currentCollidable;
};

EnvireSmurfCollidableVisualization::EnvireSmurfCollidableVisualization()
    : p(new Data)
{
}

EnvireSmurfCollidableVisualization::~EnvireSmurfCollidableVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> EnvireSmurfCollidableVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osgviz::Object();
}

void EnvireSmurfCollidableVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    if(p->currentCollidable != p->data)
    {
        p->currentCollidable = p->data;
        osgviz::Object* obj = dynamic_cast<osgviz::Object*>(node);
        if(obj)
        {
            obj->removeChildren(0, obj->getNumChildren());
            obj->addChild(new SmurfItemCollidable(p->data));
        }
    }
}

void EnvireSmurfCollidableVisualization::updateDataIntern(smurf::Collidable const& value)
{
//     std::cout << "UPDATE CALLLED" << std::endl;
    if(p->data && value != (*p->data.get()))
    {
        p->data.reset(new smurf::Collidable(value));
    }
    else if(!p->data)
    {
         p->data.reset(new smurf::Collidable(value));
    }
        
}

//VizkitQtPlugin(EnvireSmurfCollidableVisualization)
