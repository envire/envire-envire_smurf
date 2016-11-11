#pragma once
#include <osgViz/Object.h>
#include <envire_smurf/Visual.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>

namespace vizkit3d
{

class SmurfItemVisual : public osgviz::Object
{
public:
    SmurfItemVisual(const std::shared_ptr<envire::smurf::Visual> visual)
    {
        switch(visual->geometry->type)
        {
            case urdf::Geometry::BOX:
                addBox(visual);
                break;
//             break;
//             case urdf::Geometry::CYLINDER:
//             addCylinder(visual, frameId, uuid);
//             break;
//             case urdf::Geometry::MESH:
//             addMesh(visual, frameId, uuid);
//             break;
//             case urdf::Geometry::SPHERE:
//             addSphere(visual, frameId, uuid);
//             break;
//             default:
//             LOG_ERROR("[Envire Graphics] ERROR: unknown geometry type");
        }
    }
    
    void addBox(const std::shared_ptr<envire::smurf::Visual> visual)
    {
        std::cout << "ADDING BOX" << std::endl;
        std::cout << "ADDING BOX" << std::endl;
        std::cout << "ADDING BOX" << std::endl;
        std::cout << "ADDING BOX" << std::endl;
        boost::shared_ptr<urdf::Box> urdfBox = boost::dynamic_pointer_cast<urdf::Box>(visual->geometry);
        assert(urdfBox.get() != nullptr);
        osg::Box* box = new osg::Box(osg::Vec3(0,0,0), urdfBox->dim.x, urdfBox->dim.y, urdfBox->dim.z);
        osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box);
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(boxDrawable);
        addChild(geode);
//     ShapeDrawable* boxDrawable = new ShapeDrawable(boundingBox);
//     Geode* geode = new Geode();
//     geode->addDrawable(boxDrawable);
    }
    
};
}