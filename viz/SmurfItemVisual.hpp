#pragma once
#include <osgViz/Object.h>
#include <envire_smurf/Visual.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgDB/ReadFile> 

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
                std::cout << "BOX"<< std::endl;
                addBox(visual);
                break;
            case urdf::Geometry::CYLINDER:
                std::cout << "CYLINDER"<< std::endl;
//             addCylinder(visual, frameId, uuid);
            break;
            case urdf::Geometry::MESH:
                std::cout << "MESH"<< std::endl;
                addMesh(visual);
//             addMesh(visual, frameId, uuid);
            break;
            case urdf::Geometry::SPHERE:
                std::cout << "SPHERE"<< std::endl;
//             addSphere(visual, frameId, uuid);
            break;
            default:
                std::cout << "default"<< std::endl;
            break;
        }
    }
    
    void addMesh(const std::shared_ptr<envire::smurf::Visual> visual)
    {
        boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
        assert(mesh.get() != nullptr);
        std::cout << "MESH: " << mesh->filename << std::endl;
        osg::Node* meshNode = osgDB::readNodeFile(mesh->filename); 
        addChild(meshNode);
    }
    
        


    
    void addBox(const std::shared_ptr<envire::smurf::Visual> visual)
    {
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