#pragma once
#include <osgViz/Object.h>
#include <envire_smurf/Collidable.hpp>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgDB/ReadFile> 

namespace vizkit3d
{

class SmurfItemCollidable : public osgviz::Object
{
public:
    SmurfItemCollidable(const std::shared_ptr<envire::smurf::Collidable> collidable)
    {
        switch(collidable->geometry->type)
        {
            case urdf::Geometry::BOX:
                std::cout << "BOX"<< std::endl;
                addBox(collidable);
                break;
            case urdf::Geometry::CYLINDER:
                std::cout << "CYLINDER"<< std::endl;
//             addCylinder(visual, frameId, uuid);
            break;
            case urdf::Geometry::MESH:
                std::cout << "MESH"<< std::endl;
                addMesh(collidable);
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
    
    void addMesh(const std::shared_ptr<envire::smurf::Collidable> collidable)
    {
        boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(collidable->geometry);
        assert(mesh.get() != nullptr);
        std::cout << "MESH: " << mesh->filename << std::endl;
        osg::Node* meshNode = osgDB::readNodeFile(mesh->filename); 
        addChild(meshNode);
    }
    
        


    
    void addBox(const std::shared_ptr<envire::smurf::Collidable> collidable)
    {
        boost::shared_ptr<urdf::Box> urdfBox = boost::dynamic_pointer_cast<urdf::Box>(collidable->geometry);
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
