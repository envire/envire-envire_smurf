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
        urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
        assert(mesh.get() != nullptr);
        std::cout << "MESH: " << mesh->filename << std::endl;
        osg::Node* meshNode = osgDB::readNodeFile(mesh->filename); 
        addChild(meshNode);
    }
    
        


    
    void addBox(const std::shared_ptr<envire::smurf::Visual> visual)
    {
        urdf::BoxSharedPtr urdfBox = urdf::dynamic_pointer_cast<urdf::Box>(visual->geometry);
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