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

