#pragma once

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <envire_smurf/Collidable.hpp>

namespace vizkit3d
{
    class EnvireSmurfCollidableVisualization
        : public vizkit3d::Vizkit3DPlugin<envire::smurf::Collidable>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        EnvireSmurfCollidableVisualization();
        ~EnvireSmurfCollidableVisualization();

    Q_INVOKABLE void updateData(envire::smurf::Collidable const &sample)
    {vizkit3d::Vizkit3DPlugin<envire::smurf::Collidable>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(envire::smurf::Collidable const& Collidable);
        
    private:
        struct Data;
        Data* p;
    };
}
