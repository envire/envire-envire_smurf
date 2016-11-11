#ifndef envire_smurf_EnvireSmurfVisualization_H
#define envire_smurf_EnvireSmurfVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <envire_smurf/Visual.hpp>

namespace vizkit3d
{
    class EnvireSmurfVisualization
        : public vizkit3d::Vizkit3DPlugin<envire::smurf::Visual>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        EnvireSmurfVisualization();
        ~EnvireSmurfVisualization();

    Q_INVOKABLE void updateData(envire::smurf::Visual const &sample)
    {vizkit3d::Vizkit3DPlugin<envire::smurf::Visual>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(envire::smurf::Visual const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
