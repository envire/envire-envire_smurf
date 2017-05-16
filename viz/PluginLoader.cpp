#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "EnvireSmurfVisualization.hpp"
#include "EnvireSmurfCollidableVisualization.hpp"

namespace vizkit3d {
    class QtPluginVizkitSmurf : public vizkit3d::VizkitPluginFactory
    {
    public:

        QtPluginVizkitSmurf() {}

        /**
        * Returns a list of all available visualization plugins.
        * @return list of plugin names
        */
        virtual QStringList* getAvailablePlugins() const
        {
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("SmurfVisualization");
            pluginNames->push_back("SmurfCollidableVisualization");
            return pluginNames;
        }

        virtual QObject* createPlugin(QString const& pluginName)
        {
            vizkit3d::VizPluginBase* plugin = 0;
            if (pluginName == "SmurfVisualization")
            {
                plugin = new vizkit3d::EnvireSmurfVisualization();
            }
            else if (pluginName == "SmurfCollidableVisualization")
            {
                plugin = new vizkit3d::EnvireSmurfCollidableVisualization();
            }

            if (plugin)
            {
                    return plugin;
            }
            return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkitSmurf, QtPluginVizkitSmurf)
}
