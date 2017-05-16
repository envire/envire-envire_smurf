Vizkit::UiLoader.register_3d_plugin('EnvireSmurfVisualization', 'envire_smurf', 'EnvireSmurfVisualization')
Vizkit::UiLoader.register_3d_plugin_for('EnvireSmurfVisualization', "/envire/smurf/Visual", :updateData )

Vizkit::UiLoader.register_3d_plugin('EnvireSmurfCollidableVisualization', 'envire_smurf', 'EnvireSmurfCollidableVisualization')
Vizkit::UiLoader.register_3d_plugin_for('EnvireSmurfCollidableVisualization', "envire/core/Item<smurf/Collidable>", :updateData )
