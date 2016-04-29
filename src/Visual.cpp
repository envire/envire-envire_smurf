#include "Visual.hpp"

envire::smurf::Visual::Visual(const urdf::Visual& urdfVisual)
{
    geometry = urdfVisual.geometry;
    material = urdfVisual.material;
    material_name = urdfVisual.material_name;
    name = urdfVisual.name;
}

