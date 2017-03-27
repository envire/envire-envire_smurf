#include "Visual.hpp"
#include <envire_core/plugin/Plugin.hpp>

ENVIRE_REGISTER_ITEM(envire::smurf::Visual)

envire::smurf::Visual::Visual()
{}

envire::smurf::Visual::Visual(const urdf::Visual& urdfVisual)
{
    geometry = urdfVisual.geometry;
    material = urdfVisual.material;
    material_name = urdfVisual.material_name;
    name = urdfVisual.name;
    origin = urdfVisual.origin;

}

// FIX. the comparison operator
// add origin comparison
bool  envire::smurf::Visual::operator==(const envire::smurf::Visual& other) const
{
    return other.geometry == geometry &&
           other.material == material &&
           other.material_name == material_name &&
           other.name == name;
}

bool envire::smurf::Visual::operator!=(const envire::smurf::Visual& other) const
{
    return !operator==(other);
}


