#include "Collidable.hpp"
#include <envire_core/plugin/Plugin.hpp>

ENVIRE_REGISTER_ITEM(envire::smurf::Collidable)

envire::smurf::Collidable::Collidable()
{}

envire::smurf::Collidable::Collidable(const urdf::Collision& urdfCollision)
{
    geometry = urdfCollision.geometry;
    name = urdfCollision.name;
}

bool  envire::smurf::Collidable::operator==(const envire::smurf::Collidable& other) const
{
    return other.geometry == geometry &&
           other.name == name;
}

bool envire::smurf::Collidable::operator!=(const envire::smurf::Collidable& other) const
{
    return !operator==(other);
}

