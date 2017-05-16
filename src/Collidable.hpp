#ifndef COLLIDABLE_H
#define COLLIDABLE_H
#include "urdf_model/types.h"
#include "urdf_model/link.h"
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
namespace envire
{
    namespace smurf
    {
        struct Collidable
        {
            Collidable();
            Collidable(const urdf::Collision& urdfCollision);
            urdf::GeometrySharedPtr geometry;
            std::string name;

            bool operator==(const Collidable& other) const;
            bool operator!=(const Collidable& other) const;

        /**Grants access to boost serialization */
        friend class boost::serialization::access;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Envire::smurf::Collidable::serialize not implemented");
        }
        
        };
    }
}
#endif // COLLIDABLE_H
