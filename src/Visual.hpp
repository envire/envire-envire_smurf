#ifndef VISUAL_H
#define VISUAL_H
#include "urdf_model/types.h"
#include "urdf_model/link.h"
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
namespace envire
{
    namespace smurf
    {
        
        /**A replacement for urdf::Visual that hides the visual offset, because
         * in envire the offset is encoded by the structure of the
         * EnvireGraph*/
        struct Visual
        {
            Visual();
            Visual(const urdf::Visual& urdfVisual);
            urdf::GeometrySharedPtr geometry;
            std::string material_name;
            urdf::MaterialSharedPtr material;
            std::string name;
            
            bool operator==(const Visual& other) const;
            bool operator!=(const Visual& other) const;
            
        /**Grants access to boost serialization */
        friend class boost::serialization::access;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Envire::smurf::Visual::serialize not implemented");
        }
        
        };
    }
}

#endif // VISUAL_H
