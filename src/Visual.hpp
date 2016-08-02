#ifndef VISUAL_H
#define VISUAL_H
#include "urdf_model/types.h"
#include "urdf_model/link.h"

namespace envire
{
    namespace smurf
    {
        
        /**A replacement for urdf::Visual that hides the visual offset, because
         * in envire the offset is encoded by the structure of the
         * EnvireGraph*/
        struct Visual
        {
            Visual(const urdf::Visual& urdfVisual);
            urdf::GeometrySharedPtr geometry;
            std::string material_name;
            urdf::MaterialSharedPtr material;
            std::string name;
        };
    }
}

#endif // VISUAL_H
