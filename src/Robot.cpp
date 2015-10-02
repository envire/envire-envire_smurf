#include "Robot.hpp"
#include <iostream>

using namespace std;
using namespace envire::smurf;
using namespace envire::core;

void Robot::welcome()
{
    cout << "You successfully compiled and executed the envire_smurf Project. Welcome!" << endl;
}

void Robot::loadFromSmurf(const std::string& path, const envire::core::TransformGraph &graph)
{
  // This method should be very similar to
  // tools/smurf::Robot::loadFromSmurf but shoul populate a envireTransformGraph
  // Other similar methods are to be found the simulation libraries that instantiate objects in the simulation from SMURFS
  // - mars/entity_generation/smurf/src/ which generates SimEntities
  // - mars/sim/src/core/simEntity.h
  //
}
