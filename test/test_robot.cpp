#include <boost/test/unit_test.hpp>
#include <envire_smurf/Robot.hpp>

using namespace envire_smurf;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    envire_smurf::Robot robot;
    robot.welcome();
}
