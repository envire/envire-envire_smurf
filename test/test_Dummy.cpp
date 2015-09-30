#include <boost/test/unit_test.hpp>
#include <robot/Dummy.hpp>

using namespace robot;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    robot::DummyClass dummy;
    dummy.welcome();
}
