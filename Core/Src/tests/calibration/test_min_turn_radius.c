#include "tests/calibration/test_min_turn_radius.h"
#include "control/turn_control.h"

void Test_MinTurnRadius(void)
{
    Turn_RunMinRadiusTestBlocking();
}
