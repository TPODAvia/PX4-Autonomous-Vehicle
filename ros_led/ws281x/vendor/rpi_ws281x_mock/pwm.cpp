#include "pwm.h"
#include <ros/console.h>

int pwm_pin_alt(int chan, int pinnum)
{
    ROS_INFO_STREAM("pwm_pin_alt called (params: chan == " << chan << ", pinnum == " << pinnum << ")");
    return 42;
}
