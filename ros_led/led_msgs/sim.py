#!/usr/bin/env python

# Simulator of a LED strip.
# Note this should be run in a terminal supporting 24 bit color.

import sys
import rospy
from led_msgs.msg import LEDState, LEDStateArray
from led_msgs.srv import SetLED, SetLEDs


rospy.init_node('led')
led_count = rospy.get_param('~led_count', 30)
state_pub = rospy.Publisher('~state', LEDStateArray, queue_size=1, latch=True)
# create initial state
state = LEDStateArray([LEDState(index=index) for index in range(led_count)])


def set_led(req):
    state.leds[req.index].r = int(req.r)
    state.leds[req.index].g = int(req.g)
    state.leds[req.index].b = int(req.b)
    print_led()
    state_pub.publish(state)
    return {'success': True}


def set_leds(req):
    for led in req.leds:
        state.leds[led.index].r = int(led.r)
        state.leds[led.index].g = int(led.g)
        state.leds[led.index].b = int(led.b)
    print_led()
    state_pub.publish(state)
    return {'success': True}


rospy.Service('~set_led', SetLED, set_led)
rospy.Service('~set_leds', SetLEDs, set_leds)


def print_led():
    s = ''
    for led in state.leds:
        s += '\033[48;2;{};{};{}m '.format(led.r, led.g, led.b)
    sys.stdout.write('\r{}\033[0m'.format(s))
    sys.stdout.flush()


print_led()
state_pub.publish(state)
rospy.spin()
