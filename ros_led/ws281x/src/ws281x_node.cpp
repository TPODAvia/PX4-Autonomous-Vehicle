/*
 * ws281x LED strip ROS driver
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Authors: Alexey Rogachevskiy <sfalexrog@gmail.com>, Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <ros/ros.h>

#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>
#include <ws281x/SetGamma.h>

#include <ws2811.h>
#include <ros/console.h>
#include <unordered_map>
#include <signal.h>

constexpr uint32_t LED_RED_SHIFT   = 16;
constexpr uint32_t LED_GREEN_SHIFT = 8;
constexpr uint32_t LED_BLUE_SHIFT  = 0;
constexpr uint32_t LED_WHITE_SHIFT = 24;

constexpr uint32_t LED_RED_MASK   = (0xFF << LED_RED_SHIFT);
constexpr uint32_t LED_GREEN_MASK = (0xFF << LED_GREEN_SHIFT);
constexpr uint32_t LED_BLUE_MASK  = (0xFF << LED_BLUE_SHIFT);
constexpr uint32_t LED_WHITE_MASK = (0xFF << LED_WHITE_SHIFT);

constexpr uint32_t LED_RED   = (0x01 << LED_RED_SHIFT);
constexpr uint32_t LED_GREEN = (0x01 << LED_GREEN_SHIFT);
constexpr uint32_t LED_BLUE  = (0x01 << LED_BLUE_SHIFT);
constexpr uint32_t LED_WHITE = (0x01 << LED_WHITE_SHIFT);



std::unordered_map<std::string, uint64_t> ws2811_types = {
	{"SK6812_STRIP_RGBW", SK6812_STRIP_RGBW},
	{"SK6812_STRIP_RBGW", SK6812_STRIP_RBGW},
	{"SK6812_STRIP_GRBW", SK6812_STRIP_GRBW},
	{"SK6812_STRIP_GBRW", SK6812_STRIP_GBRW},
	{"SK6812_STRIP_BRGW", SK6812_STRIP_BRGW},
	{"SK6812_STRIP_BGRW", SK6812_STRIP_BGRW},
	{"WS2811_STRIP_RGB", WS2811_STRIP_RGB},
	{"WS2811_STRIP_RBG", WS2811_STRIP_RBG},
	{"WS2811_STRIP_GRB", WS2811_STRIP_GRB},
	{"WS2811_STRIP_GBR", WS2811_STRIP_GBR},
	{"WS2811_STRIP_BRG", WS2811_STRIP_BRG},
	{"WS2811_STRIP_BGR", WS2811_STRIP_BGR},
	{"WS2812_STRIP", WS2812_STRIP},
	{"SK6812_STRIP", SK6812_STRIP},
	{"SK6812W_STRIP", SK6812W_STRIP}
};


ws2811_t led_string;

ros::Publisher led_state_pub;
led_msgs::LEDStateArray strip_state;

void publishLedState()
{
	for(size_t i = 0; i < strip_state.leds.size(); ++i) {
		strip_state.leds[i].index = i;
		strip_state.leds[i].r = (led_string.channel[0].leds[i] & LED_RED_MASK) >> LED_RED_SHIFT;
		strip_state.leds[i].g = (led_string.channel[0].leds[i] & LED_GREEN_MASK) >> LED_GREEN_SHIFT;
		strip_state.leds[i].b = (led_string.channel[0].leds[i] & LED_BLUE_MASK) >> LED_BLUE_SHIFT;
		// led_state.w = (led_string.channel[0].leds[i] & LED_WHITE_MASK) >> LED_WHITE_SHIFT;
	}

	led_state_pub.publish(strip_state);
}

bool setGamma(ws281x::SetGamma::Request& req, ws281x::SetGamma::Response& resp)
{
	for(int i = 0; i < 255; ++i) {
		led_string.channel[0].gamma[i] = req.gamma[i];
	}
	resp.success = 1;
	return true;
}

bool setLeds(led_msgs::SetLEDs::Request& req, led_msgs::SetLEDs::Response& resp)
{
	// check validness
	for(auto const& led : req.leds) {
		if (led.index < 0 || led.index >= strip_state.leds.size()) {
			ROS_ERROR("[ws281x] LED index out of bounds: %d", led.index);
			resp.message = "LED index out of bounds: " + std::to_string(led.index);
			return true;
		}
	}

	for(auto const& led : req.leds) {
		auto color = uint32_t(
			LED_RED * int(led.r) +  // Red channel mask
			LED_GREEN * int(led.g) +  // Green channel mask
			LED_BLUE * int(led.b));  // Blue channel mask
		led_string.channel[0].leds[led.index] = color;
	}
	ws2811_return_t ret;
	if ((ret = ws2811_render(&led_string)) != WS2811_SUCCESS) {
		resp.message = ws2811_get_return_t_str(ret);
		ROS_ERROR_THROTTLE(1, "[ws281x] Could not set LED colors: %s", resp.message.c_str());
		resp.success = false;
	} else {
		resp.success = true;
		resp.message = "";
	}
	publishLedState();
	return true;
}

void cleanup(int signal)
{
	(void) signal;
	for(int i = 0; i < led_string.channel[0].count; ++i) {
		led_string.channel[0].leds[i] = 0;
		ws2811_render(&led_string);
	}
	ws2811_fini(&led_string);
	ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ws281x");
	ros::NodeHandle nh, nh_priv("~");

	int param_freq;
	int param_pin;
	int param_dma;
	uint64_t param_strip_type;
	int param_led_count;
	bool param_invert;
	int param_brightness;

	nh_priv.param("target_frequency", param_freq, WS2811_TARGET_FREQ);
	nh_priv.param("gpio_pin", param_pin, 21);
	nh_priv.param("dma", param_dma, 10);

	std::string strip_type_str;
	nh_priv.param("strip_type", strip_type_str, std::string("WS2811_STRIP_GBR"));
	nh_priv.param("led_count", param_led_count, 30);
	nh_priv.param("invert", param_invert, false);
	nh_priv.param("brightness", param_brightness, 255);

	auto strip_type_it = ws2811_types.find(strip_type_str);
	if (strip_type_it != ws2811_types.end()) {
		param_strip_type = strip_type_it->second;
	} else {
		ROS_WARN("[ws281x] Unknown strip type: %s", strip_type_str.c_str());
		param_strip_type = WS2811_STRIP_GBR;
	}

	if (param_freq < 0) {
		ROS_WARN("[ws281x] Target_frequency out of range, resetting to default");
		led_string.freq = (uint32_t)WS2811_TARGET_FREQ;
	} else {
		led_string.freq = (uint32_t)param_freq;
	}

	led_string.dmanum = param_dma;
	led_string.channel[0].gpionum = param_pin;
	led_string.channel[0].count = param_led_count;
	led_string.channel[0].invert = param_invert ? (1) : (0);
	led_string.channel[0].brightness = (uint8_t)param_brightness;
	led_string.channel[0].strip_type = (int)param_strip_type;

	// Disable second channel for now
	led_string.channel[1].gpionum = 0;
	led_string.channel[1].count = 0;
	led_string.channel[1].invert = 0;
	led_string.channel[1].brightness = 0;

	ws2811_return_t ret;
	if ((ret = ws2811_init(&led_string)) != WS2811_SUCCESS) {
		ROS_FATAL("[ws281x] native library init failed: %s", ws2811_get_return_t_str(ret));
		exit(1);
	}
	signal(SIGINT, cleanup);
	signal(SIGTERM, cleanup);

	strip_state.leds.resize(param_led_count);

	auto srv_gamma = nh_priv.advertiseService("set_gamma", setGamma);
	auto srv_leds = nh_priv.advertiseService("set_leds", setLeds);

	led_state_pub = nh_priv.advertise<led_msgs::LEDStateArray>("state", 1, true);
	publishLedState();

	ros::spin();

	return 0;
}
