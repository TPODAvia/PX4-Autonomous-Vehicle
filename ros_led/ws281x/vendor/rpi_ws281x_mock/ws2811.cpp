#include "ws2811.h"

#include <ros/console.h>
#include <memory.h>
#include <string.h>

typedef struct ws2811_device
{
    int driver_mode;
    volatile uint8_t *pxl_raw;
    // volatile dma_t *dma;
    volatile pwm_t *pwm;
    // volatile pcm_t *pcm;
    int spi_fd;
    // volatile dma_cb_t *dma_cb;
    uint32_t dma_cb_addr;
    // volatile gpio_t *gpio;
    // volatile cm_clk_t *cm_clk;
    // videocore_mbox_t mbox;
    int max_count;
} ws2811_device_t;

ws2811_return_t ws2811_init(ws2811_t *ws2811)
{
    ROS_INFO("Initializing mock ws2811");
    //memset(ws2811, 0, sizeof(*ws2811));
    //ws2811->rpi_hw = rpi_hw_detect();

    ws2811->device = (ws2811_device_t*)malloc(sizeof(*ws2811->device));
    if (!ws2811->device)
    {
        return ws2811_return_t::WS2811_ERROR_OUT_OF_MEMORY;
    }

    for(int channel = 0; channel < RPI_PWM_CHANNELS; ++channel)
    {
        int ledcnt = ws2811->channel[channel].count;
        if (ledcnt > 0)
        {

            ws2811->channel[channel].leds = (ws2811_led_t*)malloc(ledcnt * sizeof(ws2811_led_t));
            for(int led = 0; led < ledcnt; ++led)
            {
                ws2811->channel[channel].leds[led] = 0;
            }
            ws2811->channel[channel].gamma = (uint8_t*)malloc(256*sizeof(uint8_t));
            for(int i = 0; i < 256; ++i)
            {
                ws2811->channel[channel].gamma[i] = (uint8_t)i;
            }
        }
        else
        {
            ws2811->channel[channel].leds = NULL;
            ws2811->channel[channel].gamma = NULL;
        }
    }
    return ws2811_return_t::WS2811_SUCCESS;
}

void ws2811_fini(ws2811_t *ws2811)
{
    ROS_INFO("Finalizing mock ws2811");
    free(ws2811->device);
    for(int ch = 0; ch < RPI_PWM_CHANNELS; ++ch)
    {
        if (ws2811->channel[ch].count != 0)
        {
            free(ws2811->channel[ch].leds);
            free(ws2811->channel[ch].gamma);
        }
    }
}

ws2811_return_t ws2811_render(ws2811_t *ws2811)
{
    ROS_INFO("Called ws2811_render");
    for(int ch = 0; ch < RPI_PWM_CHANNELS; ++ch)
    {
        int ledcnt = ws2811->channel[ch].count;
        if (ledcnt > 0)
        {
            std::stringstream ledbuf;
            ledbuf << "[";
            for(int led = 0; led < ledcnt; ++led)
            {
                ledbuf << ws2811->channel[ch].leds[led] << ", ";
            }
            ledbuf << "]";

            ROS_INFO_STREAM("Rendering channel 0: " << ledbuf.str());
        }

    }
    return ws2811_return_t::WS2811_SUCCESS;
}

ws2811_return_t ws2811_wait(ws2811_t *ws2811)
{
    ROS_INFO("Called ws2811_wait");
    return ws2811_return_t::WS2811_SUCCESS;
}

const char *ws2811_return_state_strings[] = {
#define WS2811_RETURN_STR(state, name, str) str
        WS2811_RETURN_STATES(WS2811_RETURN_STR)
};

const char * ws2811_get_return_t_str(const ws2811_return_t state)
{
    int idx = -state;
    if (idx < sizeof(ws2811_return_state_strings)/sizeof(ws2811_return_state_strings[0]))
        return ws2811_return_state_strings[idx];
    return "Unknown/unexpected failure state";
}

