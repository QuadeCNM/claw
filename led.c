/**
    * @file led.c
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Implementations and variables for LED control
    * 
    * This file contains the implementations and variables for controlling an LED.
*/

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "led.h"

/*! 
 * @brief LED blink period in milliseconds
 * 
 * This variable can be modified via command interface to change the LED blink rate.
 */
volatile int led_period = LED_DELAY_MS;

/* -------------------------- LED helper functions -----------------------------*/

int pico_led_init(void) 
{
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

void pico_set_led(bool led_on) 
{
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

void process_led_tick(void)
{
    static int led_count = 0;

    if(led_count == 0)
    {
        pico_set_led(true);
    }
    else if(led_count == led_period/2)
    {
        pico_set_led(false);
    }
    else if(led_count >= led_period)
    {
        led_count = -1;
    }
    led_count++;
}
