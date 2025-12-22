/**
    * @file led.h
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Definitions, function definitions and variables for LED control
    * 
    * This file contains the definitions, function definitions and variables for controlling an LED.
*/

#ifndef LED_H
#define LED_H

// Define default LED delay if not defined
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

/*! 
 * @brief LED blink period in milliseconds
 * 
 * This variable can be modified via command interface to change the LED blink rate.
 */
extern volatile int led_period;

/*!
 * @brief Initialise the LED
 *
 * @param: none
 * @return: PICO_OK on success, error code on failure
 */
int pico_led_init(void);

/*!
 * @brief Turn the LED on or off
 *
 * @param led_on: true to turn on, false to turn off
 * @return: none
 */
void pico_set_led(bool led_on);



/*!
 * @brief Process LED timing tick
 *
 * @param: none
 * @return: none 
 */
void process_led_tick(void);

#endif