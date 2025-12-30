/**
    * @file claw.c
    * @author Jon Wade
    * @date  19 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Firmware to control a claw device with stepper motors via USB serial commands.
    * 
    * Command interface for controlling stepper motors and other functions associated with the claw device.
    * This file implements a simple command interface over USB serial to control the claw device.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <string.h>
#include "pico/assert.h"
#include <stdlib.h>
#include "hardware/gpio.h"
#include <ctype.h>
#include <math.h>

#include "sys_timer.h"
#include "stepper.h"
#include "led.h"
#include "command_processor.h"

/*!
 * @brief Main function
 * @param: none
 * @return: none    
 */ 
int main()
{
    char* cmd;
    stepper_state_t stepper;

    // Initialise the LED and stdio
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    stdio_init_all();
    stepper_init(&stepper, 0, DEFAULT_STEPPER_PERIOD);
    
    // Set up repeating timer
    struct repeating_timer timer;

    // Wait for USB serial connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Set up a repeating timer to count milliseconds
    add_repeating_timer_us(TIMER_INTERVAL_US, timer_callback, NULL, &timer);

    // Clear the screen and print welcome message 
    puts( "\033[2J" ); // Clear screen
    puts( "\033[H" );  // Move cursor to home position
    printf("Claw Command Interface\n");
    printf("----------------------\n");
    // Prompt for command
    printf("#: ");

    // Main loop
    while (true) 
    {
        // Process millisecond tasks
        if(ms_ticks_count > 0)
        {
            // Decrement the tick count
            ms_ticks_count--;

            // Process LED timing
            process_led_tick(); 

            // Process stdin input
            cmd = process_stdin_input();
            
            // If we have a command, process it
            if(cmd != NULL)
            {
                process_command(cmd, &stepper);
                // Reset for next command
                printf("#: ");
                cmd = NULL; // Clear command pointer, probably not necessary
            }
            
            // Process stepper estop input and stepper status LEDs
            process_stepper_estop(&stepper);

            // Process stepper enabled LED
            process_stepper_enabled_led(&stepper);
        }

        // Process ten microsecond tasks
        if(ten_us_ticks_count > 0)
        {
            // Decrement the tick count
            ten_us_ticks_count--;

            // Process stepper movement
            if(stepper.moving)
            {
                process_stepper_movement(&stepper); 
            }
        }
    }
}
