/**
    * @file stepper.c
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief implementation of stepper motor control and related global variables
    * 
    * This file contains the implementation of functions for controlling a stepper motor.
*/

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "stepper.h"
#include "sys_timer.h"

/* -------------------------- stepper helper functions -----------------------------*/
bool stepper_init(stepper_state_t* stepper, int initial_position, int step_period)
{
    if( stepper == NULL )
    {
        return false;
    }

    if( initial_position < MIN_STEPPER_POSITION || initial_position > MAX_STEPPER_POSITION )
    {
        return false;
    } 

    if( step_period <= 1 )
    {
        return false;
    }

    stepper->current_position = initial_position;
    stepper->target_position = initial_position;
    stepper->step_period = step_period;
    stepper->moving = false;
    stepper->enabled = false;
    stepper_enable(stepper, false); // Disable stepper motor initially

    // Initialise optional GPIO pins for stepper status LEDs and estop input
    gpio_init(STEPPER_ENABLE_LED_PIN);
    gpio_set_dir(STEPPER_ENABLE_LED_PIN, GPIO_OUT);
    gpio_put(STEPPER_ENABLE_LED_PIN, STEPPER_ENABLE_LED_PIN_ACTIVE_LEVEL); // Turn on enable LED
    gpio_init(STEPPER_ESTOP_LED_PIN);
    gpio_set_dir(STEPPER_ESTOP_LED_PIN, GPIO_OUT);
    gpio_put(STEPPER_ESTOP_LED_PIN, STEPPER_ESTOP_ACTIVE_LEVEL); // TURN on estop LED
    gpio_init(STEPPER_ESTOP_PIN);
    gpio_set_dir(STEPPER_ESTOP_PIN, GPIO_IN);
    gpio_pull_up(STEPPER_ESTOP_PIN);
    
    return true;
}

bool stepper_set_target_position(stepper_state_t* stepper, int target_position)
{
    if( stepper == NULL )
    {
        return false;
    }

    if( target_position < MIN_STEPPER_POSITION || target_position > MAX_STEPPER_POSITION )
    {
        return false;
    } 

    stepper->target_position = target_position;
    stepper->moving = true;
    return true;
}

bool stepper_set_step_period(stepper_state_t* stepper, int step_period_us)
{
    if( stepper == NULL )
    {
        return false;
    }

    if( step_period_us < (MIN_STEPPER_PERIOD * TIMER_INTERVAL_US) )
    {
        return false;
    }

    stepper->step_period = step_period_us / TIMER_INTERVAL_US;
    return true;
}

bool stepper_stop(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    stepper->target_position = stepper->current_position;
    stepper->moving = false;
    return true;
}

bool stepper_enable(stepper_state_t* stepper, bool enable)
{
    static bool gpio_initialized = false;
    if(!gpio_initialized)
    {
        gpio_init(STEPPER_ENABLE_PIN);
        gpio_set_dir(STEPPER_ENABLE_PIN, GPIO_OUT);
        gpio_initialized = true;
    }

    if( stepper == NULL )
    {
        return false;
    }

    gpio_put(STEPPER_ENABLE_PIN, enable ? (1 ^ STEPPER_ENABLE_PIN_INVERTED) : (0 ^ STEPPER_ENABLE_PIN_INVERTED)); // Enable or disable the stepper motor
    stepper->enabled = enable;
    return true;
}



bool stepper_is_estop_active(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    // Read estop input pin
    if(gpio_get(STEPPER_ESTOP_PIN) == STEPPER_ESTOP_ACTIVE_LEVEL)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool process_stepper_estop(stepper_state_t* stepper)
{
    static int extop_active_count = 0;

    if( stepper == NULL )
    {
        return false;
    }

    // Read estop input pin
    if(gpio_get(STEPPER_ESTOP_PIN) == STEPPER_ESTOP_ACTIVE_LEVEL)
    {
        // Estop is active, disable stepper motor
        stepper_enable(stepper, false);
        gpio_put(STEPPER_ESTOP_LED_PIN, STEPPER_ESTOP_LED_PIN_ACTIVE_LEVEL);
        stepper->moving = false; // Stop any movement
        stepper->target_position = stepper->current_position; // Set target to current position
        extop_active_count = STEPPER_ESTOP_DEACTIVATE_DELAY_MS; // Reset deactivate delay counter
        return true;
    }
    else
    {
        // Decrement estop deactivate delay counter
        if(extop_active_count > 0)
        {
            extop_active_count--;
            // Keep estop active until delay expires
            gpio_put(STEPPER_ESTOP_LED_PIN, STEPPER_ESTOP_LED_PIN_ACTIVE_LEVEL);
            return true;
        }
        else
        {
            // Estop is not active, enable stepper motor if it was previously enabled
            gpio_put(STEPPER_ESTOP_LED_PIN, STEPPER_ESTOP_LED_PIN_ACTIVE_LEVEL ^ 1 );
            return false;
        }
    }
}

bool process_stepper_enabled_led(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    // Set moving LED based on stepper moving status
    if(stepper->enabled)
    {
        gpio_put(STEPPER_ENABLE_LED_PIN, STEPPER_ENABLE_LED_PIN_ACTIVE_LEVEL);
    }
    else
    {
        gpio_put(STEPPER_ENABLE_LED_PIN, STEPPER_ENABLE_LED_PIN_ACTIVE_LEVEL ^ 1);
    }
    return true;
}   

/* -------------------------- stepper movement processing function -----------------------------*/

bool process_stepper_movement(stepper_state_t* stepper)
{
    static bool function_initialized = false;
    static int step_timer = 0;
    int direction;

    if(!function_initialized)
    {
        // Initialise GPIO pins for stepper control
        gpio_init(STEPPER_STEP_PIN);
        gpio_set_dir(STEPPER_STEP_PIN, GPIO_OUT);
        gpio_init(STEPPER_DIR_PIN);
        gpio_set_dir(STEPPER_DIR_PIN, GPIO_OUT);
        gpio_put(STEPPER_STEP_PIN, 0);
        gpio_put(STEPPER_DIR_PIN, 0);

        // Mark as initialized
        function_initialized = true;
    }

    if( stepper == NULL )
    {
        return false;
    }

    // Determine direction
    if( stepper->target_position > stepper->current_position)
    {
        direction = STEPPER_DIRECTION_FORWARD; // Forward
    }
    else
    {
        direction = STEPPER_DIRECTION_BACKWARD; // Backward
    }

    // Set direction pin
    gpio_put(STEPPER_DIR_PIN, direction);

    // Check if we are moving
    if( stepper->moving )
    {
        // Increment step timer
        step_timer++;

        if( step_timer == stepper->step_period/2 )
        {
            // set step pin high
            gpio_put(STEPPER_STEP_PIN, 1);
        }
        else if( step_timer >= stepper->step_period )
        {
            // set step pin low
            gpio_put(STEPPER_STEP_PIN, 0);
            step_timer = 0;

            // Update current position
            if( direction == STEPPER_DIRECTION_FORWARD )
            {
                stepper->current_position++;
            }
            else
            {
                stepper->current_position--;
            }
            // Check if we have reached the target position
            if( stepper->current_position == stepper->target_position )
            {
                stepper->moving = false;
                //printf("\nStepper reached target position: %d\n", stepper->current_position);
            }
        }
    }
    else
    {
        // Ensure step pin is low when not moving
        gpio_put(STEPPER_STEP_PIN, 0);
        step_timer = 0;
    }
    return stepper->moving;    
}
