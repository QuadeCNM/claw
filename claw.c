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

// Define default LED delay if not defined
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

#define TIMER_INTERVAL_US              10       // Timer interval in microseconds
#define DEFAULT_STEPPER_PERIOD          4       // Default step period in TIMER_INTERVAL_US units (4 * 10 us = 40 us = 25 kHz)
#define MIN_STEPPER_PERIOD              4       // Minimum step period in TIMER_INTERVAL_US units (4 * 10 us = 40 us = 25 kHz)
#define STEPPER_STEP_PIN                2       // GPIO pin for stepper step control
#define STEPPER_DIR_PIN                 3       // GPIO pin for stepper direction control
#define STEPPER_ENABLE_PIN              4       // GPIO pin for stepper enable control
#define STEPPER_ENABLE_PIN_INVERTED     true    // Set to true if enable pin is active low
#define STEPPER_DIRECTION_FORWARD       1
#define STEPPER_DIRECTION_BACKWARD      0
#define STEPPER_STEPS_PER_REV           3200    // Number of steps per revolution for the stepper motor
                                                // 16 microsteps / 1.8 degree step angle * 360 degrees = 3200 steps
#define STEPPER_MAX_REVOLUTIONS         12      // Maximum number of revolutions the stepper can move
                                                // 20 TPI lead screw with 3/4 inch travel = 15 revolutions
#define STEPPER_BUMP_STEPS              (STEPPER_STEPS_PER_REV / 4) // Number of steps to move for a bump down command (1/4 revolution)
#define MAX_STEPPER_POSITION            (STEPPER_STEPS_PER_REV * STEPPER_MAX_REVOLUTIONS)
#define MIN_STEPPER_POSITION            0


/*!
 * @brief Structure to hold stepper motor state
 */
typedef struct stepper_state
{
    int current_position; //!< Current position in steps
    int target_position;  //!< Target position in steps
    int step_period;      //!< Step period in TIMMER_INTERVAL_US units
    bool moving;          //!< Is the stepper currently moving
    bool enabled;         //!< Is the stepper enabled
} stepper_state_t;

// Global variables

/*! 
 * @brief LED blink period in milliseconds
 * 
 * This variable can be modified via command interface to change the LED blink rate.
 */
volatile int led_period = LED_DELAY_MS;

/*! 
 * @brief Global ten microsecond ticks count
 *
 * This variable is incremented by the timer callback
 * and decremented in the main loop to track when the ten microsecond tasks should run.
 */
volatile int ten_us_ticks_count = 0;

/*! 
 * @brief Global millisecond ticks count
 *
 * This variable is incremented by the timer callback every 100 calls (1 ms = 100 * 10 us)
 * and decremented in the main loop to track when the millisecond tasks should run. 
 */

volatile int ms_ticks_count = 0;

// Command definitions
#define MAX_COMMAND_LENGTH              50

#define CLAW_SET_POSITION_COMMAND       "claw_set "
#define LED_PERIOD_COMMAND              "led_period "
#define SET_STEPPER_PERIOD_COMMAND      "set_stepper_period "
#define SET_STEPPER_ZERO_COMMAND        "set_stepper_zero"
#define MOVE_STEPPER_ABSOLUTE_COMMAND   "move_stepper_absolute "
#define MOVE_STEPPER_RELATIVE_COMMAND   "move_stepper_relative "
#define MOVE_STEPPER_ROTATIONS_COMMAND  "move_stepper_rotations "
#define MOVE_STEPPER_BUMP_DOWN_COMMAND  "move_stepper_bump_down"
#define STOP_STEPPER_COMMAND            "stop_stepper"
#define GET_STEPPER_STATUS_COMMAND      "get_stepper_status"
#define ENABLE_STEPPER_COMMAND          "enable_stepper"
#define DISABLE_STEPPER_COMMAND         "disable_stepper"

/*! 
 * @brief Help message
 *
 * This message is displayed when the user requests help or enters an unknown command.
 */
const char* help_message =
    "\n"
    "Available commands:\n"
    "  claw_set <position>                - Set the claw position 0 to 100\n"
    "  led_period <ms>                    - Set the LED blink period in milliseconds\n"
    "  set_stepper_period <us>            - Set the stepper motor step period in us\n"
    "  set_stepper_zero                   - Set the current position to zero\n"
    "  move_stepper_absolute <steps>      - Move the stepper to an absolute position\n"
    "  move_stepper_relative <steps>      - Move the stepper by a relative number of steps\n"
    "  move_stepper_rotations <rotations> - Move the stepper by a number of rotations\n"
    "  move_stepper_bump_down             - Move the stepper down by a small fixed amount\n"
    "  stop_stepper                       - Stop the stepper motor\n"
    "  get_stepper_status                 - Get the current status of the stepper motor\n"
    "  enable_stepper                     - Enable the stepper motor\n"
    "  disable_stepper                    - Disable the stepper motor\n"
    "  help                               - Show this help message\n"
    "-----\n";

// Function prototypes

/*!
 * @brief Initialize the stepper state
 *
 * @param stepper: pointer to stepper state structure to initialize, must not be NULL
 * @param initial_position: initial position in steps must be between MIN_STEPPER_POSITION and MAX_STEPPER_POSITION
 * @param step_period: step period in TIMER_INTERVAL_US must be greater than 1 ms
 * @return: true on success, false on failure
 */
bool stepper_init(stepper_state_t* stepper, int initial_position, int step_period);

/*!
 * @brief Set the target position for the stepper motor
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @param target_position: target position in steps must be between MIN_STEPPER_POSITION and MAX_STEPPER_POSITION
 * @return: true on success, false on failure
 */
bool stepper_set_target_position(stepper_state_t* stepper, int target_position);

/*!
 * @brief Set the step period for the stepper motor
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @param step_period: step period in microseconds must be greater than MIN_STEPPER_PERIOD
 * @return: true on success, false on failure
 */
bool stepper_set_step_period(stepper_state_t* stepper, int step_period_us);

/*!
 * @brief Stop the stepper motor, setting target position to current position
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @return: true on success, false on failure
 */
bool stepper_stop(stepper_state_t* stepper);

/*!
 * @brief Get the current status of the stepper motor, printing to stdout
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @return: true on success, false on failure
 */
bool stepper_get_status(stepper_state_t* stepper);

/*!
 * @brief Enable the stepper motor
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @param enable: true to enable, false to disable
 * @return: true on success, false on failure
 */
bool stepper_enable(stepper_state_t* stepper, bool enable);

/*!
 * @brief Initialise the LED
 *
 * @param: none
 * @return: PICO_OK on success
            error code on failure
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
 * @brief Process stepper movement
 *
 * @param stepper: pointer to stepper state structure
 * @return: true if stepper is still moving, false if it has reached target
 */
bool process_stepper_movement(stepper_state_t* stepper);

/*!
 * @brief Process LED timing tick
 *
 * @param: none
 * @return: none 
 */
void process_led_tick(void);

/*!
 * @brief Millisecond timer callback
 *
 * @note: This function is called every millisecond by the repeating timer.
 *
 * @param t: pointer to repeating_timer struct
 * @return: true to keep repeating, false to stop
 */
bool timer_callback(struct repeating_timer *t);

/*!
 * @brief Process a command string
 *
 * @param cmd: pointer to command string
 * @param stepper: pointer to stepper state structure
 * @return: 0 on success, error code on failure
 */
int process_command(const char* cmd, stepper_state_t* stepper);

/*!
 * @brief Process stdin input
 *
 * @note: This function reads characters from stdin,
 *        builds commands, and processes them when a newline is received.
 *
 *        This function has a simple lock to prevent re-entrancy.
 *
 * @param: none
 * @return: none
 */
char* process_stdin_input(void);

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

    while (true) 
    {
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

            
        }
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

/* -------------------------- helper functions -----------------------------*/

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

bool timer_callback(struct repeating_timer *t)
{
    static int us_count = 0;
    // This function is called every 10 microseconds
    us_count++;
    if (us_count >= (1000 / TIMER_INTERVAL_US)) // 100 calls = 1 ms
    {
        us_count = 0;
        ms_ticks_count++;
    }
    ten_us_ticks_count++;
    return true;    
}

int process_command(const char* cmd, stepper_state_t* stepper)
{
    // claw set position command
    if (strncmp(cmd, CLAW_SET_POSITION_COMMAND, strlen(CLAW_SET_POSITION_COMMAND)) == 0) 
    {
        float position = atof(cmd + strlen(CLAW_SET_POSITION_COMMAND));

        // Check if stepper is enabled
        if(stepper->enabled == false)
        {
            printf("Error: Stepper motor is disabled. Enable it first.\n");
        }
        // Check for valid position
        else if (position < 0 || position > 100)
        {
            printf("Error: Claw position must be between 0 and 100\n");
        }
        // Set the target position as a percentage of max stepper position
        else
        {
            int new_stepper_position = (int)round((position * MAX_STEPPER_POSITION) / 100.0);
            stepper_set_target_position(stepper, new_stepper_position);
            stepper->moving = true; // Start moving
            printf("Claw position set to %.2f%% (%d)\n", position, new_stepper_position);
        }
    }  
    // command to set LED period
    else if (strncmp(cmd, LED_PERIOD_COMMAND, strlen(LED_PERIOD_COMMAND)) == 0) 
    {
        int new_period = atoi(cmd + strlen(LED_PERIOD_COMMAND));
        if (new_period > 0) 
        {
            led_period = new_period;
        }
        printf("LED period set to %d ms\n", led_period);
    }
    // command to set stepper period
    else if(strncmp(cmd, SET_STEPPER_PERIOD_COMMAND, strlen(SET_STEPPER_PERIOD_COMMAND)) == 0)
    {
        int new_step_period = atoi(cmd + strlen(SET_STEPPER_PERIOD_COMMAND));
        if(stepper_set_step_period(stepper, new_step_period))
        {
            printf("Stepper step period set to %d us\n", new_step_period);
        }
        else
        {
            printf("Error: Invalid step period\n");
        }
    }
    // command to set stepper position to zero
    else if(strncmp(cmd, SET_STEPPER_ZERO_COMMAND, strlen(SET_STEPPER_ZERO_COMMAND)) == 0)
    {
        stepper->current_position = 0;
        stepper->target_position = 0;
        stepper->moving = false;
        printf("Stepper position set to zero\n");
    }
    // command to move stepper to absolute position
    else if(strncmp(cmd, MOVE_STEPPER_ABSOLUTE_COMMAND, strlen(MOVE_STEPPER_ABSOLUTE_COMMAND)) == 0)
    {
        int target_position = atoi(cmd + strlen(MOVE_STEPPER_ABSOLUTE_COMMAND));
        if(stepper->enabled == false)
        {
            printf("Error: Stepper motor is disabled. Enable it first.\n");
        }
        else if(stepper_set_target_position(stepper, target_position))
        {
            printf("Moving stepper to absolute position %d\n", target_position);
            stepper->moving = true;
        }
        else
        {
            printf("Error: Invalid target position\n");
        }
    }
    // command to move stepper by relative steps
    else if(strncmp(cmd, MOVE_STEPPER_RELATIVE_COMMAND, strlen(MOVE_STEPPER_RELATIVE_COMMAND)) == 0)
    {
        int relative_steps = atoi(cmd + strlen(MOVE_STEPPER_RELATIVE_COMMAND));
        int target_position = stepper->current_position + relative_steps;
        if(stepper->enabled == false)
        {
            printf("Error: Stepper motor is disabled. Enable it first.\n");
        }
        else if(stepper_set_target_position(stepper, target_position))
        {
            printf("Moving stepper to relative position %d\n", target_position);
            stepper->moving = true;
        }
        else
        {
            printf("Error: Invalid target position\n");
        }
    }
    // command to move stepper by relative rotations
    else if(strncmp(cmd, MOVE_STEPPER_ROTATIONS_COMMAND, strlen(MOVE_STEPPER_ROTATIONS_COMMAND)) == 0)
    {
        double relative_rotations = atof(cmd + strlen(MOVE_STEPPER_ROTATIONS_COMMAND));
        int relative_steps = (int)round(relative_rotations * STEPPER_STEPS_PER_REV);
        int target_position = stepper->current_position + relative_steps;
        // Check if stepper is enabled
        if(stepper->enabled == false)
        {
            printf("Error: Stepper motor is disabled. Enable it first.\n");
        }
        // Check for valid target position
        else if(stepper_set_target_position(stepper, target_position))
        {
            printf("Moving stepper by %+f rotations to position %d\n", relative_rotations, target_position);
            stepper->moving = true;
        }
        else
        {
            printf("Error: Invalid target position\n");
        }
    }
    // command to bump stepper down by fixed amount
    else if(strncmp(cmd, MOVE_STEPPER_BUMP_DOWN_COMMAND, strlen(MOVE_STEPPER_BUMP_DOWN_COMMAND)) == 0)
    {
        if(stepper->enabled == false)
        {
            printf("Error: Stepper motor is disabled. Enable it first.\n");
        }
        // Check if bump down is within current limits
        else if(stepper->current_position > STEPPER_BUMP_STEPS)
        {
            printf("Bumping stepper down by %d steps\n", STEPPER_BUMP_STEPS);
            stepper->target_position = stepper->current_position - STEPPER_BUMP_STEPS; //
            stepper->moving = true;
        }
        // If bump down exceeds minimum position, reset to allow bump
        else
        {
            printf("Bump down exceeds minimum position, resetting zero to allow bump\n");
            stepper->current_position = STEPPER_BUMP_STEPS;  // Set current position to allow bump down
            stepper->target_position = 0; // Set target position to zero
            stepper->moving = true;
        }
    }
    // command to stop stepper
    else if(strncmp(cmd, STOP_STEPPER_COMMAND, strlen(STOP_STEPPER_COMMAND)) == 0)
    {
        if(stepper_stop(stepper))
        {
            printf("Stepper stopped at position %d\n", stepper->current_position);
        }
        else
        {
            printf("Error: Could not stop stepper\n");
        }
    }
    // command to get stepper status
    else if(strncmp(cmd, GET_STEPPER_STATUS_COMMAND, strlen(GET_STEPPER_STATUS_COMMAND)) == 0)
    {
        if(!stepper_get_status(stepper))
        {
            printf("Error: Could not get stepper status\n");
        }
    }
    // command to enable stepper
    else if(strncmp(cmd, ENABLE_STEPPER_COMMAND, strlen(ENABLE_STEPPER_COMMAND)) == 0)
    {
        stepper_enable(stepper, true);
        printf("Stepper motor enabled\n");
    }
    // command to disable stepper
    else if(strncmp(cmd, DISABLE_STEPPER_COMMAND, strlen(DISABLE_STEPPER_COMMAND)) == 0)
    {
        stepper_enable(stepper, false); 
        printf("Stepper motor disabled\n");
    }
    // help command
    else if (strncmp(cmd, "help", 4) == 0 || strncmp(cmd, "", 1) == 0) 
    {
        printf("%s", help_message);
    }
    // unknown command
    else 
    {
        printf("Unknown command: \"%s\"\n-----\n", cmd);
        printf("%s", help_message);
    }
    return 0;
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

char* process_stdin_input(void)
{
    static bool lock = false;
    int character;
    static char cmd_buffer[MAX_COMMAND_LENGTH];
    static int cmd_buffer_index = 0;
    bool process_cmd = false;
    char* result = NULL;

    // Simple lock to prevent re-entrancy
    if(lock)
    {
        return NULL;
    }   
    lock = true;

    // Try to read a character from stdin
    character = getchar_timeout_us(0);

    // If we got a character, process it
    if(character != PICO_ERROR_TIMEOUT)
    {
        // Store the character in the command buffer
        cmd_buffer[cmd_buffer_index] = (char)character;

        // Check for end of command
        if((cmd_buffer[cmd_buffer_index] == '\n'))
        {
            cmd_buffer[cmd_buffer_index] = '\0';
            process_cmd = true;
            putchar('\n');
        }
        // Handle backspace
        else if((cmd_buffer[cmd_buffer_index] == '\b' || cmd_buffer[cmd_buffer_index] == 127) && cmd_buffer_index > 0)
        {
            cmd_buffer_index--;
            putchar('\b');
            putchar(' ');
            putchar('\b');
        }

        // Echo the character and increment index
        else
        {
            // Only accept printable characters
            if((!iscntrl(cmd_buffer[cmd_buffer_index])) && (cmd_buffer[cmd_buffer_index] != '\b' && cmd_buffer[cmd_buffer_index] != 127))
            {
                putchar(cmd_buffer[cmd_buffer_index]);
                cmd_buffer_index++;
            }
        }
        
        // Prevent buffer overflow
        if(cmd_buffer_index >= sizeof(cmd_buffer) - 1)
        {
            cmd_buffer_index = sizeof(cmd_buffer) - 2;
        }
    }

    // If we have a complete command, process it
    if(process_cmd)
    {
        cmd_buffer_index = 0;
        process_cmd = false;
        result = cmd_buffer;
    }

    // Release the lock
    lock = false;
    return result;
}

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

bool stepper_get_status(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    printf("Stepper Status:\n");
    printf("  Current Position: %d\n", stepper->current_position);
    printf("  Target Position: %d\n", stepper->target_position);
    printf("  Step Period (us): %d\n", stepper->step_period * TIMER_INTERVAL_US);
    printf("  Moving: %s\n", stepper->moving ? "Yes" : "No");
    printf("  Enabled: %s\n", stepper->enabled ? "Yes" : "No");
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
