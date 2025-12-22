/**
    * @file command_processor.c
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief implementation of command processing functions
    * 
    * This file contains the implementation of functions for processing commands.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "sys_timer.h"
#include "stepper.h"
#include "led.h"
#include "command_processor.h"

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

/* -------------------------- command processor -----------------------------*/
bool process_command(const char* cmd, stepper_state_t* stepper)
{
    //check for null pointers
    if (cmd == NULL || stepper == NULL)
    {
        return false;
    }

    // check that command is null terminated and within length limits
    if(strnlen(cmd, MAX_COMMAND_LENGTH) >= MAX_COMMAND_LENGTH)
    {
        printf("Error: Command too long\n");
        return false;
    }

    // claw set position command
    if (strncmp(cmd, CLAW_SET_POSITION_COMMAND, strlen(CLAW_SET_POSITION_COMMAND)) == 0) 
    {
        return command_claw_set_position(stepper, cmd);
    }  
    // command to set LED period
    else if (strncmp(cmd, LED_PERIOD_COMMAND, strlen(LED_PERIOD_COMMAND)) == 0) 
    {
        return command_set_led_period(cmd);
    }
    // command to set stepper period
    else if(strncmp(cmd, SET_STEPPER_PERIOD_COMMAND, strlen(SET_STEPPER_PERIOD_COMMAND)) == 0)
    {
        return command_set_stepper_period(stepper, cmd);
    }
    // command to set stepper position to zero
    else if(strncmp(cmd, SET_STEPPER_ZERO_COMMAND, strlen(SET_STEPPER_ZERO_COMMAND)) == 0)
    {
        return command_set_stepper_zero(stepper);
    }
    // command to move stepper to absolute position
    else if(strncmp(cmd, MOVE_STEPPER_ABSOLUTE_COMMAND, strlen(MOVE_STEPPER_ABSOLUTE_COMMAND)) == 0)
    {
        return command_move_stepper_absolute(stepper, cmd);
    }
    // command to move stepper by relative steps
    else if(strncmp(cmd, MOVE_STEPPER_RELATIVE_COMMAND, strlen(MOVE_STEPPER_RELATIVE_COMMAND)) == 0)
    {
        return command_move_stepper_relative(stepper, cmd);
    }
    // command to move stepper by relative rotations
    else if(strncmp(cmd, MOVE_STEPPER_ROTATIONS_COMMAND, strlen(MOVE_STEPPER_ROTATIONS_COMMAND)) == 0)
    {
        return command_move_stepper_rotations(stepper, cmd);
    }
    // command to bump stepper down by fixed amount
    else if(strncmp(cmd, MOVE_STEPPER_BUMP_DOWN_COMMAND, strlen(MOVE_STEPPER_BUMP_DOWN_COMMAND)) == 0)
    {
        return command_move_stepper_bump_down(stepper);
    }
    // command to stop stepper
    else if(strncmp(cmd, STOP_STEPPER_COMMAND, strlen(STOP_STEPPER_COMMAND)) == 0)
    {
       return command_stop_stepper(stepper);
    }
    // command to get stepper status
    else if(strncmp(cmd, GET_STEPPER_STATUS_COMMAND, strlen(GET_STEPPER_STATUS_COMMAND)) == 0)
    {
        return command_get_stepper_status(stepper);
    }
    // command to enable stepper
    else if(strncmp(cmd, ENABLE_STEPPER_COMMAND, strlen(ENABLE_STEPPER_COMMAND)) == 0)
    {
        stepper_enable(stepper, true);
        printf("Stepper motor enabled\n");
        return true;
    }
    // command to disable stepper
    else if(strncmp(cmd, DISABLE_STEPPER_COMMAND, strlen(DISABLE_STEPPER_COMMAND)) == 0)
    {
        stepper_enable(stepper, false); 
        printf("Stepper motor disabled\n");
        return true;
    }
    // help command
    else if (strncmp(cmd, "help", 4) == 0 || strncmp(cmd, "", 1) == 0) 
    {
        printf("%s", help_message);
        return true;
    }
    // unknown command
    else 
    {
        printf("Unknown command: \"%s\"\n-----\n", cmd);
        printf("%s", help_message);
        return false;
    }

    // Should not reach here
    return true;
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

/* -------------------------- command helper functions -----------------------------*/
/* Note: Command helper functions should be of the form of one of the following.    */
/* bool function_name(stepper_state_t* stepper)                                     */
/* bool function_name(stepper_state_t* stepper, bool value)                         */
/* bool function_name(stepper_state_t* stepper, const char* command_string)         */
/* bool function_name(const char* command_string)                                   */
/* ---------------------------------------------------------------------------------*/
bool command_get_stepper_status(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        printf("Error: Could not get stepper status\n");
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

bool command_claw_set_position(stepper_state_t* stepper, const char* cmd)
{
    float position = atof(cmd + strlen(CLAW_SET_POSITION_COMMAND));

    if( stepper == NULL )
    {
        return false;
    }

    // Check if stepper is enabled
    if(stepper->enabled == false)
    {
        printf("Error: Stepper motor is disabled. Enable it first.\n");
        return false;
    }
    // Check for valid position
    else if (position < 0 || position > 100)
    {
        printf("Error: Claw position must be between 0 and 100\n");
        return false;
    }
    // Set the target position as a percentage of max stepper position
    else
    {
        int new_stepper_position = (int)round((position * MAX_STEPPER_POSITION) / 100.0);
        stepper_set_target_position(stepper, new_stepper_position);
        stepper->moving = true; // Start moving
        printf("Claw position set to %.2f%% (%d)\n", position, new_stepper_position);
        return true;
    }
}

bool command_set_led_period(const char* cmd)
{
    int new_period = atoi(cmd + strlen(LED_PERIOD_COMMAND));
    if (new_period > 0) 
    {
        led_period = new_period;
        printf("LED period set to %d ms\n", led_period);
        return true;
    }
    else
    {
        printf("Error: Invalid LED period\n");
        return false;
    }
}

bool command_set_stepper_period(stepper_state_t* stepper, const char* cmd)
{
    int new_step_period = atoi(cmd + strlen(SET_STEPPER_PERIOD_COMMAND));
    
    if( stepper == NULL )
    {
        return false;
    }
    
    if(stepper_set_step_period(stepper, new_step_period))
    {
        printf("Stepper step period set to %d us\n", new_step_period);
        return true;
    }
    else
    {
        printf("Error: Invalid step period\n");
        return false;
    }
}

bool command_set_stepper_zero(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    stepper->current_position = 0;
    stepper->target_position = 0;
    stepper->moving = false;
    printf("Stepper position set to zero\n");
    return true;
}

bool command_move_stepper_absolute(stepper_state_t* stepper, const char* cmd)
{
    int target_position = atoi(cmd + strlen(MOVE_STEPPER_ABSOLUTE_COMMAND));

    if( stepper == NULL )
    {
        return false;
    }

    if(stepper->enabled == false)
    {
        printf("Error: Stepper motor is disabled. Enable it first.\n");
        return false;
    }

    if(stepper_set_target_position(stepper, target_position))
    {
        printf("Moving stepper to absolute position %d\n", target_position);
        stepper->moving = true;
        return true;
    }
    else
    {
        printf("Error: Invalid target position\n");
        return false;
    }
}

bool command_move_stepper_relative(stepper_state_t* stepper, const char* cmd)
{
    int relative_steps = atoi(cmd + strlen(MOVE_STEPPER_RELATIVE_COMMAND));
    int target_position = stepper->current_position + relative_steps;

    if( stepper == NULL )
    {
        return false;
    }

    if(stepper->enabled == false)
    {
        printf("Error: Stepper motor is disabled. Enable it first.\n");
        return false;
    }

    if(stepper_set_target_position(stepper, target_position))
    {
        printf("Moving stepper to relative position %d\n", target_position);
        stepper->moving = true;
        return true;
    }
    else
    {
        printf("Error: Invalid target position\n");
        return false;
    }
}

bool command_move_stepper_rotations(stepper_state_t* stepper, const char* cmd)
{
    double relative_rotations = atof(cmd + strlen(MOVE_STEPPER_ROTATIONS_COMMAND));
    int relative_steps = (int)round(relative_rotations * STEPPER_STEPS_PER_REV);
    int target_position = stepper->current_position + relative_steps;

    if( stepper == NULL )
    {
        return false;
    }

    if(stepper->enabled == false)
    {
        printf("Error: Stepper motor is disabled. Enable it first.\n");
        return false;
    }

    if(stepper_set_target_position(stepper, target_position))
    {
        printf("Moving stepper by %+f rotations to position %d\n", relative_rotations, target_position);
        stepper->moving = true;
        return true;
    }
    else
    {
        printf("Error: Invalid target position\n");
        return false;
    }
}

bool command_move_stepper_bump_down(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    if(stepper->enabled == false)
    {
        printf("Error: Stepper motor is disabled. Enable it first.\n");
        return false;
    }

    // Check if bump down is within current limits
    if(stepper->current_position > STEPPER_BUMP_STEPS)
    {
        printf("Bumping stepper down by %d steps\n", STEPPER_BUMP_STEPS);
        stepper->target_position = stepper->current_position - STEPPER_BUMP_STEPS; //
        stepper->moving = true;
        return true;
    }
    // If bump down exceeds minimum position, reset to allow bump
    else
    {
        printf("Bump down exceeds minimum position, resetting zero to allow bump\n");
        stepper->current_position = STEPPER_BUMP_STEPS;  // Set current position to allow bump down
        stepper->target_position = 0; // Set target position to zero
        stepper->moving = true;
        return true;
    }
}

bool command_stop_stepper(stepper_state_t* stepper)
{
    if( stepper == NULL )
    {
        return false;
    }

    if(stepper_stop(stepper))
    {
        printf("Stepper stopped at position %d\n", stepper->current_position);
        return true;
    }
    else
    {
        printf("Error: Could not stop stepper\n");
        return false;
    }
}