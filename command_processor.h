/**
    * @file command_processor.h
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Definitions, functions and variables for command processing
    * 
    * This file contains the definitions, functions and variables for processing commands.
*/

#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include "stepper.h"

/*!
 * @brief Process a command string
 *
 * @note: This function parses the command string and calls the appropriate command helper function.
 *        It checks for null pointers and command length.
 * 
 * @param cmd: pointer to command string
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */
bool process_command(const char* cmd, stepper_state_t* stepper);

/*!
 * @brief Process stdin input
 *
 * @note: This function reads characters from stdin,
 *        builds commands, and returns complete command strings.
 *
 *        This function has a simple lock to prevent re-entrancy.
 *
 * @param: none
 * @return: non-null pointer to command string when a complete command is received,
 */
char* process_stdin_input(void);

/*!
 * @brief Command helper function to set claw position
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param stepper: pointer to stepper state structure
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_claw_set_position(stepper_state_t* stepper, const char* cmd);

/*!
 * @brief Command helper function to set LED period
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_set_led_period(const char* cmd);

/*!
 * @brief Command helper function to set stepper period
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param stepper: pointer to stepper state structure
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_set_stepper_period(stepper_state_t* stepper, const char* cmd);

/*!
 * @brief Command helper function to set stepper position to zero
 *
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */
bool command_set_stepper_zero(stepper_state_t* stepper);

/*!
 * @brief Command helper function to move stepper to an absolute position
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param stepper: pointer to stepper state structure
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_move_stepper_absolute(stepper_state_t* stepper, const char* cmd);

/*!
 * @brief Command helper function to move stepper by a relative amount
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param stepper: pointer to stepper state structure
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_move_stepper_relative(stepper_state_t* stepper, const char* cmd);

/*!
 * @brief Command helper function to move stepper by a relative amount
 *
 * @note: Function is not completely safe, assumes valid command string
 *
 * @param stepper: pointer to stepper state structure
 * @param cmd: pointer to command string
 * @return: true on success, false on failure
 */
bool command_move_stepper_rotations(stepper_state_t* stepper, const char* cmd);

/*!
 * @brief Command helper function to move stepper by a relative amount
 *
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */
bool command_move_stepper_bump_down(stepper_state_t* stepper);

/*!
 * @brief Command helper function to stop stepper movement
 *
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */
bool command_stop_stepper(stepper_state_t* stepper);

/*!
 * @brief Command helper function to get stepper status
 *
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */
bool command_get_stepper_status(stepper_state_t* stepper);

#endif // COMMAND_PROCESSOR_H