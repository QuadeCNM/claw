/**
    * @file stepper.h
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Definitions, functions and variables for stepper motor control
    * 
    * This file contains the definitions, functions and variables for controlling a stepper motor.
*/

#ifndef STEPPER_H
#define STEPPER_H

// Stepper motor configuration
#define DEFAULT_STEPPER_PERIOD              4       // Default step period in TIMER_INTERVAL_US units (4 * 10 us = 40 us = 25 kHz)
#define MIN_STEPPER_PERIOD                  4       // Minimum step period in TIMER_INTERVAL_US units (4 * 10 us = 40 us = 25 kHz)
#define STEPPER_STEP_PIN                    6       // GPIO pin for stepper step control
#define STEPPER_DIR_PIN                     7       // GPIO pin for stepper direction control
#define STEPPER_ENABLE_PIN                  8       // GPIO pin for stepper enable control
#define STEPPER_ENABLE_PIN_INVERTED         true    // Set to true if enable pin is active low
#define STEPPER_ENABLE_LED_PIN              14      // GPIO pin for stepper enable LED (optional)
#define STEPPER_ENABLE_LED_PIN_ACTIVE_LEVEL 1       // Active level for enable LED (0 = active low, 1 = active high)
#define STEPPER_ESTOP_LED_PIN               15      // GPIO pin for stepper estop LED (optional)
#define STEPPER_ESTOP_LED_PIN_ACTIVE_LEVEL  1   // Active level for estop LED (0 = active low, 1 = active high)
#define STEPPER_ESTOP_PIN                   16      // GPIO pin for estop input (optional)
#define STEPPER_ESTOP_ACTIVE_LEVEL          0       // Active level for estop input pin (0 = active low, 1 = active high)
#define STEPPER_ESTOP_DEACTIVATE_DELAY_MS   100     // Number of consecutive checks for estop deactivation before re-enabling stepper
#define STEPPER_DIRECTION_FORWARD           1
#define STEPPER_DIRECTION_BACKWARD          0
#define STEPPER_STEPS_PER_REV               3200    // Number of steps per revolution for the stepper motor
                                                    // 16 microsteps / 1.8 degree step angle * 360 degrees = 3200 steps
#define STEPPER_MAX_REVOLUTIONS             12      // Maximum number of revolutions the stepper can move
                                                    // 20 TPI lead screw with 3/4 inch travel = 15 revolutions
#define STEPPER_BUMP_STEPS                  (STEPPER_STEPS_PER_REV / 4) // Number of steps to move for a bump down command (1/4 revolution)
#define MAX_STEPPER_POSITION                (STEPPER_STEPS_PER_REV * STEPPER_MAX_REVOLUTIONS)
#define MIN_STEPPER_POSITION                0

#define STATUS_LED_ON                       1
#define STATUS_LED_OFF                      0

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
 * @brief Enable the stepper motor
 *
 * @param stepper: pointer to stepper state structure, must not be NULL
 * @param enable: true to enable, false to disable
 * @return: true on success, false on failure
 */
bool stepper_enable(stepper_state_t* stepper, bool enable);

/*!
 * @brief Process stepper movement
 *
 * @param stepper: pointer to stepper state structure
 * @return: true if stepper is still moving, false if it has reached target
 */
bool process_stepper_movement(stepper_state_t* stepper);

/*!
 * @brief Process stepper estop input
 * @param stepper: pointer to stepper state structure
 * @return: true if estop is active, false otherwise
 */

bool process_stepper_estop(stepper_state_t* stepper);

/*!
 * @brief Check if the estop is active and set estop status LED appropriately
 * @param stepper: pointer to stepper state structure
 * @return: true if estop is active, false otherwise
 */
bool stepper_is_estop_active(stepper_state_t* stepper);


/*!
 * @brief Process stepper enabled LED
 * @param stepper: pointer to stepper state structure
 * @return: true on success, false on failure
 */ 
bool process_stepper_enabled_led(stepper_state_t* stepper);

#endif // STEPPER_H
