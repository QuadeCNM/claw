/**
    * @file sys_timer.h
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief Definitions, functions and variables for system timer
    * 
    * This file contains the timer callback function and related definitions for the system timer.
*/

#ifndef SYS_TIMER_H
#define SYS_TIMER_H

#define TIMER_INTERVAL_US              10       // Timer interval in microseconds

/*! 
 * @brief Global ten microsecond ticks count
 *
 * This variable is incremented by the timer callback
 * and decremented in the main loop to track when the ten microsecond tasks should run.
 */
extern volatile int ten_us_ticks_count;

/*! 
 * @brief Global millisecond ticks count
 *
 * This variable is incremented by the timer callback every 100 calls (1 ms = 100 * 10 us)
 * and decremented in the main loop to track when the millisecond tasks should run. 
 */

extern volatile int ms_ticks_count;

/*!
 * @brief Millisecond timer callback
 *
 * @note: This function is called every millisecond by the repeating timer.
 *
 * @param t: pointer to repeating_timer struct
 * @return: true to keep repeating, false to stop
 */
bool timer_callback(struct repeating_timer *t);

#endif // SYS_TIMER_H