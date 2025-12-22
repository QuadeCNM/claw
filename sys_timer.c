
/**
    * @file sys_timer.c
    * @author Jon Wade
    * @date  20 Dec 2025
    * @copyright (c) 2025 Jon Wade. Standard MIT License applies. See LICENSE file.
    *
    * @brief implementation of system timer and related global variables
    * 
    * This file contains the timer callback function and related definitions for the system timer.
*/

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "sys_timer.h"

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

/* -------------------------- timer callback function -----------------------------*/
/* Note: This function is called every 10 microseconds, it needs to be fast. so    */
/* keep it minimal                                                                 */
/* --------------------------------------------------------------------------------*/
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