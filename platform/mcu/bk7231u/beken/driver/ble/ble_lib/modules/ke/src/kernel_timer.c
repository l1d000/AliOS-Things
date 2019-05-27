/**
 ****************************************************************************************
 *
 * @file kernel_timer.c
 *
 * @brief This file contains the scheduler primitives called to create or delete
 * a task. It contains also the scheduler itself.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup KERNEL_TIMER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>              // standard definition
#include <stdint.h>              // standard integer
#include <stdbool.h>             // standard boolean
#include "architect.h"                // architecture

#include "kernel_config.h"           // kernel configuration
#include "kernel_event.h"            // kernel event
#include "kernel_mem.h"              // kernel memory
#include "kernel_task.h"             // kernel task
#include "kernel_timer.h"            // kernel timer
#include "kernel_env.h"              // kernel environment
#include "kernel_queue.h"            // kernel queue


#if (EA_PRESENT)
#include "ea.h"                  // Event Arbiter defines
#if (BLE_EMB_PRESENT)
#include "lld_evt.h"
#endif // BLE_EMB_PRESENT
#else
#include "timer.h"               // timer defines
#endif // EA_PRESENT

#include "dbg_swdiag.h"          // Software diag

#if BT_EMB_PRESENT
#include "reg_btcore.h"          // HW timer resource in BT CORE
#endif //BT_EMB_PRESENT


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum timer value
#if (BT_EMB_PRESENT)
#define KERNEL_GROSSTIMER_MASK      (BT_GROSSTARGET_MASK)
#elif (BLE_EMB_PRESENT)
#define KERNEL_GROSSTIMER_MASK      (BLE_GROSSTARGET_MASK)
#else
#define KERNEL_GROSSTIMER_MASK      (HL_GROSSTARGET_MASK)
#endif // BT_EMB_PRESENT.

/// Maximum timer value
#define KERNEL_TIMER_DELAY_MAX      (KERNEL_GROSSTIMER_MASK >> 1)

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve time.
 *
 * @return time value (in Kernel time (10 ms))
 ****************************************************************************************
 */
static uint32_t kernel_time(void)
{
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    return ((ea_time_get_halfslot_rounded() >> 4) & KERNEL_GROSSTIMER_MASK);
    #else
    return (timer_get_time() & KERNEL_GROSSTIMER_MASK);
    #endif // BT_EMB_PRESENT
}

/**
 ****************************************************************************************
 * @brief Compare given time.
 *
 * @param[in] newer     newer time value
 * @param[in] older     older time value
 *
 * @return bool
 ****************************************************************************************
 */
static bool kernel_time_cmp(uint32_t newer, uint32_t older)
{
    return (((uint32_t)(newer - older) & KERNEL_GROSSTIMER_MASK) <= KERNEL_TIMER_DELAY_MAX);
}

/**
 ****************************************************************************************
 * @brief Check if ke time has passed.
 *
 * @param[in] time     time value to check
 *
 * @return if time has passed or not
 ****************************************************************************************
 */
static bool kernel_time_past(uint32_t time)
{
    return kernel_time_cmp(kernel_time(), time);
}


/**
 ****************************************************************************************
 * @brief Set the HW timer with the first timer of the queue
 *
 * @param[in] timer       Timer value
 ****************************************************************************************
 */
static void kernel_timer_hw_set(struct kernel_timer *timer)
{
    #if (BT_EMB_PRESENT)
    if (timer)
    {
        // set the abs timeout in HW
        bt_grosstimtgt_grosstarget_setf(timer->time);

        // enable timer irq
        if (!bt_intcntl_grosstgtintmsk_getf())
        {
            // if timer is not enabled, it is possible that the irq is raised
            // due to a spurious value, so ack it before
            bt_intack_clear(BT_GROSSTGTINTACK_BIT);
            bt_intcntl_grosstgtintmsk_setf(1);
        }
    }
    else
    {
        // disable timer irq
        bt_intcntl_grosstgtintmsk_setf(0);
    }
    #elif (BLE_EMB_PRESENT)
    if (timer)
    {
        // set the abs timeout in HW
        ble_grosstarget_setf(timer->time);

        // enable timer irq
        if (!ble_grosstgtimintmsk_getf())
        {
            // if timer is not enabled, it is possible that the irq is raised
            // due to a spurious value, so ack it before
            ble_intack_clear(BLE_GROSSTGTIMINTACK_BIT);
            ble_grosstgtimintmsk_setf(1);
        }
    }
    else
    {
        // disable timer irq
        ble_grosstgtimintmsk_setf(0);
    }
    
    #else
    if (timer)
    {
        // set the abs timeout in HW
        timer_set_timeout(timer->time);

        // enable timer
        timer_enable(true);
    }
    else
    {
        // disable timer irq
        timer_enable(false);
    }
    #endif // BT_EMB_PRESENT / BLE_EMB_PRESENT
}

/**
 ****************************************************************************************
 * @brief Compare timer absolute expiration time.
 *
 * @param[in] timerA Timer to compare.
 * @param[in] timerB Timer to compare.
 *
 * @return true if timerA will expire before timerB.
 ****************************************************************************************
 */
static bool cmp_abs_time(struct common_list_hdr const * timerA, struct common_list_hdr const * timerB)
{
    uint32_t timeA = ((struct kernel_timer*)timerA)->time;
    uint32_t timeB = ((struct kernel_timer*)timerB)->time;

    return (((uint32_t)(timeA - timeB) & KERNEL_GROSSTIMER_MASK) > KERNEL_TIMER_DELAY_MAX);
}

/**
 ****************************************************************************************
 * @brief Compare timer and task IDs callback
 *
 * @param[in] timer           Timer value
 * @param[in] timer_task      Timer task
 *
 * @return timer insert
 ****************************************************************************************
 */
static bool cmp_timer_id(struct common_list_hdr const * timer, uint32_t timer_task)
{
    // trick to pack 2 u16 in u32
    kernel_msg_id_t timer_id = timer_task >> 16;
    kernel_task_id_t task_id = timer_task & 0xFFFF;

    // insert the timer just before the first one older
    return (timer_id == ((struct kernel_timer*)timer)->id)
        && (task_id == ((struct kernel_timer*)timer)->task);
}

/**
 ****************************************************************************************
 * @brief Schedule the next timer(s).
 *
 * This function pops the first timer from the timer queue and notifies the appropriate
 * task by sending a kernel message. If the timer is periodic, it is set again;
 * if it is one-shot, the timer is freed. The function checks also the next timers
 * and process them if they have expired or are about to expire.
 ****************************************************************************************
 */
static void kernel_timer_schedule(void)
{
    struct kernel_timer *timer;
    DBG_SWDIAG(EVT, TIMER, 1);
    for(;;)
    {
        kernel_event_clear(KERNEL_EVENT_KERNEL_TIMER);

        // check the next timer
        timer = (struct kernel_timer*) kernel_env.queue_timer.first;

        if (!timer)
        {
            // no more timers, disable HW irq and leave
            kernel_timer_hw_set(NULL);
            break;
        }

        if (!kernel_time_past(timer->time - 1))
        {
            // timer will expire in more than 10ms, configure the HW
            kernel_timer_hw_set(timer);

            // in most case, we will break here. However, if the timer expiration
            // time has just passed, may be the HW was set too late (due to an IRQ)
            // so we do not exit the loop to process it.
            if (!kernel_time_past(timer->time))
                break;
        }

        // at this point, the next timer in the queue has expired or is about to -> pop it
        timer = (struct kernel_timer*) kernel_queue_pop(&kernel_env.queue_timer);

        // notify the task
        kernel_msg_send_basic(timer->id, timer->task, TASK_BLE_NONE);

        // free the memory allocated for the timer
        kernel_free(timer);
    }
    DBG_SWDIAG(EVT, TIMER, 0);
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


void kernel_timer_init(void)
{
    // Register timer event
    kernel_event_callback_set(KERNEL_EVENT_KERNEL_TIMER, &kernel_timer_schedule);
}

void kernel_timer_set(kernel_msg_id_t const timer_id, kernel_task_id_t const task_id, uint32_t delay)
{
    // Indicate if the HW will have to be reprogrammed
    bool hw_prog = false;
    // Timer time
    uint32_t abs_time;

    // Check if requested timer is first of the list of pending timer
    struct kernel_timer *first = (struct kernel_timer *) kernel_env.queue_timer.first;

    // Delay shall not be more than maximum allowed
    if(delay > KERNEL_TIMER_DELAY_MAX)
    {
        delay = KERNEL_TIMER_DELAY_MAX;

    }
    // Delay should not be zero
    else if(delay == 0)
    {
        delay = 1;
    }

    if(first != NULL)
    {
        if ((first->id == timer_id) && (first->task == task_id))
        {
            // Indicate that the HW tid_kernel_timemer will have to be reprogrammed
            hw_prog = true;
        }
    }

    // Extract the timer from the list if required
    struct kernel_timer *timer = (struct kernel_timer*)
        kernel_queue_extract(&kernel_env.queue_timer, cmp_timer_id, (uint32_t)timer_id << 16 | task_id);

    if (timer == NULL)
    {
        // Create new one
        timer = (struct kernel_timer*) kernel_malloc(sizeof(struct kernel_timer), KERNEL_MEM_KERNEL_MSG);
        ASSERT_ERR(timer);
        timer->id = timer_id;
        timer->task = task_id;
    }

    // update characteristics
    abs_time = kernel_time() + delay;
    timer->time = abs_time;


    // Mask calculated time to be sure it's not greater than time counter
    timer->time &= KERNEL_GROSSTIMER_MASK;

    // insert in sorted timer list
    kernel_queue_insert(&kernel_env.queue_timer,
                    (struct common_list_hdr*) timer,
                    cmp_abs_time);

    // check if HW timer set needed
    if (hw_prog || (kernel_env.queue_timer.first == (struct common_list_hdr*) timer))
    {
        kernel_timer_hw_set((struct kernel_timer *)kernel_env.queue_timer.first);
    }

    // Check that the timer did not expire before HW prog
    if (kernel_time_past(abs_time))
    {
        // Timer already expired, so schedule the timer event immediately
        kernel_event_set(KERNEL_EVENT_KERNEL_TIMER);
    }
}

void kernel_timer_clear(kernel_msg_id_t const timer_id, kernel_task_id_t const task_id)
{
    struct kernel_timer *timer = (struct kernel_timer *) kernel_env.queue_timer.first;

    if (kernel_env.queue_timer.first != NULL)
    {
        if ((timer->id == timer_id) && (timer->task == task_id))
        {
            // timer found and first to expire! pop it
            kernel_queue_pop(&kernel_env.queue_timer);

            struct kernel_timer *first = (struct kernel_timer *) kernel_env.queue_timer.first;

            // and set the following timer HW if any
            kernel_timer_hw_set(first);


            // Check that the timer did not expire before HW prog
            if ((first) && kernel_time_past(first->time))
            {
                // Timer already expired, so schedule the timer event immediately
                kernel_event_set(KERNEL_EVENT_KERNEL_TIMER);
            }

            // Check that the timer did not expire before HW prog
            //ASSERT_ERR(!kernel_time_past(first->time));
        }
        else
        {
            timer = (struct kernel_timer *)
                    kernel_queue_extract(&kernel_env.queue_timer, cmp_timer_id,
                            (uint32_t)timer_id << 16 | task_id);
        }

        if (timer != NULL)
        {
            // free the cleared timer
            kernel_free(timer);
        }
    }
}

bool kernel_timer_active(kernel_msg_id_t const timer_id, kernel_task_id_t const task_id)
{
    struct kernel_timer *timer;
    bool result;

    // check the next timer
    timer = (struct kernel_timer*) kernel_env.queue_timer.first;

    /* scan the timer queue to look for a message element with the same id and destination*/
    while (timer != NULL)
    {
        if ((timer->id == timer_id) &&
            (timer->task == task_id) )
        {
            /* Element has been found                                                   */
            break;
        }

        /* Check  next timer                                                            */
        timer = timer->next;
    }

    /* If the element has been found                                                    */
    if (timer != NULL)
        result = true;
    else
        result = false;

    return(result);

}

void kernel_timer_adjust_all(uint32_t delay)
{
    struct kernel_timer *timer;

    // check the next timer
    timer = (struct kernel_timer*) kernel_env.queue_timer.first;

    // iterate through timers, adjust
    while (timer != NULL)
    {
        timer->time += delay;
        timer = timer->next;
    }
}

#if DEEP_SLEEP
uint32_t kernel_timer_target_get(void)
{
    // by default no target expected
    uint32_t res = RWIP_INVALID_TARGET_TIME;

    struct kernel_timer *first = (struct kernel_timer*) kernel_env.queue_timer.first;

    // Check if there is an active timer or not
    if (first != NULL)
    {
        // return target time in slots
        res = RWIP_10MS_TIME_TO_CLOCK(first->time);
    }

    return (res);
}

bool kernel_timer_sleep_check(uint32_t *sleep_duration, uint32_t wakeup_delay)
{
    bool sleep_allowed = true;
    uint16_t current_kernel_time;
    uint32_t current_time, next_tmr_time, delta;
    struct kernel_timer *first = (struct kernel_timer*) kernel_env.queue_timer.first;

    // Check if there is an active timer or not
    if (first != NULL)
    {
        // Get the current time
        #if (EA_PRESENT)
        current_time = ea_time_get_halfslot_rounded();
        #endif //(EA_PRESENT)
        current_kernel_time = kernel_time();

        // Compute time difference to expiry and add it to the current time
        delta = ((uint32_t)((uint16_t)(first->time - current_kernel_time))) << 4;
        #if (BT_EMB_PRESENT)
        next_tmr_time = ((current_time + delta) & ~((uint32_t)0xF)) & MAX_SLOT_CLOCK;
        #elif (BLE_EMB_PRESENT)
        next_tmr_time = ((current_time + delta) & ~((uint32_t)0xF)) & BLE_BASETIMECNT_MASK;
        #endif // BT_EMB_PRESENT / BLE_EMB_PRESENT
		
        // Check if timer expiry is passed
        #if (BT_EMB_PRESENT)
        if(CLK_DIFF(current_time, next_tmr_time) > 0)
        #elif (BLE_EMB_PRESENT)
		if(current_time == next_tmr_time)
		{
			// Wake-up delay does not allow sleep
			return false;
		}
        if(lld_evt_time_cmp(current_time, next_tmr_time))
        #endif // BT_EMB_PRESENT / BLE_EMB_PRESENT
        {
            // Compute sleep duration
            #if (BT_EMB_PRESENT)
            delta = (next_tmr_time - current_time) & MAX_SLOT_CLOCK;
            #elif (BLE_EMB_PRESENT)
            delta = (next_tmr_time - current_time) & BLE_BASETIMECNT_MASK;
            #endif // BT_EMB_PRESENT / BLE_EMB_PRESENT

            // Check if sleep duration is shorter than initial sleep duration
            if (delta < *sleep_duration)
            {
                // Check if sleep duration is longer than wake-up delay
                if(delta > wakeup_delay)
                {
                    // Allowed sleep duration can be replaced
                    *sleep_duration = delta;
                }
                else
                {
                    // Wake-up delay does not allow sleep
                    sleep_allowed = false;
                }
            }
        }
    }

    return sleep_allowed;
}
#endif //DEEP_SLEEP

///@} KERNEL_TIMER
