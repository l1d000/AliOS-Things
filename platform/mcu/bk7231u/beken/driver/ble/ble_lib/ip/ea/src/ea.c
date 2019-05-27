/**
 ****************************************************************************************
 *
 * @file ea.c
 *
 * @brief Event Arbiter module
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup EA main module
 * @ingroup EA
 * @brief The EA main module.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (EA_PRESENT)

#include <string.h>          // For mem* functions
#include "common_bt.h"
#include "common_math.h"
#include "common_list.h"
#include "kernel_mem.h"
#include "ea.h"
#include "lowlevel.h"

#if (BT_EMB_PRESENT)
#include "reg_btcore.h"
#elif (BLE_EMB_PRESENT)
#include "reg_blecore.h"
#endif //(BT_EMB_PRESENT)

#include "rwip.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Undefined timestamp value
#define EA_UNDEF_TIME           (0xFFFFFFFF)

/// If the Fine counter is close to the greater slot boundary add 1 slot
#if (BT_EMB_PRESENT)
#define EA_CHECK_HALFSLOT_BOUNDARY(void)    ((bt_finetimecnt_get() < (624 >> 1)) ? 1 : 0)
#define EA_CHECK_SLOT_BOUNDARY(void)        ((bt_finetimecnt_get() < 106) ? 1 : 0)
#else //BT_EMB_PRESENT
#define EA_CHECK_HALFSLOT_BOUNDARY(void)    ((ble_finetimecnt_get() < (624 >> 1)) ? 1 : 0)
#define EA_CHECK_SLOT_BOUNDARY(void)        ((ble_finetimecnt_get() < 106) ? 1 : 0)
#endif //BT_EMB_PRESENT

/// Maximum difference between 2 timestamps (half of the range)
#define EA_MAX_INTERVAL_TIME    ((MAX_SLOT_CLOCK >> 1) - 1)

/// Delay needed for programming the HW timer
#define EA_TIMER_PROG_DELAY           (1)


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Definitions of conflicts between time reservations
enum ea_conflict
{
    START_BEFORE_END_BEFORE,
    START_BEFORE_END_DURING,
    START_BEFORE_END_AFTER,
    START_DURING_END_DURING,
    START_DURING_END_AFTER,
    START_AFTER_END_AFTER,
};


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */
/// Event Arbiter Environment
struct ea_env_tag
{
    /// List of pending element to program
    struct common_list elt_wait;
    /// element programmed
    struct ea_elt_tag *elt_prog;
    /// List of element canceled
    struct common_list elt_canceled;
    /// List of element pending to be freed
    struct common_list interval_list;
    #if (EA_ALARM_SUPPORT)
    /// List of alarms
    struct common_list alarm_list;
    #endif //(EA_ALARM_SUPPORT)

    /// programmed fine timer target
    uint32_t finetarget_time;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Event arbiter environment variable
static struct ea_env_tag ea_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check conflicts between 2 events
 *
 * The function select the appropriate conflict type among the followings:
 *    - A finishes before B starts
 *    - A starts before B and finishes inside B (partial overlap)
 *    - A starts before B and finishes after B (B is included in A)
 *    - A starts and finishes during B (A is included in B)
 *    - A starts during B and finishes after B (partial overlap)
 *    - A starts after B finishes
 *
 * @param[in] evt_a  Event A
 * @param[in] evt_b  Event B
 *
 * @return conflict
 ****************************************************************************************
 */
static uint8_t ea_conflict_check(struct ea_elt_tag * evt_a, struct ea_elt_tag * evt_b)
{
    uint8_t conflict;

    /*
     * Terminology:
     *  sa => start A
     *  ea => end A
     *  sb => start B
     *  eb => end B
     */

    do
    {
        // Distance from start A to start B
        int32_t diff_sa_sb_slot = CLK_DIFF(evt_a->timestamp, evt_b->timestamp);

        // A starts before B starts
        if( (diff_sa_sb_slot > 0) || ((diff_sa_sb_slot == 0) && (evt_a->delay < evt_b->delay)) )
        {
            // Distance from end A to start B
            int32_t diff_ea_sb_slot;
            int32_t diff_ea_sb_us = evt_a->delay + evt_a->duration_min - evt_b->delay;
            if(diff_ea_sb_us > 0)
            {
                diff_ea_sb_slot = diff_sa_sb_slot - ((diff_ea_sb_us - 1) / SLOT_SIZE);
            }
            else
            {
                diff_ea_sb_slot = diff_sa_sb_slot - (diff_ea_sb_us / SLOT_SIZE) + 1;
            }

            // A ends before B starts
            if(diff_ea_sb_slot > 0)
            {
                conflict = START_BEFORE_END_BEFORE;
                break;
            }
            else
            {
                // Distance from end A to end B
                int32_t diff_ea_eb_slot;
                int32_t diff_ea_eb_us = evt_a->delay + evt_a->duration_min - evt_b->delay - evt_b->duration_min;
                if(diff_ea_eb_us > 0)
                {
                    diff_ea_eb_slot = diff_sa_sb_slot - ((diff_ea_eb_us - 1) / SLOT_SIZE);
                }
                else
                {
                    diff_ea_eb_slot = diff_sa_sb_slot - (diff_ea_eb_us / SLOT_SIZE) + 1;
                }

                // A ends during B starts
                if(diff_ea_eb_slot > 0)
                {
                    conflict = START_BEFORE_END_DURING;
                }
                else
                {
                    conflict = START_BEFORE_END_AFTER;
                }
                break;
            }
        }
        // B starts before A starts
        else
        {
            // Distance from start A to end B
            int32_t diff_sa_eb_slot;
            int32_t diff_sa_eb_us = evt_b->delay + evt_b->duration_min - evt_a->delay;
            if(diff_sa_eb_us > 0)
            {
                diff_sa_eb_slot = diff_sa_sb_slot + ((diff_sa_eb_us - 1) / SLOT_SIZE) + 1;
            }
            else
            {
                diff_sa_eb_slot = diff_sa_sb_slot + (diff_sa_eb_us / SLOT_SIZE);
            }

            // A starts after B ends
            if(diff_sa_eb_slot <= 0)
            {
                conflict = START_AFTER_END_AFTER;
                break;
            }
            else
            {
                // Distance from end A to end B
                int32_t diff_ea_eb_slot;
                int32_t diff_ea_eb_us = evt_a->delay + evt_a->duration_min - evt_b->delay - evt_b->duration_min;
                if(diff_ea_eb_us > 0)
                {
                    diff_ea_eb_slot = diff_sa_sb_slot - ((diff_ea_eb_us - 1) / SLOT_SIZE);
                }
                else
                {
                    diff_ea_eb_slot = diff_sa_sb_slot - (diff_ea_eb_us / SLOT_SIZE) + 1;
                }

                // A ends during B starts
                if(diff_ea_eb_slot > 0)
                {
                    conflict = START_DURING_END_DURING;
                }
                else
                {
                    conflict = START_DURING_END_AFTER;
                }
                break;
            }
        }
    } while (0);

    return conflict;
}

/**
 ****************************************************************************************
 * @brief Cancel elements
 ****************************************************************************************
 */
void ea_elt_cancel(struct ea_elt_tag *new_elt)
{
    struct ea_elt_tag* cancel_prev = NULL;
    struct ea_elt_tag* cancel_curr = (struct ea_elt_tag*) common_list_pick(&ea_env.elt_canceled);

    // Try to reschedule each element one-by-one
    while(cancel_curr)
    {
        uint8_t resched_att = EA_ASAP_STG_RESCHED_ATT_GET(cancel_curr);

        struct ea_elt_tag* cancel_next = (struct ea_elt_tag*) common_list_next(&cancel_curr->hdr);

        // Check ASAP flag
        if( (EA_ASAP_STG_TYPE_GET(cancel_curr) != EA_FLAG_NO_ASAP) && (resched_att > 0) )
        {
            uint8_t status = EA_ERROR_OK;
            struct ea_elt_tag *wait_curr = new_elt;
            struct ea_elt_tag *wait_prev = NULL;

            // Increment priority
            cancel_curr->current_prio += EA_ASAP_STG_PRIO_INC_GET(cancel_curr);

            // Decrement number of rescheduling attempts
            EA_ASAP_STG_RESCHED_ATT_SET(cancel_curr, resched_att-1);

            // Scan the wait list
            while (wait_curr)
            {
                // Compare element boundaries
                uint8_t conflict = ea_conflict_check(cancel_curr, wait_curr);

                if(conflict == START_BEFORE_END_BEFORE)
                {
                    break;
                }
                else if(conflict == START_AFTER_END_AFTER)
                {
                    // do nothing
                }
                else
                {
                    uint8_t parity;

                    // Move the current element after the scan element
                    uint32_t shift_us = (wait_curr->delay + wait_curr->duration_min);
                    uint32_t new_timestamp = wait_curr->timestamp + (shift_us/625);
                    if(cancel_curr->delay < (shift_us % SLOT_SIZE))
                    {
                        new_timestamp += 1;
                    }

                    // Check parity
                    parity = EA_ASAP_STG_PARITY_GET(cancel_curr);
                    if(parity == EA_ODD_SLOT)
                    {
                        new_timestamp = COMMON_ALIGN2_LO(new_timestamp) + 1;
                    }
                    else if(parity == EA_EVEN_SLOT)
                    {
                        new_timestamp = COMMON_ALIGN2_HI(new_timestamp);
                    }

                    cancel_curr->timestamp = new_timestamp & MAX_SLOT_CLOCK;

                    // Check in ASAP case if the limit has been reached
                    if(   (EA_ASAP_STG_TYPE_GET(cancel_curr) >= EA_FLAG_ASAP_LIMIT)
                       && (CLK_DIFF(cancel_curr->timestamp, cancel_curr->asap_limit) < ((int32_t)(cancel_curr->duration_min/SLOT_SIZE)) )  )
                    {
                        status = EA_ERROR_REJECTED;
                        break;
                    }
                }

                // Get next element from wait list
                wait_prev = wait_curr;
                wait_curr = (struct ea_elt_tag *) common_list_next(&wait_curr->hdr);
            }

            if(status == EA_ERROR_OK)
            {
                // Remove from cancel list
                common_list_extract_after(&ea_env.elt_canceled, &cancel_prev->hdr, &cancel_curr->hdr);

                // Place in wait list
                common_list_insert_after(&ea_env.elt_wait, &wait_prev->hdr, &cancel_curr->hdr);
            }
            else
            {
                cancel_prev = cancel_curr;
            }
        }
        else
        {
            cancel_prev = cancel_curr;
        }

        // Move to next element from cancel list
        cancel_curr = cancel_next;
    }


    // Check if cancel list is still not empty
    if (!common_list_is_empty(&ea_env.elt_canceled))
    {
        #if BT_EMB_PRESENT
        bt_rwbtcntl_swint_req_setf(1);
        #elif BLE_EMB_PRESENT
        ble_swint_req_setf(1);
        #endif //BT_EMB_PRESENT
    }
}

/**
 ****************************************************************************************
 * @brief Compare absolute times
 *
 * The absolute time difference between time1 and time2 is supposed to be less than the
 * maximum interval time
 *
 * @param[in] time1 First time to compare
 * @param[in] time2 Second time to compare
 *
 * @return true if time1 is smaller than time2.
 ****************************************************************************************
 */
__INLINE bool ea_time_cmp(uint32_t time1, uint32_t time2)
{
    return (((time1 - time2) & MAX_SLOT_CLOCK) > EA_MAX_INTERVAL_TIME);
}

/**
 ****************************************************************************************
 * @brief API to try to check if a timer should be started
 ****************************************************************************************
 */
static void ea_prog_timer(void)
{
    ea_env.finetarget_time = EA_UNDEF_TIME;

    // Get current time
    uint32_t current_time = ea_time_get_slot_rounded();

    // Pick the element in the pending list
    struct ea_elt_tag *first_elt = (struct ea_elt_tag *)common_list_pick(&ea_env.elt_wait);
    // Check if list is empty
    if(first_elt != NULL)
    {
        struct ea_elt_tag *current_elt = ea_env.elt_prog;

        // Get the start notification time of the 1st element in the pending list
        ea_env.finetarget_time =  CLK_SUB(first_elt->timestamp, first_elt->start_latency);

        // Check is prevent stop timer should be started
        if(current_elt != NULL)
        {
            // Take the stop notification delay according to the priority between current and next event
            uint8_t stop_notification_delay = ((current_elt->current_prio >= first_elt->current_prio) || (EA_ASAP_STG_TO_PROTECT_GET(first_elt) == 0)) ?
                                                 current_elt->stop_latency1 : current_elt->stop_latency2;

            if(stop_notification_delay > first_elt->start_latency)
            {
                ea_env.finetarget_time = CLK_SUB(first_elt->timestamp, stop_notification_delay);

                // Delayed stop indication to the next slot
                if(CLK_DIFF(current_time, ea_env.finetarget_time) <= 0)
                {
                    ea_env.finetarget_time = CLK_ADD_2(current_time, 1);
                }
            }
        }
    }

    #if (EA_ALARM_SUPPORT)
    {
        // Check the 1st alarm
        struct ea_alarm_tag *alarm = (struct ea_alarm_tag *) common_list_pick(&ea_env.alarm_list);
        if(alarm != NULL)
        {
            // Check the alarm expiry timestamp
            if((ea_env.finetarget_time == EA_UNDEF_TIME) || CLK_DIFF(ea_env.finetarget_time, alarm->timestamp) < 0)
            {
                ea_env.finetarget_time = alarm->timestamp;
            }
        }
    }
    #endif //(EA_ALARM_SUPPORT)

    // If no fine target timer to be set
    if(ea_env.finetarget_time == EA_UNDEF_TIME)
    {
        #if (BT_EMB_PRESENT)
        bt_intcntl_finetgtintmsk_setf(0);
        bt_intack_finetgtintack_clearf(1);
        #else
        ble_finetgtimintmsk_setf(0);
        ble_intack_clear(BLE_FINETGTIMINTACK_BIT);
        #endif
    }
    else
    {
        // Check if the target is possible
        if(CLK_DIFF(current_time, ea_env.finetarget_time) > 0)
        {
            #if (BT_EMB_PRESENT)
            // Program the event target time in HW
            bt_finetimtgt_finetarget_setf(ea_env.finetarget_time);

            // Enable fine timer irq
            if (!bt_intcntl_finetgtintmsk_getf())
            {
                /*
                 * If timer is not enabled, it is possible that the irq is raised
                 * due to a spurious value, so ack it before
                 */
                bt_intack_finetgtintack_clearf(1);
                bt_intcntl_finetgtintmsk_setf(1);
            }
            #else
            // Program the event target time in HW
            ble_finetarget_setf(ea_env.finetarget_time);
            // Enable fine timer irq
            if (!ble_finetgtimintmsk_getf())
            {
              /*
               * If timer is not enabled, it is possible that the irq is raised
               * due to a spurious value, so ack it before
               */
              ble_intack_clear(BLE_FINETGTIMINTACK_BIT);
              ble_finetgtimintmsk_setf(1);
            }
            #endif //(BT_EMB_PRESENT)
        }
        else
        {
            ea_env.finetarget_time = EA_UNDEF_TIME;
            #if (BT_EMB_PRESENT)
            ASSERT_ERR(bt_intcntl_finetgtintmsk_getf() == 1);
            #elif (BLE_EMB_PRESENT)
            ASSERT_ERR(ble_finetgtimintmsk_getf() == 1);
            #endif //(BT_EMB_PRESENT)
        }
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void ea_init(bool reset)
{
    common_list_init(&ea_env.elt_wait);
    common_list_init(&ea_env.elt_canceled);
    common_list_init(&ea_env.interval_list);
    #if (EA_ALARM_SUPPORT)
    common_list_init(&ea_env.alarm_list);
    #endif //(EA_ALARM_SUPPORT)

    ea_env.elt_prog = NULL;

    ea_env.finetarget_time = EA_UNDEF_TIME;
}

struct ea_elt_tag *ea_elt_create(uint16_t size_of_env)
{
    // Create element
    struct ea_elt_tag *elt = (struct ea_elt_tag *)kernel_malloc((sizeof(struct ea_elt_tag) + size_of_env),
            KERNEL_MEM_ENV);

    ASSERT_ERR(elt);

    if (elt)
    {
        // Reset element memory
        memset(elt, 0, (sizeof(struct ea_elt_tag) + size_of_env));
    }

    return (elt);
}

uint8_t ea_elt_insert (struct ea_elt_tag *elt)
{
    // List of element waiting for programming
    struct common_list *list = &ea_env.elt_wait;
    struct ea_elt_tag *prev = NULL;
    struct ea_elt_tag *cancel = NULL;
    struct ea_elt_tag *scan = NULL;
	
    // Returned status
    uint8_t status = EA_ERROR_OK;
    // Number of canceled elements
    uint8_t cancel_cpt = 0;

    // Get current time
    uint32_t current_time = ea_time_get_halfslot_rounded();
    GLOBAL_INT_DIS();
    for (;;)
    {
        uint8_t parity;
        // Compare the TS requested and the 1st possible TS from now
        uint32_t first_allowed_ts = CLK_ADD_3(current_time, elt->start_latency, EA_TIMER_PROG_DELAY);
        if(ea_env.elt_prog && (ea_env.elt_prog->stop_latency1 != 0))
        {
            // Take the small stop notification delay into account (minimum delay to stop the ongoing activity)
            uint8_t stop_notification_delay = ea_env.elt_prog->stop_latency1;
            if(CLK_DIFF(current_time, first_allowed_ts) < stop_notification_delay)
            {
                first_allowed_ts = CLK_ADD_2(current_time, stop_notification_delay);
            }
        }

        // Check parity
        if(EA_ASAP_STG_TYPE_GET(elt) != EA_FLAG_NO_ASAP)
        {
            parity = EA_ASAP_STG_PARITY_GET(elt);
            if(parity == EA_ODD_SLOT)
            {
                first_allowed_ts = (COMMON_ALIGN2_LO(first_allowed_ts) + 1) & MAX_SLOT_CLOCK;
            }
            else if(parity == EA_EVEN_SLOT)
            {
                first_allowed_ts = COMMON_ALIGN2_HI(first_allowed_ts) & MAX_SLOT_CLOCK;
            }
        }

        if (CLK_DIFF(first_allowed_ts, elt->timestamp) < 0)
        {
            // If the element should not be programmed ASAP
            if(EA_ASAP_STG_TYPE_GET(elt) == EA_FLAG_NO_ASAP)
            {
                status = EA_ERROR_REJECTED;
                break;
            }
            else
            {
                elt->timestamp  = first_allowed_ts;
            }
        }
        // Check in ASAP case if the limit has been reached
        if(   (EA_ASAP_STG_TYPE_GET(elt) >= EA_FLAG_ASAP_LIMIT)
           && (CLK_DIFF(elt->timestamp, elt->asap_limit) < ((int32_t)(elt->duration_min/SLOT_SIZE)) )  )
        {
            status = EA_ERROR_REJECTED;
            break;
        }
        scan = (ea_env.elt_prog) ? ea_env.elt_prog : (struct ea_elt_tag *)list->first;
        // scan the list until the end or cmp() returns true
        while (scan)
        {
            uint8_t conflict = ea_conflict_check(elt, scan);

            if(conflict == START_BEFORE_END_BEFORE)
            {
                break;
            }
            else if(conflict == START_AFTER_END_AFTER)
            {
                // do nothing
            }
            else
            {
                // Check priority only if the scan element is not the programmed one
                if((elt->current_prio > scan->current_prio) && (scan != ea_env.elt_prog))
                {
                    // Save the start of the element(s) to remove
                    if(cancel_cpt == 0)
                    {
                        cancel = scan;
                    }
                    cancel_cpt++;
                    //If new element finishes before scanned element exit the loop
                    if((conflict == START_BEFORE_END_DURING) || (conflict == START_DURING_END_DURING))
                    {
                        break;
                    }
                }
                else
                {
                    // Check ASAP flag
                    if(EA_ASAP_STG_TYPE_GET(elt) != EA_FLAG_NO_ASAP)
                    {
                        uint32_t shift_us = (scan->delay + scan->duration_min);
                        uint32_t new_timestamp = scan->timestamp + (shift_us/625);
                        if(elt->delay < (shift_us % SLOT_SIZE))
                        {
                            new_timestamp += 1;
                        }

                        // Check parity
                        parity = EA_ASAP_STG_PARITY_GET(elt);
                        if(parity == EA_ODD_SLOT)
                        {
                            new_timestamp = COMMON_ALIGN2_LO(new_timestamp) + 1;
                        }
                        else if(parity == EA_EVEN_SLOT)
                        {
                            new_timestamp = COMMON_ALIGN2_HI(new_timestamp);
                        }

                        elt->timestamp = new_timestamp & MAX_SLOT_CLOCK;

                        //If the element can be reprogrammed ASAP and we have already taken the decision to cancel some elements
                        //Do not cancel those elements
                        cancel_cpt = 0;
                        cancel = NULL;
                    }
                    else
                    {
                        status = EA_ERROR_REJECTED;
                        break;
                    }
                }
            }

            // Check in ASAP case if the limit has been reached
            if(   (EA_ASAP_STG_TYPE_GET(elt) >= EA_FLAG_ASAP_LIMIT)
               && (CLK_DIFF(elt->timestamp, elt->asap_limit) < ((int32_t)(elt->duration_min/SLOT_SIZE)) )  )
            {
                status = EA_ERROR_REJECTED;
                break;
            }

            // Is the scanned element is in the wait list
            if (scan != ea_env.elt_prog)
            {
                //If elements should be canceled do not update the previous to avoid issue during insertion
                if(cancel_cpt == 0)
                {
                    prev = scan;
                }

                // Jump to next element in wait list
                scan = (struct ea_elt_tag *) common_list_next(&scan->hdr);
            }
            else // If the scanned element was the programmed one, check the wait list
            {
                scan = (struct ea_elt_tag *) common_list_pick(list);
            }
        }
        break;
    }

    if (status != EA_ERROR_REJECTED)
    {
        if(cancel_cpt)
        {
            // Remove the elements from wait list
            common_list_extract(&ea_env.elt_wait, &cancel->hdr, cancel_cpt-1);

            // Push the elements to cancel list
            while(cancel_cpt > 0)
            {
                struct ea_elt_tag *cancel_next = (struct ea_elt_tag *)cancel->hdr.next;

                common_list_push_back(&ea_env.elt_canceled, &cancel->hdr);

                cancel = cancel_next;
                cancel_cpt--;
            }
        }

        // Check the position of the new event
        if (prev)
        {
            // Second or more
            common_list_insert_after(&ea_env.elt_wait,&prev->hdr,&elt->hdr);
        }
        else
        {
            // First
            common_list_push_front(&ea_env.elt_wait,&elt->hdr);

            // Reprogram timer if needed
            ea_prog_timer();
        }

        // Check if some events have been removed from schedule
        if (!common_list_is_empty(&ea_env.elt_canceled))
        {
            // Reschedule or cancel the events
            ea_elt_cancel(elt);
        }
    }
//		if( a < 100)
//		{
//			UART_PRINTF("ea_elt_insert elt->timestamp = 0x%08x\r\n",elt->timestamp);
//			a++;
//		}
		

    GLOBAL_INT_RES();

    return (status);
}


uint8_t ea_elt_remove(struct ea_elt_tag *elt)
{
	
	 
    // Returned status
    uint8_t status = EA_ERROR_OK;

    GLOBAL_INT_DIS();

    // If the element is not NULL
    if (elt)
    {
        // Check the wait queue first
        do
        {
            // check if the element is the current in the ea
            if(elt == ea_env.elt_prog)
            {
                ea_env.elt_prog = NULL;
                break;
            }

            // If the element is the first and the fine target timer is started, stop it
            if (&elt->hdr == common_list_pick(&ea_env.elt_wait))
            {
                // Pop the first element of the list
                common_list_pop_front(&ea_env.elt_wait);

                // Update timer if needed
                ea_prog_timer();

                break;
            }

            // Try extracting the element from the wait queue
            if(common_list_extract(&ea_env.elt_wait, &elt->hdr, 0))
                break;

            // Try extracting the element from the cancel queue
            if(common_list_extract(&ea_env.elt_canceled, &elt->hdr, 0))
                break;

            // Element has not been found
            status = EA_ERROR_NOT_FOUND;
        } while (0);
    }

    GLOBAL_INT_RES();

    return (status);
}

struct ea_interval_tag *ea_interval_create(void)
{
    struct ea_interval_tag *new_intv = (struct ea_interval_tag *)kernel_malloc(sizeof(struct ea_interval_tag), KERNEL_MEM_ENV);
    memset(new_intv,0,sizeof(struct ea_interval_tag));
    return (new_intv);
}

void ea_interval_insert(struct ea_interval_tag *interval_to_add)
{
    common_list_push_back(&ea_env.interval_list,&interval_to_add->hdr);
}

void ea_interval_remove(struct ea_interval_tag *interval_to_remove)
{
    // Extract the interval linked to this element
    common_list_extract(&ea_env.interval_list, &interval_to_remove->hdr, 0);
}

void ea_interval_delete(struct ea_interval_tag *interval_to_remove)
{
    if(interval_to_remove != NULL)
    {
        // Extract the interval linked to this element
        common_list_extract(&ea_env.interval_list, &interval_to_remove->hdr, 0);
        // Free the memory
        kernel_free(interval_to_remove);
    }
}

void ea_finetimer_isr(void)
{
    // Get next element
    struct ea_elt_tag *next_elt = (struct ea_elt_tag *)common_list_pick(&ea_env.elt_wait);
    struct ea_elt_tag *current_elt = ea_env.elt_prog;
    uint32_t current_time = ea_time_get_halfslot_rounded();

    // Potentially stop the current event
    if((current_elt) && (next_elt))
    {
        // Retrieve the correct threshold
        uint8_t current_threshold = ((current_elt->current_prio >= next_elt->current_prio) || (EA_ASAP_STG_TO_PROTECT_GET(next_elt) == 0)) ?
                current_elt->stop_latency1 : current_elt->stop_latency2;

        // Check if it is time to notify the current event to stop
        if(CLK_DIFF(current_time, next_elt->timestamp) <= current_threshold)
        {
            if(current_elt->ea_cb_stop != NULL)
            {
                // Call back the end of schedule function
                current_elt->ea_cb_stop(current_elt);
            }

            // Remove the current element
            ea_env.elt_prog = NULL;
        }
    }

    // Potentially cancel the waiting elements that are planned for the past
    while(next_elt != NULL)
    {
        if(ea_time_cmp((next_elt->timestamp - next_elt->start_latency) & MAX_SLOT_CLOCK, current_time))
        {
            // Pop the element
            next_elt = (struct ea_elt_tag *)common_list_pop_front(&ea_env.elt_wait);

            // Push element in the canceled list
            common_list_push_back(&ea_env.elt_canceled, &next_elt->hdr);

            //Pick the next element
            next_elt = (struct ea_elt_tag *)common_list_pick(&ea_env.elt_wait);

            if(next_elt != NULL)
            {
                continue;
            }
        }
        break;
    };

    // Check if the element should be programmed soon
    if (next_elt)
    {
        // if the expiration time is now
        if (((next_elt->timestamp - next_elt->start_latency)&MAX_SLOT_CLOCK) == current_time)
        {
            // Pop the element
            next_elt = (struct ea_elt_tag *)common_list_pop_front(&ea_env.elt_wait);

            // If a current event is still present, it is notified to stop (before the start notification of the waiting one)
            if(ea_env.elt_prog != NULL)
            {
                if(ea_env.elt_prog->ea_cb_stop != NULL)
                {
                    // Call back the end of schedule function
                    ea_env.elt_prog->ea_cb_stop(current_elt);
                }
            }

            //Set the current element programmed
            ea_env.elt_prog = next_elt;

            if(next_elt->ea_cb_start != NULL)
            {
                // Call back the end of schedule function
                next_elt->ea_cb_start(next_elt);
            }
         }
    }
    // Check if element(s) has(have) been canceled
    if(!common_list_is_empty(&ea_env.elt_canceled))
    {
        #if BT_EMB_PRESENT
        bt_rwbtcntl_swint_req_setf(1);
        #elif BLE_EMB_PRESENT
        ble_swint_req_setf(1);
        #endif //BT_EMB_PRESENT
    }

    #if (EA_ALARM_SUPPORT)
    {
        // Trigger expired alarms
        struct ea_alarm_tag *alarm_next;
        struct ea_alarm_tag *alarm = (struct ea_alarm_tag *) common_list_pick(&ea_env.alarm_list);
        while(alarm != NULL)
        {
            // Check the alarm expiry timestamp
            if(CLK_DIFF(current_time, alarm->timestamp) <= 0)
            {
                // Save next
                alarm_next = (struct ea_alarm_tag*) common_list_next((struct common_list_hdr *) alarm);

                // Pop the alarm
                common_list_pop_front(&ea_env.alarm_list);

                // Invoke call back function
                if(alarm->ea_cb_alarm != NULL)
                {
                    alarm->ea_cb_alarm(alarm);
                }
                else
                {
                    ASSERT_ERR(0);
                }

                // Jump to next
                alarm = alarm_next;
            }
            else
            {
                break;
            }
        }
    }
    #endif //(EA_ALARM_SUPPORT)

    // Check if the base time timer should be started
    ea_prog_timer();
}

void ea_sw_isr(void)
{
    // Pop the element in the pending list
    struct ea_elt_tag *elt = NULL;

    while (!common_list_is_empty(&ea_env.elt_canceled))
    {
        elt = (struct ea_elt_tag *)common_list_pop_front(&ea_env.elt_canceled);

        if(elt->ea_cb_cancel != NULL)
        {
            // Call back the cancel function
            elt->ea_cb_cancel(elt);
        }
    }
}

// Choose an appropriate offset for the event
uint8_t ea_offset_req(struct ea_param_input* input_param, struct ea_param_output* output_param)
{
    uint8_t status = EA_ERROR_OK;

    struct ea_interval_tag *scan = NULL;
    uint16_t newoffset, newperiod, newsize, minperiod, event_start, event_end, newstart, newend, temp;

    newperiod = output_param->interval;
    newsize = output_param->duration;

    if ((newperiod == 0) || (newsize == 0))
        return(EA_ERROR_REJECTED);

    // If the requested action is to check a particular offset, perform the checks for this value only
    // Else, start from offset 0 (even value) or offset 1 (odd value) and continue until an offset is found or all the offsets have been exhausted
    if (input_param->action == EA_PARAM_REQ_CHECK)
        newoffset = input_param->offset;
    else if (!input_param->odd_offset)
        newoffset = 0;
    else
        newoffset = 1;

    scan = (struct ea_interval_tag *) common_list_pick(&ea_env.interval_list);

    // Main loop, look for an offset or check if the one provided is ok
    // If newoffset >= newperiod then we have failed
    while ((scan != NULL) && (newoffset < newperiod) && (input_param->role != UNKNOWN_ROLE))
    {
        if ((scan->interval_used > 0) && (scan->bandwidth_used > 0) && (scan->conhdl_used != input_param->conhdl))
        {
            minperiod = (newperiod <= scan->interval_used) ? newperiod : scan->interval_used;

            // Check if we are dealing with a scatternet here, in which case "padding" is added
            if ( (scan->role_used != input_param->role) ||
                    ((scan->role_used == input_param->role) && (input_param->role == SLAVE_ROLE) && (scan->linkid != input_param->linkid)) )
            {
                if ((scan->bandwidth_used + newsize) > minperiod)
			//	 if ((scan->bandwidth_used + newsize + EA_BW_USED_DFT_SLOT) > minperiod)

                    return(EA_ERROR_BW_FULL);

                event_start = (scan->offset_used + minperiod - 1) % minperiod;
                event_end = event_start + scan->bandwidth_used + 1;
            }
            else
            {
                if ((scan->bandwidth_used + newsize) > minperiod)
                    return(EA_ERROR_BW_FULL);

                event_start = scan->offset_used % minperiod;
                event_end = event_start + scan->bandwidth_used - 1;
            }

            // If the intervals of the events compared are not multiples then skip all checks and move on to the next element
            if ((newperiod % minperiod) || (scan->interval_used % minperiod))
            {
                scan = (struct ea_interval_tag *)scan->hdr.next;
                continue;
            }

            newstart = newoffset % minperiod;
            newend = newstart + newsize - 1;

            // The following if statement checks whether the events overlap, taking into account wrap-around effects

            // Check if beginning of event is in another event`
            if ( ((newstart >= event_start) && (newstart <= event_end)) ||
                    // Check if end of event is in another event
                    ((newend >= event_start) && (newend <= event_end)) ||
                    // Check if event envelops another event
                    ((newstart <= event_start) && (newend >= event_end)) ||
                    //Check if the other event wraps around
                    ((event_end >= minperiod) && (newstart <= (event_end % minperiod))) ||
                    // Check if this event wraps around
                    ((newend >= minperiod) && (event_start <= (newend % minperiod))) )
            {
                if (input_param->action == EA_PARAM_REQ_CHECK)
                    // Check for the offset provided has failed, so exit immediately
                    newoffset = newperiod;
                else
                {
                    // Find the next candidate
                    temp = (event_end + 1) % minperiod;
                    while (temp <= newoffset)
                        temp += minperiod;

                    newoffset = temp;

                    // If the new offset does not agree with the "odd_offset" setting, increment by 1
                    if ((input_param->odd_offset && !(newoffset & 0x01)) || (!input_param->odd_offset && (newoffset & 0x01)))
                        newoffset++;
                }

                // Go back to the beginning of the list and repeat the checks using the new offset (newoffset)
                scan = (struct ea_interval_tag *) common_list_pick(&ea_env.interval_list);
                continue;
            }
        }

        // Proceed to the next element
        scan = (struct ea_interval_tag *)scan->hdr.next;
    }

    if (newoffset >= newperiod)
    {
        status = EA_ERROR_BW_FULL;
    }
    else
    {
        output_param->offset = newoffset;
    }

    return(status);
}


uint32_t ea_time_get_halfslot_rounded(void)
{
    #if (BT_EMB_PRESENT)
    //Sample the base time count
    bt_slotclk_samp_setf(1);
    while (bt_slotclk_samp_getf());

    // Read current time in HW
    return ((bt_slotclk_sclk_getf() + 1) >> 1) & MAX_SLOT_CLOCK;
    #else
    //Sample the base time count
    ble_samp_setf(1);
    while (ble_samp_getf());

    // Read current time in HW
    return ((ble_basetimecnt_get() + EA_CHECK_HALFSLOT_BOUNDARY()) & MAX_SLOT_CLOCK);
    #endif
}

uint32_t ea_time_get_slot_rounded(void)
{
    #if (BT_EMB_PRESENT)
    //Sample the base time count
    bt_slotclk_samp_setf(1);
    while (bt_slotclk_samp_getf());

    // Read current time in HW
    return (((bt_slotclk_sclk_getf() >> 1) + EA_CHECK_SLOT_BOUNDARY()) & MAX_SLOT_CLOCK);
    #else
    //Sample the base time count
    ble_samp_setf(1);
    while (ble_samp_getf());

    // Read current time in HW
    return ((ble_basetimecnt_get() + EA_CHECK_SLOT_BOUNDARY()) & MAX_SLOT_CLOCK);
    #endif
}


uint32_t ea_timer_target_get(uint32_t current_time)
{
     // by default no target expected
    uint32_t res = RWIP_INVALID_TARGET_TIME;


    if (ea_env.elt_prog != NULL)
    {
        res = current_time;
    }
    else if(ea_env.finetarget_time != EA_UNDEF_TIME)
    {
        res = ea_env.finetarget_time;
    }

    return (res);
}
bool ea_sleep_check(uint32_t *sleep_duration, uint32_t wakeup_delay)
{
    bool sleep_allowed = false;
    uint32_t current_time;
    uint32_t duration_min;
    uint32_t next_evt_time;
    // Take first element in list of element waiting for programming
    struct ea_elt_tag *elt = (struct ea_elt_tag *)common_list_pick(&ea_env.elt_wait);

    do
    {
        // Check if an event is programmed in the HW or not. If yes, no need to do any
        // additional check, as we cannot go to deep sleep in that case
        if (ea_env.elt_prog != NULL)
            break;

        // Check if there will be an event in the future
        if(elt == NULL)
        {
            // No event planned in the future, so allow the sleep
            sleep_allowed = true;
            break;
        }
        // Get the current time
        current_time = ea_time_get_halfslot_rounded();

        // Get the time to the next event (BLE)
        next_evt_time = (elt->timestamp - elt->start_latency - EA_CLOCK_CORR_LAT)& MAX_SLOT_CLOCK;
		
        // Check if we have enough time to go to sleep
        if (ea_time_cmp(current_time, next_evt_time))
        {
            // Compute sleep duration_min allowed for BLE
            duration_min = (uint32_t)((next_evt_time - current_time)& MAX_SLOT_CLOCK);

            // Check if sleep duration_min is longer than wake-up delay
            if (duration_min > wakeup_delay)
            {
                // Sleep is allowed
                sleep_allowed = true;

                // Check if sleep duration_min is shorter than initial sleep duration_min
                if(duration_min < *sleep_duration)
                {
                    // Allowed sleep duration_min can be replaced
                    *sleep_duration = duration_min;
                }
            }
        }
    
    } while (0);

    return (sleep_allowed);
}

void ea_interval_duration_req(struct ea_param_input* input_param, struct ea_param_output* output_param)
{
    output_param->interval = input_param->interval_max;

    // Check if a range of possible intervals is provided, else do nothing
    if (input_param->interval_min < input_param->interval_max)
    {
        uint16_t multiple = 0xFFFF;
        uint16_t interval_min = 0xFFFF;
        uint8_t nb_links = 0;
        struct ea_interval_tag *scan = (struct ea_interval_tag *) common_list_pick(&ea_env.interval_list);

        // Find the minimal interval and the total number of links
        while (scan != NULL)
        {
            nb_links++;

            if ((scan->conhdl_used != input_param->conhdl) && (scan->interval_used < interval_min))
            {
                interval_min = scan->interval_used;
            }

            // Proceed to the next element
            scan = (struct ea_interval_tag *)scan->hdr.next;
        }

        // If this is the only link, take preferred periodicity into account
        if (nb_links < 2)
        {
            if (input_param->pref_period != 0)
            {
                //Calculate a multiple of preferred periodicity
                multiple = /*floor*/(input_param->interval_max / input_param->pref_period);
                multiple *= input_param->pref_period;
            }
        }
        else if (input_param->interval_max >= interval_min)
        {
            //Calculate a multiple between existing and requested interval
            multiple = /*floor*/(input_param->interval_max / interval_min);
            multiple *= interval_min;
        }
        else // interval_min > input_param->interval_max
        {
            uint16_t candidate = input_param->interval_max;
            bool found = false;

            while ((candidate >= input_param->interval_min) && (!found))
            {
                if ((interval_min % candidate) == 0)
                {
                    multiple = candidate;
                    found = true;
                }

                candidate -= 2;
            }
        }

        //Check if the computed interval matches the requirements
        if ((input_param->interval_min <= multiple) && (multiple <= input_param->interval_max))
        {
            output_param->interval = multiple;
        }
    }

    // If the ce length min is less than 2 set it to 1.25ms to avoid scheduling overlapping
    output_param->duration = common_max(input_param->duration_min, EA_BW_USED_DFT_SLOT);
}

#if (EA_ALARM_SUPPORT)
uint8_t ea_alarm_set(struct ea_alarm_tag* elt)
{
    uint8_t status = EA_ERROR_REJECTED;

    // Get current time
    uint32_t current_time = ea_time_get_halfslot_rounded();

    GLOBAL_INT_DIS();

    if(CLK_DIFF(current_time, elt->timestamp) >= EA_TIMER_PROG_DELAY)
    {
        struct ea_alarm_tag* scan_prev = NULL;
        struct ea_alarm_tag* scan_next = (struct ea_alarm_tag*) common_list_pick(&ea_env.alarm_list);

        // Find the place to insert
        while(scan_next != NULL)
        {
            if(CLK_DIFF(scan_next->timestamp, elt->timestamp) < 0)
            {
                break;
            }

            // Jump to next element
            scan_prev = scan_next;
            scan_next = (struct ea_alarm_tag*) common_list_next((struct common_list_hdr*) scan_next);
        }

        // Insert into alarm list
        if(scan_prev == NULL)
        {
            // Push as first element of the empty list
            common_list_push_front(&ea_env.alarm_list, (struct common_list_hdr*) elt);

            // Update timer if needed
            ea_prog_timer();
        }
        else if(scan_next == NULL)
        {
            // Push as last element of the list
            common_list_push_back(&ea_env.alarm_list, (struct common_list_hdr*) elt);
        }
        else
        {
            // Insert between previous and next
            scan_prev->hdr.next = (struct common_list_hdr*) elt;
            elt->hdr.next = (struct common_list_hdr*) scan_next;
        }

        status = EA_ERROR_OK;
    }

    GLOBAL_INT_RES();

    return (status);
}

uint8_t ea_alarm_clear(struct ea_alarm_tag* elt)
{
    uint8_t status = EA_ERROR_OK;

    GLOBAL_INT_DIS();

    do
    {
        if(((struct common_list_hdr*)elt) == common_list_pick(&ea_env.alarm_list))
        {
            // Pop the first element of the list
            common_list_pop_front(&ea_env.alarm_list);

            // Update timer if needed
            ea_prog_timer();

            break;
        }

        if(common_list_extract(&ea_env.alarm_list, (struct common_list_hdr*) elt, 0))
            break;

        status = EA_ERROR_NOT_FOUND;
    } while(0);

    GLOBAL_INT_RES();

    return (status);
}
#endif //(EA_ALARM_SUPPORT)

#endif //(EA_PRESENT)

///@} EA
