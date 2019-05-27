/**
 ****************************************************************************************
 *
 * @file ke.c
 *
 * @brief This file contains the kernel definition.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup KE
 * @{
 ****************************************************************************************
 */
   
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"        // stack configuration

#include "architect.h"               // architecture
#include <string.h>             // used for memset.
#include "kernel.h"                 // kernel
#include "kernel_event.h"           // kernel event
#include "kernel_mem.h"             // kernel memory
#include "kernel_msg.h"             // kernel message
#include "kernel_task.h"            // kernel task
#include "kernel_timer.h"           // kernel timer
#include "kernel_env.h"             // kernel environment
#include "kernel_queue.h"           // kernel queue


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Kernel environment
struct kernel_env_tag kernel_env;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void kernel_init(void)
{
    // kernel_mem_init MUST be called first to be able to allocate memory right from start
    #if (KERNEL_MEM_RW)
    memset(kernel_env.heap, 0, sizeof(struct mblock_free*) * KERNEL_MEM_BLOCK_MAX);
    memset(kernel_env.heap_size, 0, sizeof(uint16_t) * KERNEL_MEM_BLOCK_MAX);
    #if (KERNEL_PROFILING)
    kernel_env.max_heap_used = 0;
    memset(kernel_env.heap_used, 0, sizeof(uint16_t) * KERNEL_MEM_BLOCK_MAX);
    #endif //KERNEL_PROFILING
    #endif //KERNEL_MEM_RW

    // Initialize event module
    kernel_event_init();

    // initialize the kernel message queue, mandatory before any message can be transmitted
    kernel_env.queue_saved.first = NULL;
    kernel_env.queue_saved.last = NULL;
    kernel_env.queue_sent.first = NULL;
    kernel_env.queue_sent.last = NULL;
    kernel_env.queue_timer.first = NULL;
    kernel_env.queue_timer.last = NULL;

    // Initialize task module
    kernel_task_init();

    // Initialize timer module
    kernel_timer_init();
}

void kernel_flush(void)
{
    // free all pending message(s)
    while(1)
    {
        struct kernel_msg *msg = (struct kernel_msg*) kernel_queue_pop(&kernel_env.queue_sent);
        if(msg == NULL)
            break;
        kernel_msg_free(msg);
    }
    // free all saved message(s)
    while(1)
    {
        struct kernel_msg *msg = (struct kernel_msg*) kernel_queue_pop(&kernel_env.queue_saved);
        if(msg == NULL)
            break;
        kernel_msg_free(msg);
    }
    // free all timers
    while(1)
    {
        struct kernel_timer *timer = (struct kernel_timer*) kernel_queue_pop(&kernel_env.queue_timer);
        if(timer == NULL)
            break;
        kernel_free(timer);
    }

    // Flush events
    kernel_event_flush();
}

bool kernel_sleep_check(void)
{
    return (kernel_event_get_all() == 0);
}

#if (KERNEL_PROFILING)
enum KERNEL_STATUS kernel_stats_get(uint8_t* max_msg_sent,
                uint8_t* max_msg_saved,
                uint8_t* max_timer_used,
                uint16_t* max_heap_used)
{
    *max_msg_sent   = kernel_env.queue_sent.maxcnt;
    *max_msg_saved  = kernel_env.queue_saved.maxcnt;
    *max_timer_used = kernel_env.queue_timer.maxcnt;
    *max_heap_used  = kernel_env.max_heap_used;

    return KERNEL_SUCCESS;
}
#endif //KERNEL_PROFILING

///@} KE
