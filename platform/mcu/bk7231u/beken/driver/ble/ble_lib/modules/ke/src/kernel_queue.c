/**
 ****************************************************************************************
 *
 * @file kernel_queue.c
 *
 * @brief This file contains all the functions that handle the different queues
 * (timer queue, save queue, user queue)
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QUEUE
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "kernel_queue.h"      // kernel queues

#include <stddef.h>        // standard definitions
#include <stdint.h>        // standard integer
#include <stdbool.h>       // standard boolean
#include "architect.h"          // architectural definitions
#include "kernel_config.h"     // kernel configuration
#include "kernel_mem.h"        // kernel memory definitions

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
struct common_list_hdr *kernel_queue_extract(struct common_list * const queue,
                                     bool (*func)(struct common_list_hdr const * elmt, uint32_t arg), uint32_t arg)
{
    struct common_list_hdr *prev = NULL;
    struct common_list_hdr *element  = queue->first;

    while (element)
    {
        if (func(element, arg))
        {
            if (prev)
            {
                // second or more
                prev->next = element->next;
            }
            else
            {
                // first message
                queue->first = element->next;
            }

            if (element->next)
            {
                // not the last
                element->next = NULL;
            }
            else
            {
                // last message
                queue->last = prev;
            }

            break;
        }

        prev = element;
        element = element->next;
    }

    #if (KERNEL_PROFILING)
    if(element != NULL)
    {
        queue->cnt--;
        if(queue->mincnt > queue->cnt)
        {
            queue->mincnt = queue->cnt;
        }
    }
    #endif //KERNEL_PROFILING

    return element;
}

void kernel_queue_insert(struct common_list * const queue, struct common_list_hdr * const element,
                     bool (*cmp)(struct common_list_hdr const *elementA, struct common_list_hdr const *elementB))
{
    struct common_list_hdr *prev = NULL;
    struct common_list_hdr *scan = queue->first;

    for(;;)
    {
        // scan the list until the end or cmp() returns true
        if (scan)
        {
            if (cmp(element, scan))
            {
                // insert now
                break;
            }
            prev = scan;
            scan = scan->next;
        }
        else
        {
            // end of list
            queue->last = element;
            break;
        }
    }

    element->next = scan;

    if (prev)
    {
        // second or more
        prev->next = element;
    }
    else
    {
        // first message
        queue->first = element;
    }

    #if (KERNEL_PROFILING)
    queue->cnt++;
    if(queue->maxcnt < queue->cnt)
    {
        queue->maxcnt = queue->cnt;
    }
    #endif //KERNEL_PROFILING
}

/// @} QUEUE
