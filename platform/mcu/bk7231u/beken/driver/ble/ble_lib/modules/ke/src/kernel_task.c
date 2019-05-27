/**
 ****************************************************************************************
 *
 * @file kernel_task.c
 *
 * @brief This file contains the implementation of the kernel task management.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"       // stack configuration

#include <stddef.h>            // standard definition
#include <stdint.h>            // standard integer
#include <stdbool.h>           // standard boolean
#include <string.h>            // memcpy defintion

#include "kernel_config.h"         // kernel configuration
#include "kernel_event.h"          // kernel event
#include "kernel_mem.h"            // kernel memory
#include "kernel_task.h"           // kernel task
#include "kernel_env.h"            // kernel environment
#include "kernel_queue.h"          // kernel queue
#include "dbg_swdiag.h"        // Software diag



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINTIONS
 ****************************************************************************************
 */

/// KE TASK element structure
struct kernel_task_elem
{
    struct kernel_task_desc const * p_desc;
};

/// KE TASK environment structure
struct kernel_task_env_tag
{
    struct kernel_task_elem task_list[TASK_BLE_MAX];
};


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// KE TASK environment
static struct kernel_task_env_tag kernel_task_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Compare destination task callback.
 *
 * @param[in] msg          kernel message
 * @param[in] dest_id      destination id
 *
 * @return bool
 ****************************************************************************************
 */
static bool cmp_dest_id(struct common_list_hdr const * msg, uint32_t dest_id)
{
    return ((struct kernel_msg*)msg)->dest_id == dest_id;
}

/**
 ****************************************************************************************
 * @brief Reactivation of saved messages.
 *
 * This primitive looks for all the messages destined to the task kernel_task_id that
 * have been saved and inserts them into the sent priority queue. These
 * messages will be scheduled at the next scheduler pass.
 *
 * @param[in] kernel_task_id    Destination Identifier
 ****************************************************************************************
 */
static void kernel_task_saved_update(kernel_task_id_t const kernel_task_id)
{
    struct kernel_msg * msg;

    for(;;)
    {
        // if the state has changed look in the Save queue if a message
        // need to be handled
        msg = (struct kernel_msg*) kernel_queue_extract(&kernel_env.queue_saved,
                                                &cmp_dest_id,
                                                (uint32_t) kernel_task_id);

        if (msg == NULL) break;

        // Insert it back in the sent queue
        GLOBAL_INT_DIS();
        kernel_queue_push(&kernel_env.queue_sent, (struct common_list_hdr*)msg);
        GLOBAL_INT_RES();

        // trigger the event
        kernel_event_set(KERNEL_EVENT_KERNEL_MESSAGE);
    }

    return;
}


/**
 ****************************************************************************************
 * @brief Search message handler function matching the msg id
 *
 * @param[in] msg_id        Message identifier
 * @param[in] state_handler Pointer to the state handler
 *
 * @return                  Pointer to the message handler (NULL if not found)
 *
 ****************************************************************************************
 */
static kernel_msg_func_t kernel_handler_search(kernel_msg_id_t const msg_id, struct kernel_state_handler const *state_handler)
{

	int i = 0;
    // Get the message handler function by parsing the message table
    for (i = (state_handler->msg_cnt-1); 0 <= i; i--)
    {
        if ((state_handler->msg_table[i].id == msg_id)
                || (state_handler->msg_table[i].id == KERNEL_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(state_handler->msg_table[i].func);

            return state_handler->msg_table[i].func;
        }
    }

    // If we execute this line of code, it means that we did not find the handler
    return NULL;
}

/**
 ****************************************************************************************
 * @brief Retrieve appropriate message handler function of a task
 *
 * @param[in]  msg_id   Message identifier
 * @param[in]  task_id  Task instance identifier
 *
 * @return              Pointer to the message handler (NULL if not found)
 *
 ****************************************************************************************
 */
static kernel_msg_func_t kernel_task_handler_get(kernel_msg_id_t const msg_id, kernel_task_id_t const task_id)
{
    kernel_msg_func_t func = NULL;
    int idx = KERNEL_IDX_GET(task_id);
    int type = KERNEL_TYPE_GET(task_id);
    struct kernel_task_desc const * p_task_desc = NULL;
    ASSERT_INFO(type < TASK_BLE_MAX, msg_id, type);

    p_task_desc = kernel_task_env.task_list[type].p_desc;

    // inactive task have no instance, drop message automatically for inactive tasks
    if(p_task_desc->idx_max != 0)
    {
        ASSERT_INFO(p_task_desc != NULL, task_id, msg_id);
        ASSERT_INFO((idx < p_task_desc->idx_max), task_id, msg_id);
				
        // If the idx found is out of range return NULL
        if(idx < p_task_desc->idx_max)
        {
            // Retrieve a pointer to the task instance data
            if (p_task_desc->state_handler)
            {
                func = kernel_handler_search(msg_id, p_task_desc->state_handler + p_task_desc->state[idx]);
            }

            // No handler... need to retrieve the default one
            if (func == NULL && p_task_desc->default_handler)
            {
                func = kernel_handler_search(msg_id, p_task_desc->default_handler);
            }
        }
    }

    return func;
}


/**
 ****************************************************************************************
 * @brief Scheduler entry point.
 *
 * This function is the scheduler of messages. It tries to get a message
 * from the sent queue, then try to get the appropriate message handler
 * function (from the current state, or the default one). This function
 * is called, then the message is saved or freed.
 ****************************************************************************************
 */
static void kernel_task_schedule(void)
{
    DBG_SWDIAG(EVT, MESSAGE, 1);
    // Process one message at a time to ensure that events having higher priority are
    // handled in time
    do
    {
        int msg_status;
        struct kernel_msg *msg;
        // Get a message from the queue
        GLOBAL_INT_DIS();
        msg = (struct kernel_msg*) kernel_queue_pop(&kernel_env.queue_sent);
        GLOBAL_INT_RES();
        if (msg == NULL) break;
        // Mark that message is no more in message queue
        msg->hdr.next = KERNEL_MSG_NOT_IN_QUEUE;

        // check if message already free
        if(kernel_is_free(msg))
        {
            ASSERT_INFO(0,msg->id, msg->dest_id);
            break;
        }

        // Retrieve a pointer to the task instance data
        kernel_msg_func_t func = kernel_task_handler_get(msg->id, msg->dest_id);

        // sanity check
        ASSERT_WARN(func != NULL, msg->id, msg->dest_id);

        // Call the message handler
        if (func != NULL)
        {
            msg_status = func(msg->id, kernel_msg2param(msg), msg->dest_id, msg->src_id);
        }
        else
        {
            msg_status = KERNEL_MSG_CONSUMED;
        }

        switch (msg_status)
        {
        case KERNEL_MSG_CONSUMED:
            // Free the message
            kernel_msg_free(msg);
            break;

        case KERNEL_MSG_NO_FREE:
            break;

        case KERNEL_MSG_SAVED:
            // The message has been saved
            // Insert it at the end of the save queue
            kernel_queue_push(&kernel_env.queue_saved, (struct common_list_hdr*) msg);
            break;

        default:
            ASSERT_ERR(0);
            break;
        } // switch case
    } while(0);

    // Verify if we can clear the event bit
    GLOBAL_INT_DIS();
    if (common_list_is_empty(&kernel_env.queue_sent))
        kernel_event_clear(KERNEL_EVENT_KERNEL_MESSAGE);
    GLOBAL_INT_RES();
    DBG_SWDIAG(EVT, MESSAGE, 0);
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void kernel_task_init(void)
{
    memset(&kernel_task_env, 0, sizeof(kernel_task_env));

    // Register message event
    kernel_event_callback_set(KERNEL_EVENT_KERNEL_MESSAGE, &kernel_task_schedule);
}

uint8_t kernel_task_create(uint8_t task_type, struct kernel_task_desc const * p_task_desc)
{
    uint8_t status = KERNEL_TASK_OK;

    GLOBAL_INT_DIS();

    do
    {
        ASSERT_INFO(task_type < TASK_BLE_MAX, task_type, TASK_MAX);

        if(task_type >= TASK_BLE_MAX)
        {
            status = KERNEL_TASK_CAPA_EXCEEDED;
            break;
        }

        if(kernel_task_env.task_list[task_type].p_desc != NULL)
        {
            status = KERNEL_TASK_ALREADY_EXISTS;
            break;
        }

        // Save task descriptor
        kernel_task_env.task_list[task_type].p_desc = p_task_desc;
    } while(0);

    GLOBAL_INT_RES();

    return (status);
}

uint8_t kernel_task_delete(uint8_t task_type)
{
    uint8_t status = KERNEL_TASK_OK;
    GLOBAL_INT_DIS();

    do
    {
        if(task_type >= TASK_BLE_MAX)
        {
            status = KERNEL_TASK_UNKNOWN;
            break;
        }

        // Clear task descriptor
        kernel_task_env.task_list[task_type].p_desc = NULL;
    } while(0);

    GLOBAL_INT_RES();

    return (status);
}


void kernel_state_set(kernel_task_id_t const id, kernel_state_t const state_id)
{
    int idx = KERNEL_IDX_GET(id);
    int type = KERNEL_TYPE_GET(id);
    struct kernel_task_desc const * p_task_desc = NULL;
    kernel_state_t *kernel_stateid_ptr = NULL;

    // sanity checks
    ASSERT_ERR(type < TASK_BLE_MAX);

    if(type < TASK_BLE_MAX)
    {
        p_task_desc = kernel_task_env.task_list[type].p_desc;
    }

    ASSERT_INFO(p_task_desc != NULL, type, idx);
    ASSERT_INFO((idx < p_task_desc->idx_max), idx, p_task_desc->idx_max);

    // If the idx found is out of range return NULL
    if(idx < p_task_desc->idx_max)
    {
        // Get the state
        kernel_stateid_ptr = &p_task_desc->state[idx];

        ASSERT_ERR(kernel_stateid_ptr);

        // set the state
        if (*kernel_stateid_ptr != state_id)
        {
            *kernel_stateid_ptr = state_id;

            // if the state has changed update the SAVE queue
            kernel_task_saved_update(id);
        }
    }
}


kernel_state_t kernel_state_get(kernel_task_id_t const id)
{
    int idx = KERNEL_IDX_GET(id);
    int type = KERNEL_TYPE_GET(id);
    struct kernel_task_desc const * p_task_desc = NULL;
    kernel_state_t state = 0xFF;

    ASSERT_ERR(type < TASK_BLE_MAX);

    if(type < TASK_BLE_MAX)
    {
        p_task_desc = kernel_task_env.task_list[type].p_desc;
    }

    ASSERT_INFO(p_task_desc != NULL, type, idx);
    ASSERT_INFO((idx < p_task_desc->idx_max), idx, p_task_desc->idx_max);

    // If the idx found is out of range return NULL
    if(idx < p_task_desc->idx_max)
    {
        state = p_task_desc->state[idx];
    }

    // Get the state
    return state;
}

int kernel_msg_discard(kernel_msg_id_t const msgid, void const *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    return (KERNEL_MSG_CONSUMED);
}

int kernel_msg_save(kernel_msg_id_t const msgid, void const *param,
                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    return KERNEL_MSG_SAVED;
}



void kernel_task_msg_flush(kernel_task_id_t task)
{
    // Free messages in sent queue
    struct kernel_msg *msg = (struct kernel_msg*) common_list_pick(&kernel_env.queue_sent);
    struct kernel_msg *msg_to_delete = NULL;
    bool s_queue_flushed = false;

    while(msg != NULL)
    {
        while(msg != NULL)
        {
            // check if current message have to be handled by task to flush
            if(KERNEL_TYPE_GET(msg->dest_id) == task)
            {
                // update message pointer
                msg_to_delete = msg;
                msg = (struct kernel_msg *)msg->hdr.next;

                // delete message
                common_list_extract(&kernel_env.queue_sent, &(msg_to_delete->hdr), 0);
                kernel_msg_free(msg_to_delete);
            }
            else
            {
                // go to next message
                msg = (struct kernel_msg *)msg->hdr.next;
            }
        }

        // Flush messages in save queue
        if(!s_queue_flushed)
        {
            msg = (struct kernel_msg*) common_list_pick(&kernel_env.queue_saved);
            s_queue_flushed = true;
        }
    }
}



kernel_task_id_t kernel_task_check(kernel_task_id_t task)
{
    uint8_t idx  = KERNEL_IDX_GET(task);
    uint8_t type = KERNEL_TYPE_GET(task);

    // check if task type exist
    if(   (type > TASK_BLE_MAX)
       || (kernel_task_env.task_list[type].p_desc == NULL)
    // check that task instance exist
       || (idx > kernel_task_env.task_list[type].p_desc->idx_max))
    {
        task = TASK_BLE_NONE;
    }

    return task;
}

/// @} TASK
