/**
 ****************************************************************************************
 *
 * @file kernel_msg.c
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
 * @addtogroup MSG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>          // standard definition
#include <stdint.h>          // standard integer definition
#include <stdbool.h>         // standard boolean definition
#include <string.h>          // string definition
#include "architect.h"            // platform architecture define

#include "rwip_config.h"     // stack configuration

#include "kernel_config.h"       // kernel configuration
#include "kernel_mem.h"          // kernel memory defines
#include "kernel_msg.h"          // kernel message defines
#include "kernel_task.h"         // kernel task defines
#include "kernel_queue.h"        // kernel queue defines

#if (KERNEL_FULL)
#include "kernel_event.h"        // kernel event defines
#include "kernel_env.h"          // kernel environment defines
#endif /* KERNEL_FULL */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void *kernel_msg_alloc(kernel_msg_id_t const id, kernel_task_id_t const dest_id,
                   kernel_task_id_t const src_id, uint16_t const param_len)
{
    struct kernel_msg *msg = (struct kernel_msg*) kernel_malloc(sizeof(struct kernel_msg) +
                                                    param_len - sizeof (uint32_t), KERNEL_MEM_KERNEL_MSG);
    void *param_ptr = NULL;

    ASSERT_ERR(msg != NULL);
    msg->hdr.next  = KERNEL_MSG_NOT_IN_QUEUE;
    msg->id        = id;
    msg->dest_id   = dest_id;
    msg->src_id    = src_id;
    msg->param_len = param_len;

    param_ptr = kernel_msg2param(msg);

    memset(param_ptr, 0, param_len);

    return param_ptr;
}

void kernel_msg_send(void const *param_ptr)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);
    // Enqueue the message. Protect against IRQs as messages can be sent from IRQ
    GLOBAL_INT_DIS();
    kernel_queue_push(&kernel_env.queue_sent, (struct common_list_hdr*)msg);
    GLOBAL_INT_RES();

    // trigger the event
    kernel_event_set(KERNEL_EVENT_KERNEL_MESSAGE);
}

void kernel_msg_send_basic(kernel_msg_id_t const id, kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    void *no_param = kernel_msg_alloc(id, dest_id, src_id, 0);
    kernel_msg_send(no_param);
}

void kernel_msg_forward(void const *param_ptr, kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);

    // update the source and destination of the message
    msg->dest_id = dest_id;
    msg->src_id  = src_id;

    // send the message
    kernel_msg_send(param_ptr);
}

void kernel_msg_forward_new_id(void const *param_ptr,
                           kernel_msg_id_t const msg_id, kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);

    // update the id of the message
    msg->id      = msg_id;
    // update the source and destination of the message
    msg->dest_id = dest_id;
    msg->src_id  = src_id;

    // send the message
    kernel_msg_send(param_ptr);
}

void kernel_msg_free(struct kernel_msg const *msg)
{
    kernel_free( (void*) msg);
}


kernel_msg_id_t kernel_msg_dest_id_get(void const *param_ptr)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);

    return msg->dest_id;
}


kernel_msg_id_t kernel_msg_src_id_get(void const *param_ptr)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);

    return msg->src_id;
}


bool kernel_msg_in_queue(void const *param_ptr)
{
    struct kernel_msg * msg = kernel_param2msg(param_ptr);

    return (msg->hdr.next != KERNEL_MSG_NOT_IN_QUEUE);
}
/// @} MSG
