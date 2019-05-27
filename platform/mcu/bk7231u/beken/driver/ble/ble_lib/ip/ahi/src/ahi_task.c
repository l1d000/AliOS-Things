/**
 ****************************************************************************************
 *
 * @file ahi_task.c
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AHI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)


#include "ahi.h"
#include "ahi_task.h"
#include "kernel_msg.h"          // kernel message defines
#include "gapm.h"

/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static void ahi_kernel_msg_tx_done(uint8_t* tx_data)
{
    // retrieve message pointer
    struct kernel_msg* msg = (struct kernel_msg*) (tx_data - sizeof(struct common_list_hdr));

    // free it.
    kernel_msg_free(msg);
}


/**
 ****************************************************************************************
 * @brief Function called to send a message through UART.
 *
 * @param[in]  msgid   U16 message id from kernel_msg.
 * @param[in] *param   Pointer to parameters of the message in kernel_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KERNEL_MSG_NO_FREE.
 *****************************************************************************************
 */
static int ahi_msg_send_handler (kernel_msg_id_t const msgid,
                          void *param,
                          kernel_task_id_t const dest_id,
                          kernel_task_id_t const src_id)
{
    //extract the kernel_msg pointer from the param passed and push it in AHI queue
    struct kernel_msg *msg = kernel_param2msg(param);

    // Update source and dest task number with task identifiers
    msg->src_id  = gapm_get_id_from_task(msg->src_id);
    msg->dest_id = gapm_get_id_from_task(msg->dest_id);

    // request to send the message.
    ahi_send_msg(AHI_KERNEL_MSG_TYPE, msg->param_len+AHI_KERNEL_MSG_HDR_LEN, (uint8_t *)&(msg->id),  &ahi_kernel_msg_tx_done);

    //return NO_FREE always since ahi_eif_write handles the freeing
    return KERNEL_MSG_NO_FREE;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
const struct kernel_msg_handler ahi_default_state[] =
{

    /** Default handler for AHI TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KERNEL_MSG_DEFAULT_HANDLER,  (kernel_msg_func_t)ahi_msg_send_handler},
};

/// Specifies the message handlers that are common to all states.
const struct kernel_state_handler ahi_default_handler = KERNEL_STATE_HANDLER(ahi_default_state);

/// Defines the placeholder for the states of all the task instances.
kernel_state_t ahi_state[AHI_IDX_MAX];


#endif //AHI_TL_SUPPORT

/// @} AHI
