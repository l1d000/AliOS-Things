/**
 ****************************************************************************************
 *
 * @file l2cc_task.c
 *
 * @brief L2CAP Controller Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CCTASK
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_L2CC)

#if BLE_EMB_PRESENT
#include "em_buf.h"          // stack buffering
#endif //#if BLE_EMB_PRESENT

#include "l2cc_task.h"
#include "l2cc_sig.h"
#include "l2cc_lecb.h"
#include "l2cc_int.h"
#include "l2cc_pdu_int.h"

#include "l2cm_int.h" // Internal API required

#include "gap.h"
#include "gapm.h"
#include "gapc.h"
#include "gattc.h"
#include "gattm.h"

#include "hci.h"

#include "common_math.h"
#include "common_utils.h"

#include "kernel_mem.h"
#include "kernel_event.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/// Calculate minimum of LECB local credit required to be sure that at least one SDU can be received.
#define L2CC_LECB_MIN_CREDIT(sdu_len, mps) ((((sdu_len + L2C_SDU_LEN) + (mps - 1)) / mps) + 1)

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if current operation can be processed or not.
 * if it can be proceed, initialize an operation request.
 * If a command complete event with error code can be triggered.
 *
 * Function returns how the message should be handled by message handler.
 *
 * @param[in] conidx        Connection Index
 * @param[in] op_type       Operation type.
 * @param[in] op_msg        Requested operation message (note op_msg cannot be null)
 * @param[in] supp_ops      Supported operations array.
 *                          Latest array value shall be L2CC_NO_OP.
 *
 * @return operation can be executed if message status equals KERNEL_MSG_NO_FREE,
 * else nothing to do, just exit from the handler.
 ****************************************************************************************
 */
int l2cc_process_op(uint8_t conidx, uint8_t op_type, void* op_msg, enum l2cc_operation* supp_ops)
{
    ASSERT_ERR(op_type < L2CC_OP_MAX);
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED; // Reset State
    // Current process state
    uint8_t state = kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx));
    uint8_t operation = *((uint8_t*)op_msg);

    uint8_t status = GAP_ERR_COMMAND_DISALLOWED;

    /* no operation on going or requested operation is current on going operation. */
    // in a disconnect state, reject all commands
    if(state != L2CC_FREE)
    {
        if(l2cc_get_operation_ptr(conidx, op_type) != op_msg)
        {
            status = GAP_ERR_NO_ERROR;

            // check what to do with command if an operation is ongoing.
            if((state & (1 << op_type)) != L2CC_READY)
            {
                // operation are queued by default
                // save it for later.
                msg_status = KERNEL_MSG_SAVED;
            }
            else
            {
                // check if operation is supported
                while(*supp_ops != L2CC_NO_OP)
                {
                    // operation supported by command
                    if(operation == *supp_ops)
                    {
                        break;
                    }
                    // check next operation
                    else
                    {
                        supp_ops++;
                    }
                }

                // operation not supported
                if(*supp_ops == L2CC_NO_OP)
                {
                    status = GAP_ERR_INVALID_PARAM;
                }
                else
                {
                    // message memory will be managed by GAPM
                    msg_status = KERNEL_MSG_NO_FREE;

                    // store operation
                    l2cc_set_operation_ptr(conidx, op_type, op_msg);

                    // set state to busy
                    l2cc_update_state(conidx, (1 << op_type), true);
                }
            }
        }
        else
        {
            // message memory managed by GAPC
            msg_status = KERNEL_MSG_NO_FREE;
            status = GAP_ERR_NO_ERROR;
        }
    }

    // if an error detected, send command completed with error status
    if(status != GAP_ERR_NO_ERROR)
    {
        l2cc_send_error_evt(conidx, operation, kernel_msg_src_id_get(op_msg), status, L2C_CID_LE_SIGNALING);
    }

    return msg_status;
}


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Data from upper layer handler.
 * This handler will be responsible for the data transmission request from SMP, ATT and GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_L2CC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int l2cc_pdu_send_cmd_handler(kernel_msg_id_t const msgid, struct l2cc_pdu_send_cmd *param,
                                     kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state     = kernel_state_get(dest_id);
    // Status of the message
    int msg_state     = KERNEL_MSG_CONSUMED;
    // Connection index
    uint8_t conidx    = KERNEL_IDX_GET(dest_id);
    uint8_t status    = GAP_ERR_NO_ERROR;

    if(state == L2CC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else if(param->operation != L2CC_PDU_SEND)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        // retrieve message
        struct kernel_msg* msg = kernel_param2msg(param);
        // put at end of the list
        common_list_push_back(&(l2cc_env[conidx]->tx_queue), &(msg->hdr));
        msg_state = KERNEL_MSG_NO_FREE;

        l2cm_tx_status(conidx, true);
        // initialize offset internal variable
        param->offset = 0;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        l2cc_send_error_evt(conidx, param->operation, src_id, status, param->pdu.chan_id);
    }

    // else nothing to do
    return (msg_state);
}


/**
 ****************************************************************************************
 * @brief LECB SDU Data from upper layer handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_L2CC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int l2cc_lecb_sdu_send_cmd_handler(kernel_msg_id_t const msgid, struct l2cc_lecb_sdu_send_cmd *param,
                                     kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state     = kernel_state_get(dest_id);
    // Status of the message
    int msg_state     = KERNEL_MSG_CONSUMED;
    // Connection index
    uint8_t conidx    = KERNEL_IDX_GET(dest_id);
    uint8_t status    = GAP_ERR_NO_ERROR;

    if(state == L2CC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else if(param->operation != L2CC_LECB_SDU_SEND)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    else
    {
        #if (BLE_LECB)
        struct l2cc_sdu* sdu   = &(param->sdu);
        struct l2cc_lecb_info* lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, sdu->cid);

        // check that channel exists
        if(lecb != NULL)
        {
            // check if there is no ongoing transmission
            if(lecb->tx_sdu == NULL)
            {
                // check SDU size which shall not exceed peer MTU
                if(sdu->length > lecb->peer_mtu)
                {
                    status = L2C_ERR_INVALID_MTU_EXCEED;
                }
                else
                {
                    // register the SDU to send
                    lecb->tx_sdu = param;
                }
            }
            else
            {
                // not possible to queue 2 requests on same channel id
                status = GAP_ERR_COMMAND_DISALLOWED;
            }
        }
        else
        {
            status = L2C_ERR_INVALID_CID;
        }
        #else  // !(BLE_LECB)
        status   = GAP_ERR_NOT_SUPPORTED;
        #endif // (BLE_LECB)
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        l2cc_send_error_evt(conidx, param->operation, src_id, status, param->sdu.cid);
    }
    else
    {
        // retrieve message
        struct kernel_msg* msg = kernel_param2msg(param);
        // put at end of the list
        common_list_push_back(&(l2cc_env[conidx]->tx_queue), &(msg->hdr));
        msg_state = KERNEL_MSG_NO_FREE;

        l2cm_tx_status(conidx, true);
        // initialize offset internal variable
        param->offset = 0;
    }

    // else nothing to do
    return (msg_state);
}



/**
 ****************************************************************************************
 * @brief DBG Data from upper layer handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_L2CC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int l2cc_dbg_pdu_send_cmd_handler(kernel_msg_id_t const msgid, struct l2cc_dbg_pdu_send_cmd *param,
                                     kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state     = kernel_state_get(dest_id);
    // Status of the message
    int msg_state     = KERNEL_MSG_CONSUMED;
    // Connection index
    uint8_t conidx    = KERNEL_IDX_GET(dest_id);
    uint8_t status    = GAP_ERR_NO_ERROR;

    if(state == L2CC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else if(param->operation != L2CC_DBG_PDU_SEND)
    {
        status = GAP_ERR_COMMAND_DISALLOWED;
    }
    #if (BLE_DEBUG)
    else if(param->pdu.length > (gapm_get_max_mtu() + L2C_HEADER_LEN + L2C_SDU_LEN))
    {
        status = L2C_ERR_INVALID_MTU_EXCEED;
    }
    #else // !(BLE_DEBUG)
    else
    {
        status = GAP_ERR_NOT_SUPPORTED;
    }
    #endif // (BLE_DEBUG)

    if(status != GAP_ERR_NO_ERROR)
    {
        l2cc_send_error_evt(conidx, param->operation, src_id, status, common_read16p(param->pdu.data));
    }
    else
    {
        // retrieve message
        struct kernel_msg* msg = kernel_param2msg(param);
        // put at end of the list
        common_list_push_back(&(l2cc_env[conidx]->tx_queue), &(msg->hdr));
        msg_state = KERNEL_MSG_NO_FREE;

        l2cm_tx_status(conidx, true);
        // initialize offset internal variable
        param->offset = 0;
    }

    // else nothing to do
    return (msg_state);
}

/**
 ****************************************************************************************
 * @brief Handles number of completed packet events.
 * When the host sends an ACL packet to controller, if the packet is
 * acknowledged, the controller sends this event to the host to indicate
 * that peer link layer has received the packet
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_L2CC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int hci_nb_cmp_pkts_evt_handler(kernel_msg_id_t const msgid,
                                      struct hci_nb_cmp_pkts_evt const *event,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    if(src_id == HCI_NB_CMP_PKTS_EVT_CODE)
    {
        uint16_t old_nb_buffer = l2cm_get_nb_buffer_available();

        /* this will update the number of buffers
         * available per connection handle */
        register unsigned int i = 0;
        for ( i = 0; i < event->nb_of_hdl; i++)
        {
            /* increment the number of buffers available */
            l2cm_buffer_release(event->nb_comp_pkt[i]);
        }

        // check that l2cc states can be updated.
        if (old_nb_buffer == 0)
        {
            // Inform that data TX can be continued
            kernel_event_set(KERNEL_EVENT_L2CAP_TX);
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles data coming from Link Layer.
 * This handler receives the data from lower layer, and then send the data to respective
 * host block.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_L2CC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int hci_acl_data_rx_handler(kernel_msg_id_t const msgid,
                                       struct hci_acl_data_rx const *param,
                                       kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    uint8_t status = GAP_ERR_NO_ERROR;
    switch (state)
    {
        case (L2CC_FREE):
        {
            // Do nothing, the RX buffer will be free
        } break;

        default:
        {
            // Connection Index
            uint8_t conidx = KERNEL_IDX_GET(dest_id);
            // Get the environment
            struct l2cc_env_tag *env = l2cc_env[conidx];
            // LLID
            uint8_t llid = param->pb_bc_flag;
            uint8_t* buffer = NULL;

            #if (BLE_EMB_PRESENT)
            // Get the reception buffer
            buffer = em_buf_rx_buff_addr_get(param->rx_hdl);
            ASSERT_ERR(buffer != NULL);
            #else// (BLE_HOST_PRESENT)
            buffer = param->buffer;
            #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

            ASSERT_ERR(env != NULL);

            switch (llid)
            {
                // First part of the payload
                case L2C_PB_START_FLUSH:
                {
                    // Perform sanity check on PDU header and retrieve buffer used for packet reception
                    status = l2cc_pdu_header_check(conidx, buffer);

                } /* No break*/

                // Fragment received
                case L2C_PB_CONTINUE:
                {
                    // Check if the indication can be sent to the upper layers
                    if (env->rx_buffer)
                    {
                        switch(env->rx_buffer->id)
                        {
                            case L2CC_PDU_RECV_IND:
                            {
                                struct l2cc_pdu_recv_ind* pdu_ind =
                                        (struct l2cc_pdu_recv_ind*) kernel_msg2param(env->rx_buffer);

                                // process reception of message
                                status = l2cc_pdu_unpack(&(pdu_ind->pdu), &(pdu_ind->offset), &(env->rx_pdu_rem_len),
                                                         buffer, param->length, llid);

                                // PDU has been fully received or an error occurs
                                if((status != GAP_ERR_NO_ERROR) || (env->rx_pdu_rem_len == 0))
                                {
                                    env->rx_buffer = NULL;
                                    pdu_ind->status = status;
                                    kernel_msg_send(pdu_ind);
                                }
                            }
                            break;
                            #if (BLE_LECB)
                            case L2CC_LECB_SDU_RECV_IND:
                            {
                                struct l2cc_lecb_sdu_recv_ind* sdu_ind =
                                        (struct l2cc_lecb_sdu_recv_ind*) kernel_msg2param(env->rx_buffer);

                                status = l2cc_lecb_pdu_unpack(&(sdu_ind->sdu), buffer, param->length,
                                                              &(sdu_ind->offset), &(env->rx_pdu_rem_len), llid);

                                // Segment has been fully received
                                if((status != GAP_ERR_NO_ERROR) || (env->rx_pdu_rem_len == 0))
                                {
                                    // Check state of the LECB connection
                                    struct l2cc_lecb_info* lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID,
                                                                                 sdu_ind->sdu.cid);

                                    if((lecb != NULL) && (GETB(lecb->state, L2CC_LECB_CONNECTED)))
                                    {
                                        if(status == GAP_ERR_NO_ERROR)
                                        {
                                            // all segment correctly received, inform application
                                            if((sdu_ind->offset == sdu_ind->sdu.length))
                                            {
                                                lecb->rx_sdu = NULL;
                                                kernel_msg_send(sdu_ind);
                                            }
                                            // else wait for new segment
                                        }
                                        else
                                        {
                                            l2cc_lecb_init_disconnect(conidx, lecb, status);
                                        }
                                    }
                                    else
                                    {
										if(env->rx_buffer != NULL)
										{
											 kernel_free(env->rx_buffer);
										}else
										{
											  //uart_putchar("1");
										}
                                       
                                    }
                                    env->rx_buffer = NULL;
                                }
                            }
                            break;
                            #endif // (BLE_LECB)
                            #if (BLE_DEBUG)
                            case L2CC_DBG_PDU_RECV_IND:
                            {
                                struct l2cc_dbg_pdu_recv_ind* dbg_ind =
                                        (struct l2cc_dbg_pdu_recv_ind*) kernel_msg2param(env->rx_buffer);

                                status = l2cc_dbg_pdu_unpack(&(dbg_ind->pdu), buffer, param->length,
                                                             &(dbg_ind->offset), llid);

                                // PDU has been fully received
                                if((status != GAP_ERR_NO_ERROR) || (dbg_ind->offset == dbg_ind->pdu.length))
                                {
                                    dbg_ind->status = status;
                                    env->rx_buffer = NULL;
                                    kernel_msg_send(dbg_ind);
                                }
                            }
                            break;
                            #endif // (BLE_DEBUG)
                            default:
                            {
                                ASSERT_INFO(0, env->rx_buffer->id, conidx);
                            }
                        }
                    }
                } break;

                case L2C_PB_START_NON_FLUSH:
                default:
                {
                    /* Do nothing */
                } break;
            }
        } break;
    }

    // Free RX buffer.
    #if (BLE_EMB_PRESENT)
    em_buf_rx_free(param->rx_hdl);
    #else// (BLE_HOST_PRESENT)
    if(param->buffer != NULL)
    {
        kernel_free(param->buffer);
    }
    #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

    // Message is consumed
    return (KERNEL_MSG_CONSUMED);
}



#if (BLE_LECB)

/**
 ****************************************************************************************
 * @brief Handles the reception of L2CC_LECB_CONNECT_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_lecb_connect_cmd_handler(kernel_msg_id_t const msgid,
                                                struct l2cc_lecb_connect_cmd *param,
                                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum l2cc_operation supp_ops[] = {L2CC_LECB_CONNECT, L2CC_NO_OP};

    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    // Status
    uint16_t status = GAP_ERR_COMMAND_DISALLOWED;

    // check if operation can be executed
    int msg_status = l2cc_process_op(conidx, L2CC_OP_SIG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        do
        {
            // perform a sanity check of connection parameters
            if (!L2C_IS_VALID_LEPSM(param->le_psm))
            {
                status = L2C_ERR_LEPSM_NOT_SUPP;
                break;
            }

            // Check MTU
            if((param->local_mtu > gapm_get_max_mtu()) || (param->local_mtu < L2C_MIN_LE_MTUSIG))
            {
                status = L2C_ERR_INVALID_MTU_EXCEED;
                break;
            }
            // Check MPS
            if((param->local_mps > gapm_get_max_mps()) || (param->local_mps < L2C_MIN_LE_MTUSIG))
            {
                status = L2C_ERR_INVALID_MPS_EXCEED;
                break;
            }
            // Check MPS
            if((param->local_credit < L2CC_LECB_MIN_CREDIT(param->local_mtu, param->local_mps)))
            {
                status = L2C_ERR_CREDIT_ERROR;
                break;
            }

            // allocate dedicated channel
            if(param->local_cid == 0)
            {
                param->local_cid = L2C_CID_DYN_MIN;

                /* Check that the Channel Id is not used by another LECB connection */
                while (l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, param->local_cid) != NULL)
                {
                    param->local_cid++;
                }
            }

            // check CID range
            if ((!L2C_IS_DYNAMIC_CID(param->local_cid))
                    // Check that the Channel Id is not used by another LECB connection
                    || (l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, param->local_cid) != NULL))
            {
                status = L2C_ERR_INVALID_CID;
                break;
            }

            // Register channel and check if there is enough buffer
            status = gapm_lecb_register(param->le_psm, false);

            if(status == GAP_ERR_NO_ERROR)
            {
                // allocate environment variable for new LECB connection
                struct l2cc_lecb_info* lecb = (struct l2cc_lecb_info*) kernel_malloc(sizeof(struct l2cc_lecb_info), KERNEL_MEM_ATT_DB);

                lecb->le_psm       = param->le_psm;
                lecb->local_cid    = param->local_cid;
                lecb->peer_cid     = 0;
                lecb->local_mtu    = param->local_mtu;
                lecb->local_mps    = param->local_mps;
                lecb->local_credit = param->local_credit;
                lecb->task_id      = src_id;
                lecb->state        = 0;
                lecb->disc_reason  = GAP_ERR_NO_ERROR;
                lecb->tx_sdu       = NULL;
                lecb->rx_sdu       = NULL;

                common_list_push_front(&(l2cc_env[conidx]->lecb_list), &(lecb->hdr));

                // generate packet identifier
                param->pkt_id = common_rand_word() & 0xFF;
                if(param->pkt_id == 0)
                {
                    param->pkt_id = 1;
                }

                l2cc_lecb_send_con_req(conidx, param->pkt_id, lecb->le_psm, lecb->local_cid, lecb->local_credit,
                                       lecb->local_mps, lecb->local_mtu);
            }
        } while (0);

        if (status != GAP_ERR_NO_ERROR)
        {
            /* send command complete event with error code */
            l2cc_send_complete_evt(conidx, L2CC_OP_SIG, status);
        }
    }
    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles the reception of L2CC_LECB_CONNECT_CFM message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_lecb_connect_cfm_handler(kernel_msg_id_t const msgid,
                                                struct l2cc_lecb_connect_cfm *param,
                                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != L2CC_FREE)
    {
        // Current connection index
        uint8_t idx = KERNEL_IDX_GET(dest_id);

        // Search element
        struct l2cc_lecb_info *lecb = l2cc_lecb_find(idx, L2CC_LECB_PEER_CID, param->peer_cid);

        if (lecb != NULL)
        {
            uint8_t status = param->accept ? GAP_ERR_NO_ERROR : L2C_ERR_INSUFF_AUTHOR;

            do
            {
                if(status != GAP_ERR_NO_ERROR)
                {
                    break;
                }

                // perform a sanity check of connection parameters
                if (param->le_psm != lecb->le_psm)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Check that connection not already exist
                if(GETB(lecb->state, L2CC_LECB_CONNECTED))
                {
                    status = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }

                // Check MTU
                if((param->local_mtu > gapm_get_max_mtu()) || (param->local_mtu < L2C_MIN_LE_MTUSIG))
                {
                    status = L2C_ERR_INVALID_MTU_EXCEED;
                    break;
                }
                // Check MPS
                if((param->local_mps > gapm_get_max_mps()) || (param->local_mps < L2C_MIN_LE_MTUSIG))
                {
                    status = L2C_ERR_INVALID_MPS_EXCEED;
                    break;
                }
                // Check MPS
                if((param->local_credit < L2CC_LECB_MIN_CREDIT(param->local_mtu, param->local_mps)))
                {
                    status = L2C_ERR_CREDIT_ERROR;
                    break;
                }

                // allocate dedicated channel
                if(param->local_cid == 0)
                {
                    param->local_cid = L2C_CID_DYN_MIN;

                    /* Check that the Channel Id is not used by another LECB connection */
                    while (l2cc_lecb_find(idx, L2CC_LECB_LOCAL_CID, param->local_cid) != NULL)
                    {
                        param->local_cid++;
                    }
                }

                // check CID range
                if (!L2C_IS_DYNAMIC_CID(param->local_cid))
                {
                    status = L2C_ERR_INVALID_CID;
                    break;
                }

                // Check that the Channel Id is not used by another LECB connection
                if(l2cc_lecb_find(idx, L2CC_LECB_LOCAL_CID, param->local_cid) != NULL)
                {
                    status = L2C_ERR_CID_ALREADY_ALLOC;
                    break;
                }

            } while (0);

            lecb->le_psm       = param->le_psm;
            lecb->local_cid    = param->local_cid;
            lecb->local_mtu    = param->local_mtu;
            lecb->local_mps    = param->local_mps;
            lecb->local_credit = param->local_credit;

            // Send connection response
            l2cc_lecb_send_con_rsp(idx, l2cc_lecb_h2l_err(status), lecb->pkt_id, lecb->local_cid, lecb->local_credit,
                                    lecb->local_mps, lecb->local_mtu);

            if(param->accept)
            {
                // allocate a request indication with credit based connection information
                struct l2cc_lecb_connect_ind* conn_ind
                        = KERNEL_MSG_ALLOC(L2CC_LECB_CONNECT_IND, lecb->task_id, dest_id, l2cc_lecb_connect_ind);

                // Fill up parameters
                conn_ind->le_psm      = lecb->le_psm;
                conn_ind->peer_cid    = lecb->peer_cid;
                conn_ind->status      = status;
                conn_ind->local_cid   = lecb->local_cid;
                conn_ind->peer_credit = lecb->peer_credit;
                conn_ind->peer_mtu    = lecb->peer_mtu;
                conn_ind->peer_mps    = lecb->peer_mps;

                // send the message
                kernel_msg_send(conn_ind);
            }

            if(status == GAP_ERR_NO_ERROR)
            {
                SETB(lecb->state, L2CC_LECB_CONNECTED, true);
            }
            else
            {
                l2cc_lecb_free(idx, lecb, false);
            }
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of L2CC_LECB_DISCONNECT_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_lecb_disconnect_cmd_handler(kernel_msg_id_t const msgid,
                                            struct l2cc_lecb_disconnect_cmd *param,
                                            kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);
    // list of handler supported operations
    enum l2cc_operation supp_ops[] = {L2CC_LECB_DISCONNECT, L2CC_NO_OP};
    // check if operation can be executed
    int msg_status = l2cc_process_op(conidx, L2CC_OP_SIG, param, supp_ops);
    // Status of the operation
    uint8_t status = L2C_ERR_INVALID_CID;

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // Search element
        struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, param->peer_cid);
        if (lecb != NULL)
        {
            // Check status
            if (GETB(lecb->state, L2CC_LECB_CONNECTED) || (lecb->disc_reason != GAP_ERR_NO_ERROR))
            {
                // Generate packet ID// Generate packet ID
                param->pkt_id = common_rand_word() & 0xFF;
                if (param->pkt_id == 0)
                {
                    // ensure that packet identifier is never equals to zero.
                    param->pkt_id = 1;
                }

                l2cc_lecb_send_disc_req(conidx, param->pkt_id, lecb->local_cid, lecb->peer_cid);

                // set status
                if (GETB(lecb->state, L2CC_LECB_CONNECTED))
                {
                    lecb->disc_reason = LL_ERR_CON_TERM_BY_LOCAL_HOST;
                    SETB(lecb->state, L2CC_LECB_CONNECTED, false);
                }

                // Set status
                status = GAP_ERR_NO_ERROR;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            /* send command complete event with error code */
            l2cc_send_complete_evt(conidx, L2CC_OP_SIG, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles the reception of L2CC_SIGNALING_TRANS_TO_IND message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_signaling_trans_to_ind_handler(kernel_msg_id_t const msgid, void const *param,
                                             kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    if(state != L2CC_FREE)
    {
        switch(l2cc_get_operation(conidx, L2CC_OP_SIG))
        {
            case L2CC_LECB_CONNECT:
            {
                // send complete event
                l2cc_send_complete_evt(conidx, L2CC_OP_SIG, GAP_ERR_TIMEOUT);
            } break;
            case L2CC_LECB_DISCONNECT:
            {
                struct l2cc_lecb_disconnect_cmd* cmd = (struct l2cc_lecb_disconnect_cmd*) param;

                struct l2cc_lecb_info* lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, cmd->peer_cid);

                l2cc_lecb_free(conidx, lecb, true);

                // send complete event
                l2cc_send_complete_evt(conidx, L2CC_OP_SIG, GAP_ERR_TIMEOUT);
            } break;
            default: /* Nothing to do */ break;
        }

    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of L2CC_LECB_ADD_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_lecb_add_cmd_handler(kernel_msg_id_t const msgid,
                                     struct l2cc_lecb_add_cmd *param,
                                     kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum l2cc_operation supp_ops[] = {L2CC_LECB_CREDIT_ADD, L2CC_NO_OP};

    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    // check if operation can be executed
    int msg_status = l2cc_process_op(conidx, L2CC_OP_SIG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // Status
        uint8_t status = GAP_ERR_NOT_FOUND;

        // Search element
        struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, param->local_cid);

        if (lecb != NULL)
        {
            if (GETB(lecb->state, L2CC_LECB_CONNECTED))
            {
                status = GAP_ERR_INVALID_PARAM;
                // Check not to exceed credit count
                if ((param->credit != 0) && (param->credit <= (L2C_LECB_MAX_CREDIT - lecb->local_credit)))
                {
                    // generate packet identifier
                    param->pkt_id = common_rand_word() & 0xFF;
                    if (param->pkt_id == 0)
                    {
                        param->pkt_id = 1;
                    }

                    l2cc_lecb_send_credit_add(conidx, param->pkt_id, lecb->local_cid, param->credit);

                    // Save credit count
                    lecb->local_credit += param->credit;

                    status = GAP_ERR_NO_ERROR;
                }
            }
        }

        /* send command complete event with error code */
        l2cc_send_complete_evt(conidx, L2CC_OP_SIG, status);
    }

    return msg_status;
}
#endif // (BLE_LECB)






/**
 ****************************************************************************************
 * @brief Receives a pdu that should be handled by GAP. This PDU is a signaling packet
 * such as Update connection parameters or pdu error.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_pdu_recv_ind_handler(kernel_msg_id_t const msgid, struct l2cc_pdu_recv_ind *ind,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;

    if(state != L2CC_FREE)
    {
        struct l2cc_pdu *pdu = &(ind->pdu);

        switch (pdu->chan_id)
        {
            // Signaling PDU
            case L2C_CID_LE_SIGNALING:
            {
                // No error detected during message reception
                if(ind->status == GAP_ERR_NO_ERROR)
                {
                    // Handles signaling message received
                    msg_status = l2cc_sig_pdu_recv_handler(conidx, pdu);
                }
                // Invalid Channel ID revceived
                else if(ind->status == L2C_ERR_INVALID_CID)
                {
                    /* Invalid CID received  */
                    l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_INVALID_CID,
                            pdu->chan_id, L2C_CID_RESERVED);
                }
                // Packet MTU exceeds maximum allowed size.
                else if(ind->status == L2C_ERR_INVALID_MTU_EXCEED)
                {
                    /* MTU Exceeded  */
                    if(pdu->chan_id == L2C_CID_LE_SIGNALING)
                    {
                        l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_MTU_SIG_EXCEEDED,
                                L2C_MIN_LE_MTUSIG, 0);
                    }
                    else if(pdu->chan_id == L2C_CID_ATTRIBUTE)
                    {
                        l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_MTU_SIG_EXCEEDED,
                                gattc_get_mtu(conidx), 0);
                    }
                    else
                    {
                        l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_MTU_SIG_EXCEEDED,
                                gapm_get_max_mtu(), 0);
                    }
                }
                // PDU received is invalid
                else // L2C_ERR_INVALID_PDU
                {
                    /* PDU invalid, reject command  */
                    if (pdu->data.reject.pkt_id)
                    {
                        l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_CMD_NOT_UNDERSTOOD, 0, 0);
                    }
                }
            } break;

            // Invalid CID
            default:
            {
                if (!L2C_IS_DYNAMIC_CID(pdu->chan_id))
                {
                    /* Invalid CID received  */
                    l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_INVALID_CID,
                            pdu->chan_id, L2C_CID_RESERVED);
                }
            } break;
        }
    }
    /* message is consumed */
    return (msg_status);
}


static int l2cc_cmp_evt_handler(kernel_msg_id_t const msgid, struct l2cc_cmp_evt *ind,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	return KERNEL_MSG_CONSUMED;
}


/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
const struct kernel_msg_handler l2cc_default_state[] =
{
	{ L2CC_CMP_EVT,					    (kernel_msg_func_t) l2cc_cmp_evt_handler },
    { L2CC_PDU_SEND_CMD,                (kernel_msg_func_t) l2cc_pdu_send_cmd_handler },
    { L2CC_LECB_SDU_SEND_CMD,           (kernel_msg_func_t) l2cc_lecb_sdu_send_cmd_handler },
    { L2CC_DBG_PDU_SEND_CMD,            (kernel_msg_func_t) l2cc_dbg_pdu_send_cmd_handler },

    #if (BLE_LECB)
    // LE Credit Based
    { L2CC_LECB_CONNECT_CMD,            (kernel_msg_func_t) l2cc_lecb_connect_cmd_handler},
    { L2CC_LECB_CONNECT_CFM,            (kernel_msg_func_t) l2cc_lecb_connect_cfm_handler},
    { L2CC_LECB_ADD_CMD,                (kernel_msg_func_t) l2cc_lecb_add_cmd_handler},
    { L2CC_LECB_DISCONNECT_CMD,         (kernel_msg_func_t) l2cc_lecb_disconnect_cmd_handler},

    { L2CC_SIGNALING_TRANS_TO_IND,      (kernel_msg_func_t) l2cc_signaling_trans_to_ind_handler},
    #endif // (BLE_LECB)

    { HCI_EVENT,                        (kernel_msg_func_t) hci_nb_cmp_pkts_evt_handler },
    { HCI_ACL_DATA_RX,                  (kernel_msg_func_t) hci_acl_data_rx_handler },

    { L2CC_PDU_RECV_IND,                (kernel_msg_func_t) l2cc_pdu_recv_ind_handler },
};

/// Specifies the message handlers that are common to all states.
const struct kernel_state_handler l2cc_default_handler = KERNEL_STATE_HANDLER(l2cc_default_state);

/// Defines the place holder for the states of all the task instances.
kernel_state_t l2cc_state[BLE_CONNECTION_MAX];




/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void* l2cc_pdu_alloc(uint8_t conidx, uint16_t cid, uint8_t code,  kernel_task_id_t src_id, uint16_t length)
{
    // send connection response
    struct l2cc_pdu_send_cmd *msg = KERNEL_MSG_ALLOC_DYN(L2CC_PDU_SEND_CMD, KERNEL_BUILD_ID(TASK_L2CC, conidx), src_id, l2cc_pdu_send_cmd, length);

    // Set channel/packet ID and OP code
    msg->operation       = L2CC_PDU_SEND;
    msg->pdu.chan_id     = cid;
    msg->pdu.data.code   = code;

    return &(msg->pdu.data.code);
}


void l2cc_pdu_send(void* pdu)
{
    kernel_msg_send(L2CC_PDU_TO_CMD(pdu));
}

#endif //(BLE_L2CC)

/// @} L2CCTASK
