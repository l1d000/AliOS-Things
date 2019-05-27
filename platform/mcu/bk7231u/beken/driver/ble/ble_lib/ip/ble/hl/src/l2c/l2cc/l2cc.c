/**
 ****************************************************************************************
 *
 * @file l2cc.c
 *
 * @brief L2CAP Controller implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup L2CC
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_L2CC)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "l2cc_int.h"
#include "l2cc_pdu_int.h"
#include "l2cc_task.h"
#include "l2cc_lecb.h"
#include "l2cm_int.h" // Internal API required

#include "kernel_timer.h"
#include "kernel_mem.h"

#include "gapc.h"
#include "hci.h"

#include "common_math.h"
#include "common_utils.h"

#if BLE_EMB_PRESENT
#include "em_buf.h"          // stack buffering
#endif //#if BLE_EMB_PRESENT


#include <string.h>


/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

extern const struct kernel_state_handler l2cc_default_handler;
extern kernel_state_t l2cc_state[L2CC_IDX_MAX];


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// L2CAP environments pool
struct l2cc_env_tag *l2cc_env[L2CC_IDX_MAX];

/// L2CC task descriptor
static const struct kernel_task_desc TASK_DESC_L2CC = {NULL, &l2cc_default_handler, l2cc_state, L2CC_STATE_MAX, L2CC_IDX_MAX};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void l2cc_init(bool reset)
{
    // Index
    uint8_t conidx;

    if(!reset)
    {
        // Create L2CC task
        kernel_task_create(TASK_L2CC, &TASK_DESC_L2CC);

        // Initialize env structure
        for(conidx = 0 ; conidx < L2CC_IDX_MAX ; conidx++)
        {
            l2cc_env[conidx] = NULL;
        }
    }
    else
    {
        // Initialize L2Cap controllers
        for (conidx = 0; conidx < L2CC_IDX_MAX; conidx++)
        {
            // clean environment variables
            l2cc_cleanup(conidx, true);
        }
    }
}

void l2cc_create(uint8_t conidx)
{
    struct l2cc_env_tag * env;

    ASSERT_ERR(l2cc_env[conidx] == NULL);

    // set device into a non free state
    kernel_state_set(KERNEL_BUILD_ID(TASK_L2CC, conidx), L2CC_READY);


    // Allocate environment variable for connection
    env = (struct l2cc_env_tag *)kernel_malloc(sizeof(struct l2cc_env_tag), KERNEL_MEM_ENV);

    ASSERT_ERR(env != NULL);

    if(env != NULL)
    {
        /// initialize operation pointer
        uint8_t i;
        for(i = 0 ; i < L2CC_OP_MAX ; i++)
        {
            env->operation[i] = NULL;
        }

        // Clean-up structure
        common_list_init(&env->tx_queue);
        env->rx_buffer = NULL;

        #if (BLE_LECB)
        // Initialization of the message list
        common_list_init(&env->lecb_list);
        #endif //  (BLE_LECB)
    }

    l2cc_env[conidx] = env;
}

void l2cc_cleanup(uint8_t conidx, bool reset)
{
    struct l2cc_env_tag * env = l2cc_env[conidx];

    // cleanup environment variable for connection
    if(l2cc_env[conidx] != NULL)
    {
        /// clean-up operations
        kernel_timer_clear(L2CC_SIGNALING_TRANS_TO_IND, KERNEL_BUILD_ID(TASK_L2CC, conidx));

        uint8_t i;
        for(i = 0 ; i < L2CC_OP_MAX ; i++)
        {
            if(env->operation[i] != NULL)
            {
                /// initialize operation pointer
                if(!reset)
                {
                    l2cc_send_complete_evt(conidx, i, GAP_ERR_DISCONNECTED);
                }
                else
                {
                    struct kernel_msg* cmd = kernel_param2msg(env->operation[i]);
                    // clean-up operation parameters
                    kernel_msg_free(cmd);
                }

            }
        }

        #if (BLE_LECB)
        // free environment variable
        while(!common_list_is_empty(&(env->lecb_list)))
        {
            struct l2cc_lecb_info *lecb = (struct l2cc_lecb_info*) common_list_pick(&(env->lecb_list));

            l2cc_lecb_free(conidx, lecb, !reset);
        }
        #endif // (BLE_LECB)

        // clean-up message which is received
        if(env->rx_buffer != NULL)
        {
            kernel_free(env->rx_buffer);
        }

        // free environment variable
        while(!common_list_is_empty(&(env->tx_queue)))
        {
            struct kernel_msg *msg = (struct kernel_msg*) common_list_pop_front(&(env->tx_queue));

            if(!reset)
            {
                if(msg->id == L2CC_PDU_SEND_CMD)
                {
                    struct l2cc_pdu_send_cmd *pdu = (struct l2cc_pdu_send_cmd *) kernel_msg2param(msg);
                    l2cc_send_error_evt(conidx, L2CC_PDU_SEND, msg->src_id, GAP_ERR_DISCONNECTED, pdu->pdu.chan_id);
                }
                #if (BLE_DEBUG)
                else if(msg->id == L2CC_DBG_PDU_SEND_CMD)
                {
                    struct l2cc_dbg_pdu_send_cmd *dbg_pdu = (struct l2cc_dbg_pdu_send_cmd *) kernel_msg2param(msg);
                    l2cc_send_error_evt(conidx, L2CC_DBG_PDU_SEND, msg->src_id, GAP_ERR_DISCONNECTED, common_read16p(dbg_pdu->pdu.data));
                }
                #endif // (BLE_DEBUG)
                else
                {
                    // not expected
                    ASSERT_WARN(0, conidx, msg->id);
                }
            }

            kernel_free(msg);
        }

        kernel_free(l2cc_env[conidx]);
        l2cc_env[conidx] = NULL;
    }

    // Set Free state
    kernel_state_set(KERNEL_BUILD_ID(TASK_L2CC, conidx), L2CC_FREE);
}

void l2cc_update_state(uint8_t conidx, kernel_state_t state, bool busy)
{
    // change state only if there is no ongoing disconnect.
    kernel_state_t old_state  = kernel_state_get(KERNEL_BUILD_ID(TASK_L2CC, conidx));
    if(busy)
    {
        // set state to busy
        kernel_state_set(KERNEL_BUILD_ID(TASK_L2CC, conidx), old_state | state);
    }
    else
    {
        // set state to idle
        kernel_state_set(KERNEL_BUILD_ID(TASK_L2CC, conidx), old_state & ~(state));
    }
}

void l2cc_send_complete_evt(uint8_t conidx, uint8_t op_type, uint8_t status)
{
    struct l2cc_env_tag * env = l2cc_env[conidx];
    if((env != NULL) && (op_type < L2CC_OP_MAX))
    {
        void* operation    = env->operation[op_type];

        if(operation != NULL)
        {
            struct kernel_msg* cmd = kernel_param2msg(operation);

            // send command completed event
            struct l2cc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(L2CC_CMP_EVT, cmd->src_id,
                    KERNEL_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

            /* fill up the parameters */
            cmp_evt->operation = *((uint8_t*) operation);
            cmp_evt->status    = status;
            cmp_evt->credit    = 0;
            cmp_evt->cid       = L2C_CID_LE_SIGNALING;

            /* send the complete event */
            kernel_msg_send(cmp_evt);

            // clean-up operation parameters
            kernel_msg_free(cmd);

            env->operation[op_type] = NULL;
        }

        // set state to ready
        l2cc_update_state(conidx, (1 << op_type), false);
    }
}


void l2cc_send_error_evt(uint8_t conidx, uint8_t operation, const kernel_task_id_t requester, uint8_t status, uint16_t cid)
{
    // prepare command completed event with error status
    struct l2cc_cmp_evt* cmp_evt = KERNEL_MSG_ALLOC(L2CC_CMP_EVT,
            requester, KERNEL_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

    cmp_evt->operation = operation;
    cmp_evt->status = status;
    cmp_evt->credit = 0;
    cmp_evt->cid    = cid;

    // send event
    kernel_msg_send(cmp_evt);
}

uint8_t l2cc_get_operation(uint8_t conidx, uint8_t op_type)
{
    struct l2cc_env_tag * env = l2cc_env[conidx];
    // by default no operation
    uint8_t ret = L2CC_NO_OP;

    ASSERT_ERR(conidx < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);

    // check if an operation is registered
    if(env->operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t*) env->operation[op_type]));
    }

    return ret;
}

void* l2cc_get_operation_ptr(uint8_t conidx, uint8_t op_type)
{
    struct l2cc_env_tag * env = l2cc_env[conidx];
    ASSERT_ERR(conidx  < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);
    // return operation pointer
    return env->operation[op_type];
}

void l2cc_set_operation_ptr(uint8_t conidx, uint8_t op_type, void* op)
{
    struct l2cc_env_tag * env = l2cc_env[conidx];
    ASSERT_ERR(conidx  < L2CC_IDX_MAX);
    ASSERT_ERR(op_type < L2CC_OP_MAX);
    // set operation pointer
    env->operation[op_type] = op;
}


void l2cc_data_send(uint8_t conidx, uint8_t nb_buffer)
{
    struct l2cc_env_tag *env = l2cc_env[conidx];

    if(env != NULL)
    {
        // L2CC Environment
        struct kernel_msg *pkt = (struct kernel_msg *) common_list_pick(&(env->tx_queue));

        bool process_pdu = (pkt != NULL) && (nb_buffer > 0);

        while (process_pdu)
        {
            // Status of the request
            uint8_t  status = GAP_ERR_NO_ERROR;
            // Length of the packet
            uint16_t tx_length = 0;
            // Allocate TX Buffer
            uint8_t* buffer;
            uint8_t  llid;


            // ************ Allocate a TX Buffer
            #if BLE_EMB_PRESENT
            struct em_buf_node* buf_node = em_buf_tx_alloc();
            // This should not happen
            ASSERT_ERR(buf_node != NULL);
            // Retrieve buffer
            buffer = (uint8_t *)(EM_BASE_ADDR + buf_node->buf_ptr);
            #else // (BLE_HOST_PRESENT)
            // allocate buffer in message heap
            buffer = kernel_malloc(l2cm_get_buffer_size() + HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN, KERNEL_MEM_KERNEL_MSG);
            // move pointer to payload beginning
            buffer += HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN;
            #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)

            // ************* Pack PDU into allocated buffer
            switch(pkt->id)
            {
                case L2CC_PDU_SEND_CMD:
                {
                    struct l2cc_pdu_send_cmd* pdu_cmd = (struct l2cc_pdu_send_cmd*) kernel_msg2param(pkt);
                    struct l2cc_pdu* pdu= (struct l2cc_pdu*) &(pdu_cmd->pdu);

                    // calculate LLID - an error here
                    llid    = ((pdu_cmd->offset == 0) ? L2C_PB_START_NON_FLUSH : L2C_PB_CONTINUE);

                    // Pack the PDU
                    status = l2cc_pdu_pack(pdu, &(pdu_cmd->offset), &tx_length, buffer, &llid);

                    // Check if another packet need to be sent or if an error has occurred
                    if ((status != GAP_ERR_NO_ERROR) || (pdu->payld_len == 0))
                    {
                        // send command completed event
                        struct l2cc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(L2CC_CMP_EVT, pkt->src_id,
                                KERNEL_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

                        /* fill up the parameters */
                        cmp_evt->operation = L2CC_PDU_SEND;
                        cmp_evt->status    = status;
                        cmp_evt->cid       = pdu->chan_id;
                        cmp_evt->credit    = 0;

                        /* send the complete event */
                        kernel_msg_send(cmp_evt);

                        common_list_pop_front(&env->tx_queue);

                        kernel_msg_free(pkt);
                        process_pdu = false;
                    }
                } break;

                #if (BLE_LECB)
                case L2CC_LECB_SDU_SEND_CMD:
                {
                    struct l2cc_lecb_sdu_send_cmd* sdu_cmd = (struct l2cc_lecb_sdu_send_cmd*) kernel_msg2param(pkt);
                    struct l2cc_sdu* sdu= (struct l2cc_sdu*) &(sdu_cmd->sdu);

                    struct l2cc_lecb_info* lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, sdu->cid);

                    // size of segment that have to be transmitted
                    uint16_t rem_seg_len = 0;
                    // size of segment already sent
                    uint16_t tx_seg_len = 0;

                    if((lecb == NULL) || (!GETB(lecb->state, L2CC_LECB_CONNECTED)))
                    {
                        status = L2C_ERR_INVALID_CID;
                    }
                    else
                    {
                        // New SDU to send
                        if(sdu_cmd->offset == 0)
                        {
                            rem_seg_len = common_min(sdu->length + L2C_SDU_LEN, lecb->peer_mps);
                        }
                        else
                        {
                            tx_seg_len = ((sdu_cmd->offset + L2C_SDU_LEN) % lecb->peer_mps);

                            // Recalculate length to be copied
                            rem_seg_len = common_min(sdu->length - sdu_cmd->offset, (lecb->peer_mps - tx_seg_len));
                        }

                        // New Segment to send
                        if(tx_seg_len == 0)
                        {
                            if(lecb->peer_credit == 0)
                            {
                                // mark that buffer is waiting for new credits
                                SETB(lecb->state, L2CC_LECB_TX_WAIT, true);

                                // remove message from tx queue
                                common_list_pop_front(&env->tx_queue);
                                process_pdu = false;

                                break; // nothing to send
                            }
                            else
                            {
                                // Decrease credit
                                lecb->peer_credit--;
                                sdu->credit++;
                            }
                        }
                    }

                    if(status == GAP_ERR_NO_ERROR)
                    {
                        // Pack the PDU
                        llid   = ((tx_seg_len == 0) ? L2C_PB_START_NON_FLUSH : L2C_PB_CONTINUE);
                        status = l2cc_lecb_pdu_pack(conidx, sdu, &tx_length, buffer, &(sdu_cmd->offset), rem_seg_len, llid);
                    }


                    // Check if another packet need to be sent or if an error has occurred
                    if ((status != GAP_ERR_NO_ERROR) || (sdu_cmd->offset == sdu->length))
                    {
                        // send command completed event
                        struct l2cc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(L2CC_CMP_EVT, pkt->src_id,
                                KERNEL_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

                        /* fill up the parameters */
                        cmp_evt->operation = L2CC_LECB_SDU_SEND;
                        cmp_evt->status    = status;
                        cmp_evt->cid       = sdu->cid;
                        cmp_evt->credit    = sdu->credit;

                        /* send the complete event */
                        kernel_msg_send(cmp_evt);

                        common_list_pop_front(&env->tx_queue);

                        kernel_msg_free(pkt);

                        if(lecb != NULL)
                        {
                            lecb->tx_sdu = NULL;
                        }
                        process_pdu = false;
                    }
                } break;
                #endif // (BLE_LECB)

                #if (BLE_DEBUG)
                case L2CC_DBG_PDU_SEND_CMD:
                {
                    struct l2cc_dbg_pdu_send_cmd* dbg_pdu_cmd= (struct l2cc_dbg_pdu_send_cmd*) kernel_msg2param(pkt);
                    struct l2cc_dbg_pdu* dbg_pdu = (struct l2cc_dbg_pdu*) &(dbg_pdu_cmd->pdu);

                    // Pack the PDU
                    status = l2cc_dbg_pdu_pack(dbg_pdu, &tx_length, buffer, &(dbg_pdu_cmd->offset), &llid);

                    // Check if another packet need to be sent or if an error has occurred
                    if ((status != GAP_ERR_NO_ERROR) || (dbg_pdu->length == 0))
                    {
                        // send command completed event
                        struct l2cc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(L2CC_CMP_EVT, pkt->src_id,
                                KERNEL_BUILD_ID(TASK_L2CC, conidx), l2cc_cmp_evt);

                        /* fill up the parameters */
                        cmp_evt->operation = L2CC_DBG_PDU_SEND;
                        cmp_evt->status    = status;
                        cmp_evt->cid       = common_read16p(dbg_pdu->data);
                        cmp_evt->credit    = 0;

                        /* send the complete event */
                        kernel_msg_send(cmp_evt);

                        common_list_pop_front(&env->tx_queue);

                        kernel_msg_free(pkt);
                        process_pdu = false;
                    }
                } break;
                #endif // (BLE_DEBUG)

                default:
                {
                    ASSERT_INFO(0, pkt->id, conidx);
                } break;
            }


            if (status == GAP_ERR_NO_ERROR)
            {
                // Allocate the message for L2CC task
                struct hci_acl_data_tx *data_tx = KERNEL_MSG_ALLOC(HCI_ACL_DATA_TX, conidx, KERNEL_BUILD_ID(TASK_L2CC, conidx),
                                                               hci_acl_data_tx);
                // Connection handle
                data_tx->conhdl      = gapc_get_conhdl(conidx);
                #if (BLE_EMB_PRESENT)
                // Pass the buffer node
                data_tx->buf         = buf_node;
                #else// (BLE_HOST_PRESENT)
                data_tx->buffer      = buffer;
                #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)
                // Pass the packet length
                data_tx->length      = tx_length;
                // Flag set to l2cap start or continue
                data_tx->pb_bc_flag  = llid;
                // Send message
                hci_send_2_controller(data_tx);

                // Decrement the number of available buffers
                l2cm_buffer_acquire();
                nb_buffer--;
            }
            else
            {
                // Just ignore to send the packet
                #if (BLE_EMB_PRESENT)
                em_buf_tx_buff_free(buf_node->idx);
                #else// (BLE_HOST_PRESENT)
                kernel_free(buffer - (HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN));
                #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)
            }

            // Check if there are other available buffers
            if (nb_buffer == 0)
            {
                process_pdu = false;
            }
        }

        // set if tx buffer queue is empty or not
        l2cm_tx_status(conidx, ! common_list_is_empty(&(env->tx_queue)));
    }
}



#endif //(BLE_L2CC)

/// @} L2CC
