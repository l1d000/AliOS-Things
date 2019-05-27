/**
 ****************************************************************************************
 *
 * @file llc_task.c
 *
 * @brief LLC task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "common_bt.h"
#include "common_error.h"
#include "common_llcp.h"
#include "kernel_msg.h"
#include "kernel_timer.h"
#include "llm_task.h"
#include "llm_util.h"
#include "llc_llcp.h"
#include "llc_util.h"
#include "llc_task.h"
#include "llcontrl.h"
#include "lld_util.h"
#include "lld_pdu.h"
#include "lld.h"
#include "em_buf.h"

#include "reg_ble_em_rx_desc.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_ble_em_cs.h"

#include "dbg_swdiag.h"

#if (HCI_PRESENT)
#include "hci.h"
#endif //(HCI_PRESENT)

#if (BLE_CHNL_ASSESS)
#include "llc_ch_asses.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif //(NVDS_SUPPORT))
#endif //(BLE_CHNL_ASSESS)

#if(BLE_AUDIO)
#include "reg_blecore.h"
#include "audio.h"
#endif // (BLE_AUDIO)
#if (BLE_PERIPHERAL || BLE_CENTRAL)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */
// alen.xu
/**
 ****************************************************************************************
 * @brief Handles the update of the connection parameters.
 * The handler is in charge to change the Link Layer connection parameters of a connection..
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int hci_le_con_update_cmd_handler(kernel_msg_id_t const msgid,
                                         struct hci_le_con_update_cmd const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;
    uint8_t complete_status = COMMON_ERROR_UNDEFINED;

    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    struct lld_evt_tag *evt = NULL;
    struct llc_env_tag *llc_env_ptr = llc_env[param->conhdl];

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);
        status = COMMON_ERROR_NO_ERROR;
        #if (BLE_QUALIF)
        if(lld_get_mode(param->conhdl) == LLD_EVT_SLV_MODE)
        {
            complete_status = COMMON_ERROR_COMMAND_DISALLOWED;
        }
        else
        #endif //(BLE_QUALIF)
        #if (BLE_PERIPHERAL)
        // Slave role and remote device doesn't support this feature
        if((lld_get_mode(param->conhdl) == LLD_EVT_SLV_MODE) && GETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH)
                        && (!(llc_env_ptr->feats_used.feats[0] & BLE_CON_PARAM_REQ_PROC_FEATURE)))
        {
            complete_status = COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        else
        #endif //(BLE_PERIPHERAL)
        if (  (    (param->con_intv_min > param->con_intv_max) || (param->ce_len_min > param->ce_len_max)
               #if(BLE_AUDIO)
               || (param->con_intv_max > LLC_CNX_INTERVAL_MAX)|| (param->con_intv_min < AUDIO_MIN_INTERVAL)
               #else
               || (param->con_intv_max > LLC_CNX_INTERVAL_MAX)|| (param->con_intv_min < LLC_CNX_INTERVAL_MIN)
               #endif
               || (param->con_latency > LLC_CNX_LATENCY_MAX)
               // check supervision to range
               || (param->superv_to > LLC_CNX_SUP_TO_MAX)     || (param->superv_to < LLC_CNX_SUP_TO_MIN)
                // CSA/ESR6 : supervision timeout minimum value does not apply for connection request with a 4.0 device.
                // so supervision timeout must be <=  (1 + Conn_Latency) * Conn_Interval_Max *2
                // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
                // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
                // (sup_to * 10) <= ((1+latency)* con_interval*1.25*2)
                // to simplify computation and remove floating point we factor everything by 2/5
                // (hci_sup_to * 4) <= ((1+hci_latency)* hci_interval)
               || ((((uint32_t) param->superv_to) <<2) <= ((1 + ((uint32_t)param->con_latency)) * ((uint32_t)param->con_intv_max))))
            #if (BLE_TESTER)
            && !(llc_env_ptr->tester_params.tester_feats & LLC_TESTER_FORCE_UP_PARAM)
            #endif // (BLE_TESTER)
           )
        {
            status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
        // Check if an actual connection update procedure is needed
        // If the parameters are the same (or very similar, i.e. no need for update)
        // then just send the connection update complete event directly without initiating the whole procedure
        else if (   (   (evt->interval/2 >= param->con_intv_min) && (evt->interval/2 <= param->con_intv_max)
                     && (llc_env_ptr->elt->duration_min/(2*SLOT_SIZE) >= param->ce_len_min)
                     && (llc_env_ptr->elt->duration_min/(2*SLOT_SIZE) <= param->ce_len_max)
                     && (llc_env_ptr->sup_to == param->superv_to)
                     && ((evt->evt.conn.latency - 1) == param->con_latency))
                #if (BLE_TESTER)
                 && !(llc_env_ptr->tester_params.tester_feats & LLC_TESTER_FORCE_UP_PARAM)
                #endif // (BLE_TESTER)
                )
        {
            complete_status = COMMON_ERROR_NO_ERROR;
        }
        else
        {
            struct llc_con_upd_req_ind * con_update = KERNEL_MSG_ALLOC(LLC_CON_UPD_REQ_IND, dest_id, dest_id, llc_con_upd_req_ind);

            con_update->operation    = LLC_CON_UP_HCI_REQ;
            con_update->con_intv_min = param->con_intv_min;
            con_update->con_intv_max = param->con_intv_max;
            con_update->con_latency  = param->con_latency;
            con_update->superv_to    = param->superv_to;
            con_update->ce_len_min   = param->ce_len_min;
            con_update->ce_len_max   = param->ce_len_max;

            kernel_msg_send(con_update);
        }
    }

    if(msg_status != KERNEL_MSG_SAVED)
    {
        llc_common_cmd_status_send(src_id, status, param->conhdl);

        // Checks if the event is not filtered
        if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT) && (complete_status != COMMON_ERROR_UNDEFINED))
        {

            // Send the LE META event connection update to the host
            llc_con_update_complete_send(complete_status, param->conhdl, evt);
        }
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the creation of link indication from the logical link driver.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_chnl_map_cmd_handler(kernel_msg_id_t const msgid,
                                          struct hci_basic_conhdl_cmd const *param,
                                          kernel_task_id_t const dest_id,
                                          kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    // structure type for the complete command event
    struct hci_le_rd_chnl_map_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_LE_RD_CHNL_MAP_CMD_OPCODE,
                         hci_le_rd_chnl_map_cmd_cmp_evt);

    uint8_t state = kernel_state_get(dest_id);

    // Check if a connection is active
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        status = COMMON_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        // gets the channel map
        memcpy(&event->ch_map.map[0], &llc_env[param->conhdl]->ch_map.map[0], LE_CHNL_MAP_LEN);
    }

    // update the status
    event->status = status;

    //gets the connection handle
    event->conhdl = param->conhdl;

    // sends the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the creation of link indication from the logical link driver.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_rem_used_feats_cmd_handler(kernel_msg_id_t const msgid,
                                                struct hci_le_rd_rem_used_feats_cmd const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    // Command status
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    #if (BLE_QUALIF)
    else if(lld_get_mode(param->conhdl) == LLD_EVT_SLV_MODE)
    {
        // Not supported in version 4.0
    }
    #endif // (BLE_QUALIF)
    // Features have been exchanged
    else if (GETF(llc_env[param->conhdl]->llc_status, LLC_STAT_FEAT_EXCH))
    {
        // nothing to negotiate, known peer feature will be provided to host
        status = COMMON_ERROR_NO_ERROR;
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            // check if traffic is paused
            || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
            // check if we are waiting for TX ack
            //|| llc_state_chk(state, LLC_WAIT_TX_ACK_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        status = COMMON_ERROR_NO_ERROR;

        // wait for peer response
        llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
        llc_env[param->conhdl]->loc_proc_state = LLC_LOC_WAIT_FEAT_RSP;

        // Send the feature request PDU
        llc_llcp_feats_req_pdu_send(param->conhdl);

        // Start the response timeout
        kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        // Send the command status event
        llc_common_cmd_status_send(src_id, status, param->conhdl);

        if (GETF(llc_env[param->conhdl]->llc_status, LLC_STAT_FEAT_EXCH))
        {
            // Send the meta event with the values received from the peer
            llc_feats_rd_event_send(COMMON_ERROR_NO_ERROR, param->conhdl, &(llc_env[param->conhdl]->feats_used));
        }
    }

    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles the creation of link indication from the logical link driver.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_start_enc_cmd_handler(kernel_msg_id_t const msgid,
                                        struct hci_le_start_enc_cmd const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    // Get the connection handle
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;

    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        llc_common_cmd_status_send(src_id, COMMON_ERROR_COMMAND_DISALLOWED, param->conhdl);
        // Nothing to do
    }
    // check if traffic is paused
    else if(llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        llc_common_cmd_status_send(src_id, COMMON_ERROR_CONTROLLER_BUSY, param->conhdl);
        // Nothing to do, status already correct
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        // This command shall be issued by host only if device role is master, and if
        // encryption is set in the used features
        if ((lld_get_mode(conhdl) != LLD_EVT_MST_MODE) ||
            !(llc_env[param->conhdl]->feats_used.feats[0] & BLE_ENC_FEATURE))
        {
            llc_common_cmd_status_send(src_id, COMMON_ERROR_COMMAND_DISALLOWED, param->conhdl);
        }
        else
        {
            uint8_t tx_pkt_cnt;

            // Send the command status event
            llc_common_cmd_status_send(src_id, COMMON_ERROR_NO_ERROR, param->conhdl);


            // Still some data programmed, so set the state to LLC_ENC_PAUSING
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
            llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, true);

            // wait for traffic to be paused
            llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_TRAFFIC_PAUSED;

            GLOBAL_INT_DIS();
            // Encryption procedure is started in the master, so we are not supposed
            // to transmit any unexpected PDUs until procedure is complete
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_TX_FLOW_OFF, true);

            tx_pkt_cnt = lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt);
            GLOBAL_INT_RES();

            // Check if we still have data programmed for transmission
            if (tx_pkt_cnt)
            {
                SETF(llc_env[conhdl]->llc_status, LLC_STAT_WAIT_TRAFFIC_PAUSED, true);
            }
            else
            {
                // device ready to continue starting encryption
                kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
            }

            // Save the message
            llc_util_set_operation_ptr(param->conhdl, LLC_OP_ENCRYPT, (void*)param);

            msg_status = KERNEL_MSG_NO_FREE;
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the creation of link indication from the logical link driver.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_ltk_req_reply_cmd_handler(kernel_msg_id_t const msgid,
                                            struct hci_le_ltk_req_reply_cmd const *param,
                                            kernel_task_id_t const dest_id,
                                            kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;;

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if Remote procedure is on-going
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        if(llc_env[param->conhdl]->rem_proc_state == LLC_REM_WAIT_LTK)
        {
            // Allocate the LLM encryption command
            struct llm_enc_req* msg = KERNEL_MSG_ALLOC(LLM_ENC_REQ, TASK_LLM, dest_id, llm_enc_req);

            // Fill in the message parameters
            memcpy(&msg->key.ltk[0], &param->ltk.ltk[0], KEY_LEN);
            memcpy(&msg->plain_data[0], &llc_env[param->conhdl]->encrypt.skd.skd[0], KEY_LEN);

            // Send the command
            kernel_msg_send(msg);

            // Set the state of the LLC
            llc_env[param->conhdl]->rem_proc_state = LLC_REM_WAIT_SK;

            // Status is successful
            status = COMMON_ERROR_NO_ERROR;
        }
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        // Send the command complete event
        llc_common_cmd_complete_send(src_id, status, param->conhdl);
    }

    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles the creation of link indication from the logical link driver.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_ltk_req_neg_reply_cmd_handler(kernel_msg_id_t const msgid,
                                                struct hci_basic_conhdl_cmd const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;;

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if Remote procedure is on-going
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        if(llc_env[param->conhdl]->rem_proc_state == LLC_REM_WAIT_LTK)
        {
            // If the Host does not provide a Long Term Key, because it indicates that
            // a key is not available the slave shall:
            //   1. If the Pause Encryption Procedure was executed before restarting
            //      the encryption, initiate the Termination Procedure
            //   2. Otherwise, send an LL_REJECT_IND PDU to abort the Encryption Start
            //      procedure
            // In both cases, the reason shall be set to PIN or key Missing.
            if (LLC_UTIL_ENC_STATE_IS_SET(param->conhdl, LLC_ENC_REFRESH_PENDING))
            {
                // Key refresh procedure is pending, so initiate the Termination Procedure
                llc_llcp_terminate_ind_pdu_send(param->conhdl, COMMON_ERROR_PIN_MISSING);
            }
            else
            {
                // restart LLCP TO timer if on-going transaction is pending
                if (llc_state_chk(state, LLC_LOC_PROC_BUSY))
                {
                    // Restartt LLCP TO timer
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                }

                // Go back to remote procedure idle state
                llc_env[param->conhdl]->rem_proc_state = LLC_REM_WAIT_ENC_REJECT_ACK;

                GLOBAL_INT_DIS();
                // RX flow can now be restarted
                LLC_UTIL_ENC_STATE_UP(param->conhdl, LLC_ENC_RX_FLOW_OFF, false);
                GLOBAL_INT_RES();


                // Initial encryption start procedure is pending, so simply reject
                llc_llcp_reject_ind_pdu_send(param->conhdl, LLCP_ENC_REQ_OPCODE, COMMON_ERROR_PIN_MISSING);

            }

            // Status is successful
            status = COMMON_ERROR_NO_ERROR;
        }
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        // Send the command complete event
        llc_common_cmd_complete_send(src_id, status, param->conhdl);
    }

    return (msg_status);
}
#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the HCI LE Remote Connection Parameter Request Reply Command.
 * The handler is in charge of handling the HCI LE Remote Connection Parameter Request Reply Command
 * and of sending the command complete event to the host.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rem_con_param_req_reply_cmd_handler(kernel_msg_id_t const msgid,
                                                struct hci_le_rem_con_param_req_reply_cmd const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if Remote procedure is on-going
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        struct llc_env_tag *llc_env_ptr = llc_env[param->conhdl];

        if(llc_env_ptr->rem_proc_state == LLC_REM_WAIT_CON_PARAM_HOST_RSP)
        {
            struct llc_con_upd_req_ind *param_req = (struct llc_con_upd_req_ind *)llc_util_get_operation_ptr(param->conhdl, LLC_OP_REM_PARAM_UPD);

            status = COMMON_ERROR_NO_ERROR;

            if (lld_get_mode(param->conhdl) == LLD_EVT_MST_MODE)
            {
                param_req->operation    = LLC_CON_UP_FORCE;
                param_req->interval_min = common_min(param_req->interval_min, param->interval_min);
                param_req->interval_max = common_max(param_req->interval_max, param->interval_max);
                param_req->con_latency  = param->latency;
                param_req->superv_to    = common_max(param_req->superv_to,param->timeout);
                kernel_msg_send(param_req);

                llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, false);
                llc_env_ptr->rem_proc_state = LLC_REM_IDLE;

                // Clear Operation
                llc_util_set_operation_ptr(param->conhdl, LLC_OP_REM_PARAM_UPD, NULL);
            }
            else if (lld_get_mode(param->conhdl) == LLD_EVT_SLV_MODE)
            {
                param_req->interval_min = param->interval_min;
                param_req->interval_max = param->interval_max;
                param_req->con_latency  = param->latency;
                param_req->superv_to    = param->timeout;

                lld_con_param_rsp(param->conhdl, llc_env_ptr->elt, param_req);

                // Wait for peer connection parameter response
                llc_env_ptr->rem_proc_state = LLC_REM_WAIT_CON_UPD_REQ;

                llc_llcp_con_param_rsp_pdu_send(param->conhdl, param_req);
                // Start the LLCP Response TO
                kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                // Clear Operation
                llc_util_clear_operation_ptr(param->conhdl, LLC_OP_REM_PARAM_UPD);
            }
        }
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        // Send the command status event
        llc_common_cmd_complete_send(src_id, status, param->conhdl);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the HCI LE Remote Connection Parameter Request Negative Reply Command.
 * The handler is in charge of handling the HCI LE Remote Connection Parameter Request Negative Reply Command
 * and of sending the command complete event to the host.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rem_con_param_req_neg_reply_cmd_handler(kernel_msg_id_t const msgid,
                                                struct hci_le_rem_con_param_req_neg_reply_cmd const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;;

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if Remote procedure is on-going
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        if(llc_env[param->conhdl]->rem_proc_state == LLC_REM_WAIT_CON_PARAM_HOST_RSP)
        {
            if(param->reason != COMMON_ERROR_UNACCEPTABLE_CONN_INT)
            {
                status = COMMON_ERROR_INVALID_HCI_PARAM;
            }
            else
            {
                // Clear Operation
                llc_util_clear_operation_ptr(param->conhdl, LLC_OP_REM_PARAM_UPD);

                // Go back to remote procedure idle state
                llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, false);
                llc_env[param->conhdl]->rem_proc_state = LLC_REM_IDLE;

                status = COMMON_ERROR_NO_ERROR;

                // Send the reject extended indication PDU
                llc_llcp_reject_ind_pdu_send(param->conhdl, LLCP_CONNECTION_PARAM_REQ_OPCODE, param->reason);
            }
        }
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        // Send the command status event
        llc_common_cmd_complete_send(src_id, status, param->conhdl);
    }

    return (msg_status);
}
#endif //#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the flush request from the host
 * Discards all data that is currently pending for transmission in the controller for
 * the specified connection handle.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_flush_cmd_handler(kernel_msg_id_t const msgid,
                                 struct hci_basic_conhdl_cmd const *param,
                                 kernel_task_id_t const dest_id,
                                 kernel_task_id_t const src_id)
{
    uint16_t conhdl = param->conhdl;

    // allocates the message to send
    struct hci_flush_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_FLUSH_CMD_OPCODE, hci_flush_cmd_cmp_evt);

    // sends the flush occurred event
    llc_common_flush_occurred_send(conhdl);
    // free the allocated buffer to this conhdl
    lld_pdu_tx_flush(LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt));

    // sends the command complete event
    event->conhdl= conhdl;
    event->status= COMMON_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the disconnection request from the host
 * Closes the connection for the dedicated connection handle.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_disconnect_cmd_handler(kernel_msg_id_t const msgid,
                                      struct hci_disconnect_cmd const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;;

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        // check reason range
        if (llc_util_disc_reason_ok(param->reason))
        {
            // Termination procedure can be started at any time
            status = COMMON_ERROR_NO_ERROR;
            // Initiate the termination procedure
            llc_llcp_terminate_ind_pdu_send(param->conhdl, param->reason);
        }
        else
        {
            // reason is not acceptable
            status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
    }
    if(!llc_env[param->conhdl]->disc_event_sent)
    {
        llc_common_cmd_status_send(src_id, status, param->conhdl);
        llc_env[param->conhdl]->disc_event_sent = true;
    }
    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        llc_env[param->conhdl]->disc_event_sent = false;
    }

    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles the rssi information request from the host
 * Sends the rssi for the dedicated link to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_rssi_cmd_handler(kernel_msg_id_t const msgid,
                                   struct hci_basic_conhdl_cmd const *param,
                                   kernel_task_id_t const dest_id,
                                   kernel_task_id_t const src_id)
{
    // allocate the status event message
    struct hci_rd_rssi_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_RSSI_CMD_OPCODE, hci_rd_rssi_cmd_cmp_evt);
    uint8_t state = kernel_state_get(dest_id);
    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        event->status = COMMON_ERROR_COMMAND_DISALLOWED;
        // gets the rssi
        event->rssi = 0;
    }
    else
    {
        event->status = COMMON_ERROR_NO_ERROR;
        // gets the rssi
        event->rssi = llc_env[param->conhdl]->rssi;
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the rssi information request from the host
 * Sends the rssi for the dedicated link to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_tx_pwr_lvl_cmd_handler(kernel_msg_id_t const msgid,
                                         struct hci_rd_tx_pwr_lvl_cmd const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    // allocate the status event message
    struct hci_rd_tx_pwr_lvl_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_TX_PWR_LVL_CMD_OPCODE, hci_rd_tx_pwr_lvl_cmd_cmp_evt);


    uint8_t state = kernel_state_get(dest_id);
    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        event->status = COMMON_ERROR_COMMAND_DISALLOWED;
        event->tx_pow_lvl = 0;
    }
    else
    {
        event->status = COMMON_ERROR_NO_ERROR;
        switch(param->type)
        {
            case TX_LVL_CURRENT:
            {
                // gets the current level
                event->tx_pow_lvl = rwip_rf.txpwr_dbm_get(ble_txpwr_getf(param->conhdl), MOD_GFSK);
                // sets status
            } break;
            case TX_LVL_MAX:
            {
                // gets the level max
                event->tx_pow_lvl = rwip_rf.txpwr_dbm_get(rwip_rf.txpwr_max, MOD_GFSK);
            } break;
            default:
            {   // sets status
                event->status = COMMON_ERROR_INVALID_HCI_PARAM;
                event->tx_pow_lvl = 0;
            } break;
        }
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the read remote information version request from the host
 * Sends the version number company id and subversion number for the dedicated link to
 * the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_rem_ver_info_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_rd_rem_ver_info_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        llc_common_cmd_status_send(src_id, COMMON_ERROR_COMMAND_DISALLOWED, param->conhdl);
        // Nothing to do
    }
    // If peer version is already known, we can finish the procedure immediately
    else if(GETF(llc_env[conhdl]->llc_status, LLC_STAT_PEER_VERS_KNOWN))
    {
        // Send the command status event
        llc_common_cmd_status_send(src_id, COMMON_ERROR_NO_ERROR, param->conhdl);

        // Immediately followed by the Version Indication event
        llc_version_rd_event_send(COMMON_ERROR_NO_ERROR, conhdl);
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            // check if traffic is paused
            || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
        llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_VERS_IND;

        // Send the version indication to the peer
        llc_llcp_version_ind_pdu_send(param->conhdl);

        // Start the response timeout
        kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
        // Send the command status event to the host
        llc_common_cmd_status_send(src_id, COMMON_ERROR_NO_ERROR, param->conhdl);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the read authenticated payload timeout command from the host
 * Sends the status to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_auth_payl_to_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_rd_auth_payl_to_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    // allocate the status event message
    struct hci_rd_auth_payl_to_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, hci_rd_auth_payl_to_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        event->status = COMMON_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        event->status = COMMON_ERROR_NO_ERROR;
        // The authenticated payload timeout is expressed in units of 10 ms
        event->auth_payl_to = llc_env[conhdl]->auth_payl_to;
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the write authenticated payload timeout command from the host
 * Sends the status to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_wr_auth_payl_to_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_wr_auth_payl_to_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);

    // allocate the status event message
    struct hci_wr_auth_payl_to_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, hci_wr_auth_payl_to_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        event->status = COMMON_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt);

        // The authenticated payload timeout is expressed in units of 10 ms
        // The connection interval is expressed in units of 625 us
        // Therefore multiply the timeout by 16 to get units of 625 us
        // llc_env[conhdl]->evt->latency is lm latency plus 1
        if ((uint32_t)(param->auth_payl_to*16) >= (uint32_t)(evt->interval * evt->evt.conn.latency))
        {
            event->status = COMMON_ERROR_NO_ERROR;
            llc_env[conhdl]->auth_payl_to = param->auth_payl_to;
            llc_util_set_auth_payl_to_margin(LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt));
            #if !(BLE_QUALIF)
            // If the timers are already running, reset them as per spec
            if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_ENABLE))
            {
                kernel_timer_set(LLC_AUTH_PAYL_NEARLY_TO, dest_id, llc_env[conhdl]->auth_payl_to - llc_env[conhdl]->auth_payl_to_margin);
                kernel_timer_set(LLC_AUTH_PAYL_REAL_TO, dest_id, llc_env[conhdl]->auth_payl_to);
            }
            #endif //!(BLE_QUALIF)
        }
        else
        {
            event->status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_TESTER)
/**
 ****************************************************************************************
 * @brief Handles the tester set LE parameters command from the host
 * Sends the status to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_tester_set_le_params_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_tester_set_le_params_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        status = COMMON_ERROR_NO_ERROR;

        // Save the parameters in the environment
        llc_env[conhdl]->tester_params = *param;
    }

    llc_common_cmd_status_send(src_id, status, conhdl);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the tester enable BLE LLCP pass through mechanism
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_dbg_ble_tst_llcp_pt_en_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_dbg_ble_tst_llcp_pt_en_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        status = COMMON_ERROR_NO_ERROR;

        // Save the parameters in the environment
        llc_env[conhdl]->llcp_pass_through_enable = param->enable;
    }

    llc_common_cmd_status_send(src_id, status, conhdl);

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the tester to send a LLCP using pass through mechanism
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_dbg_ble_tst_send_llcp_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_dbg_ble_tst_send_llcp_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if pass through mechanism is enabled or not
    else if(!llc_env[conhdl]->llcp_pass_through_enable)
    {
        // Nothing to do
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            // check if traffic is paused
            || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        llc_llcp_tester_send(conhdl, param->length, (uint8_t*) param->data);
        status = COMMON_ERROR_NO_ERROR;
    }

    if(msg_status == KERNEL_MSG_CONSUMED)
    {
        llc_common_cmd_status_send(src_id, status, param->conhdl);
    }

    return (msg_status);
}

#endif // (BLE_TESTER)



/**
 ****************************************************************************************
 * @brief Handles the command HCI LE Set Data Length Command.
 * The LE_Set_Data_Length command allows the Host to suggest maximum transmission packet
 * size and maximum packet transmission time to be used for a given connection.
 * The Controller may use smaller or larger values based on local information.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if !(BLE_QUALIF)
/**
 ***************************************************************************************
 * BLE 4.2 handlers
 ***************************************************************************************
*/
static int hci_le_set_data_len_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_data_len_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        //Get environment pointer
        struct llc_env_tag *llc_env_ptr = llc_env[param->conhdl];

        //If the length request can be sent to the peer
        if(llc_env_ptr->data_len_ext_info.send_req_not_allowed == true)
        {
            status = COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        else if( (param->tx_octets < BLE_MIN_OCTETS ) || (param->tx_octets > BLE_MAX_OCTETS )
                || (param->tx_time < BLE_MIN_TIME ) || (param->tx_time > BLE_MAX_TIME ) || (param->conhdl > BLE_MAX_CONHDL) )
        {
            status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            // If the values are different from the current ones
            if((llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets != param->tx_octets)
                    ||(llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time != param->tx_time))
            {
                /**
                 * DLE
                 * save the operation to use the parameters when needed
                 */
                llc_util_set_operation_ptr(param->conhdl, LLC_OP_DLE_UPD, (void*)param);

                /**
                 * DLE
                 * Raise an IND to the LLD to send the LLCP LL_LENGTH_REQ with the local parameters
                 */
                kernel_msg_send_basic(LLC_LENGTH_REQ_IND, dest_id, dest_id);
                //Do not free the message until it is used
                msg_status = KERNEL_MSG_NO_FREE;
            }
            status = COMMON_ERROR_NO_ERROR;
        }
    }

    llc_common_cmd_complete_send(src_id, status, param->conhdl);

    return (msg_status);
}

/**
 ***************************************************************************************
 * BLE 5.0 handlers
 ***************************************************************************************
*/
#if (BLE_2MBPS)
/**
 ****************************************************************************************
 * @brief Handles the command HCI set default PHY comamnd.
 * The handler processes the command to read the value of the PHY for a dedicated link
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_phy_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_rd_phy_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    // allocate the status event message
    struct hci_le_rd_phy_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_LE_RD_PHY_CMD_OPCODE, hci_le_rd_phy_cmd_cmp_evt);
    // Get current state
    uint8_t state = kernel_state_get(dest_id);
    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        event->conhdl = param->conhdl;
        event->status = COMMON_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        // Gets the rx and tx phys used
        lld_util_get_phys(llc_env[param->conhdl]->elt, &event->tx_phy, &event->rx_phy);
        // gets connection handle
        event->conhdl = param->conhdl;
        event->status = COMMON_ERROR_NO_ERROR;
    }

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI set default PHY comamnd.
 * The handler processes the command to do an update of the PHY for a dedicated link
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_phy_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_phy_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[param->conhdl];

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        // Check if the feature exchange has been done and the feature is available or phy procedure disabled
        if((GETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH) && (!(llc_env_ptr->feats_used.feats[0] & BLE_2MBPS_FEATURE)))
                || !(GETF(llc_env_ptr->llc_status, LLC_STAT_PHY_ENABLED)))
        {
            status = COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        else
        {
            struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, dest_id, dest_id, llc_phy_upd_req_ind);

            phy_update->operation   = LLC_PHY_UPD_HCI_REQ;
            phy_update->all_phys    = param->all_phys;
            phy_update->rx_phys     = param->rx_phys;
            phy_update->tx_phys     = param->tx_phys;

            kernel_msg_send(phy_update);

            status = COMMON_ERROR_NO_ERROR;
        }
    }

    llc_common_cmd_status_send(src_id, status, param->conhdl);
    return (msg_status);
}
#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)
/*
 * HCI callback table
 ****************************************************************************************
 */
/// The message handlers for HCI command complete events
static const struct kernel_msg_handler llc_hci_command_handler_tab[] =
{
    {HCI_DISCONNECT_CMD_OPCODE                      , (kernel_msg_func_t)hci_disconnect_cmd_handler},
    {HCI_RD_REM_VER_INFO_CMD_OPCODE                 , (kernel_msg_func_t)hci_rd_rem_ver_info_cmd_handler},
    {HCI_FLUSH_CMD_OPCODE                           , (kernel_msg_func_t)hci_flush_cmd_handler},
    {HCI_RD_TX_PWR_LVL_CMD_OPCODE                   , (kernel_msg_func_t)hci_rd_tx_pwr_lvl_cmd_handler},
    {HCI_RD_RSSI_CMD_OPCODE                         , (kernel_msg_func_t)hci_rd_rssi_cmd_handler},
    {HCI_RD_AUTH_PAYL_TO_CMD_OPCODE                 , (kernel_msg_func_t)hci_rd_auth_payl_to_cmd_handler},
    {HCI_WR_AUTH_PAYL_TO_CMD_OPCODE                 , (kernel_msg_func_t)hci_wr_auth_payl_to_cmd_handler},
    {HCI_LE_CON_UPDATE_CMD_OPCODE                   , (kernel_msg_func_t)hci_le_con_update_cmd_handler},
    {HCI_LE_RD_CHNL_MAP_CMD_OPCODE                  , (kernel_msg_func_t)hci_le_rd_chnl_map_cmd_handler},
    {HCI_LE_RD_REM_USED_FEATS_CMD_OPCODE            , (kernel_msg_func_t)hci_le_rd_rem_used_feats_cmd_handler},
    {HCI_LE_START_ENC_CMD_OPCODE                    , (kernel_msg_func_t)hci_le_start_enc_cmd_handler},
    {HCI_LE_LTK_REQ_REPLY_CMD_OPCODE                , (kernel_msg_func_t)hci_le_ltk_req_reply_cmd_handler},
    {HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE            , (kernel_msg_func_t)hci_le_ltk_req_neg_reply_cmd_handler},
    #if !(BLE_QUALIF)
    {HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE      , (kernel_msg_func_t)hci_le_rem_con_param_req_reply_cmd_handler},
    {HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE  , (kernel_msg_func_t)hci_le_rem_con_param_req_neg_reply_cmd_handler},
    {HCI_LE_SET_DATA_LEN_CMD_OPCODE                 , (kernel_msg_func_t)hci_le_set_data_len_cmd_handler},
    #if(BLE_2MBPS)
    {HCI_LE_RD_PHY_CMD_OPCODE                       , (kernel_msg_func_t)hci_le_rd_phy_cmd_handler},
    {HCI_LE_SET_PHY_CMD_OPCODE                      , (kernel_msg_func_t)hci_le_set_phy_cmd_handler},
    #endif // (BLE_2MBPS)
    #endif // !(BLE_QUALIF)
    #if (BLE_TESTER)
    {HCI_TESTER_SET_LE_PARAMS_CMD_OPCODE            , (kernel_msg_func_t)hci_tester_set_le_params_cmd_handler},
    {HCI_DBG_BLE_TST_LLCP_PT_EN_CMD_OPCODE          , (kernel_msg_func_t)hci_dbg_ble_tst_llcp_pt_en_cmd_handler},
    {HCI_DBG_BLE_TST_SEND_LLCP_CMD_OPCODE           , (kernel_msg_func_t)hci_dbg_ble_tst_send_llcp_cmd_handler},
    #endif
};


/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles any HCI command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int llc_hci_command_handler(kernel_msg_id_t const msgid,
                        void const *param,
                        kernel_task_id_t const dest_id,
                        kernel_task_id_t const src_id)
{
    int return_status = KERNEL_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    int i = 0;
    for( i = 0; i < (sizeof(llc_hci_command_handler_tab)/sizeof(llc_hci_command_handler_tab[0])); i++)
    {
        // Check if opcode matches
        if(llc_hci_command_handler_tab[i].id == src_id)
        {
            // Check if there is a handler function
            if(llc_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = llc_hci_command_handler_tab[i].func(src_id, param, dest_id, src_id);
            }
            break;
        }
    }


    return (return_status);
}



/**
 ****************************************************************************************
 * @brief Handles the packet data to be sent.
 * The handler is in charge to change the state if necessary and push the data in the
 * logical link controller.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int llc_hci_acl_data_tx_handler(kernel_msg_id_t const msgid,
                            struct hci_acl_data_tx const *param,
                            kernel_task_id_t const dest_id,
                            kernel_task_id_t const src_id)
{
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY)
            || (param->length == 0))
    {
        // Free the allocated TX buffer
        GLOBAL_INT_DIS();
        em_buf_tx_buff_free(param->buf->idx);
        GLOBAL_INT_RES();

        // LLC is free or in the process of disconnection, so acknowledge immediately
        llc_common_nb_of_pkt_comp_evt_send(param->conhdl, 1);
    }
    else
    {

        uint8_t status = lld_pdu_data_send((void*)param);

        if(!status)
        {
            // Acknowledge immediately
            llc_common_nb_of_pkt_comp_evt_send(param->conhdl, 1);
        }
     
        #if(BLE_PERIPHERAL)
        // Schedule the next event as soon as possible
        lld_evt_schedule_next( llc_env[param->conhdl]->elt);
        #endif //(BLE_PERIPHERAL)
    }

    return (KERNEL_MSG_CONSUMED);
}

#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCTASK
