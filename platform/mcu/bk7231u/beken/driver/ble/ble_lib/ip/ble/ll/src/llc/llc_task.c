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

#include "llm_task.h"
#include "llm_util.h"
#include "llc_llcp.h"
#include "llc_util.h"
#include "llc_task.h"

#include "common_bt.h"
#include "common_error.h"
#include "common_llcp.h"
#include "kernel_msg.h"
#include "kernel_timer.h"
#include "llcontrl.h"
#include "lld_pdu.h"
#include "lld_util.h"
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
#endif
#if (BLE_PERIPHERAL || BLE_CENTRAL)


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

// handlers in llc_hci.c
extern int llc_hci_command_handler(kernel_msg_id_t const msgid, void const *param,
                               kernel_task_id_t const dest_id, kernel_task_id_t const src_id);
extern int llc_hci_acl_data_tx_handler(kernel_msg_id_t const msgid,  struct hci_acl_data_tx const *param,
                                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Request a random number generation to the HW aes block.
 * @param[in] state_proc state procedure to help the encryption mangement procedure.
 ****************************************************************************************
 */
static void llc_task_random_gen_request(kernel_task_id_t dest_id)
{
    struct llm_enc_req *msg = KERNEL_MSG_ALLOC(LLM_ENC_REQ, TASK_LLM, dest_id, llm_enc_req);
    uint32_t randn = 0;
    uint8_t i = 0 ;
    for ( i = 0 ; i < 4 ; i++)
    {
        randn = common_rand_word();
        memcpy(&msg->key.ltk[i*4], (uint8_t*)&randn, 4);
        randn = common_rand_word();
        memcpy(&msg->plain_data[i*4], (uint8_t*)&randn, 4);
    }

    // Send the command to start the RANDN generation
    kernel_msg_send(msg);

}

/**
 ****************************************************************************************
 * @brief Handles the encryption management in case where au pause/resume is requested.
 * The handler is in charge to change the state of the current logical link controller
 * task and to confirm the creation of the link to the logical link manager.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_enc_mgt_ind_handler(kernel_msg_id_t const msgid,
                                        void const *param,
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
        // Nothing to do
    }
    // master mode encryption handling
    else if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
    {
        // Sanity check
        ASSERT_INFO(lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt) == 0, dest_id, lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt));

        if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
        {
            switch(llc_env[conhdl]->loc_proc_state)
            {
                case LLC_LOC_WAIT_TRAFFIC_PAUSED:
                {
                    // If the connection is already encrypted when this command is issued, the
                    // key refresh procedure has to be started. It requires to first pause
                    // the encryption
                    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_ENABLE))
                    {
                        // Remind that we are proceeding to a key refresh
                        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_REFRESH_PENDING | LLC_ENC_PAUSE_PENDING, true);

                        // Send the LL_PAUSE_ENC_REQ pdu
                        llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_PAUSE_ENC_RSP;
                        llc_llcp_pause_enc_req_pdu_send(conhdl);
                        break;
                    }
                }
                // no break;
                case LLC_LOC_WAIT_PAUSE_ENC_RSP_SENT:
                {
                    // Generate the random number before sending the LLCP
                    llc_task_random_gen_request(dest_id);

                    llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_RANDN_GEN_IND;
                }
                break;
                case LLC_LOC_WAIT_RANDN_GEN_IND:
                {
                    // else send directly the encryption request
                    // Retrieve the message
                    struct hci_le_start_enc_cmd *enc_cmd = (struct hci_le_start_enc_cmd *)llc_util_get_operation_ptr(conhdl, LLC_OP_ENCRYPT);
                    // Sends the LLCP_ENC_REQ_OPCODE pdu
                    llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_ENC_RSP;
                    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_PAUSE_PENDING, false);

                    llc_llcp_enc_req_pdu_send(conhdl, enc_cmd);
                    // Clear Operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_ENCRYPT);
                } break;

                case LLC_LOC_WAIT_SK_AND_START_ENC_REQ:
                {
                    llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_START_ENC_REQ;
                } break;
                // ready to send Start Enc Rsp LLCP
                case LLC_LOC_WAIT_SK:
                case LLC_LOC_SEND_START_ENC_RSP:
                {
                    // Initialize the CCM counters
                    ble_txccmpktcnt0_set(conhdl,0);
                    ble_txccmpktcnt1_set(conhdl,0);
                    ble_txccmpktcnt2_set(conhdl,0);
                    ble_rxccmpktcnt0_set(conhdl,0);
                    ble_rxccmpktcnt1_set(conhdl,0);
                    ble_rxccmpktcnt2_set(conhdl,0);

                    // Change the state for the local encryption mechanism
                    llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_START_ENC_RSP;

                    // Send the start encryption response encrypted
                    llc_llcp_start_enc_rsp_pdu_send(conhdl);

                    // Start the response timeout
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);

                } break;

                default:
                {
                    /* nothing to do */
                    ASSERT_INFO(0, conhdl,llc_env[conhdl]->loc_proc_state);
                } break;
            }
        }
    }
    // slave mode encryption handling
    else
    {
        if(llc_state_chk(state, LLC_REM_PROC_BUSY))
        {
            switch(llc_env[conhdl]->rem_proc_state)
            {
                case LLC_REM_WAIT_TP_FOR_PAUSE_ENC_REQ:
                {
                    // Sanity check
                    ASSERT_INFO(lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt) == 0, dest_id, lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt));

                    // A refresh key procedure is now ongoing
                    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_REFRESH_PENDING|LLC_ENC_PAUSE_PENDING, true);

                    llc_env[conhdl]->rem_proc_state = LLC_REM_WAIT_PAUSE_ENC_RSP;

                    // Send the pause response PDU
                    llc_llcp_pause_enc_rsp_pdu_send(conhdl);
                    // Start the response timeout
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                } break;

                case LLC_REM_WAIT_RANDN_GEN_IND:
                {
                    struct llc_llcp_recv_ind *enc_req =  (struct llc_llcp_recv_ind *)llc_util_get_operation_ptr(conhdl, LLC_OP_ENCRYPT);
                    struct llcp_enc_req *pdu = &(enc_req->pdu.enc_req);

                    llc_env[conhdl]->rem_proc_state = LLC_REM_WAIT_LTK;

                    // Copy the SKDm from the PDU to the local env
                    memcpy(&llc_env[conhdl]->encrypt.skd.skd[0], &pdu->skdm.skdiv[0], SESS_KEY_DIV_LEN);
                    // Set the IVm in the control structure
                    llc_util_ivm_set(conhdl, &pdu->ivm.iv[0]);

                    // Send the encryption response
                    llc_llcp_enc_rsp_pdu_send(conhdl, pdu);

                    // And notify the HL with the Rand and EDIV fields.
                    llc_ltk_req_send(conhdl, pdu);

                    // Clear Operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_ENCRYPT);
                } break;

                case LLC_REM_WAIT_TP_FOR_ENC_REQ:
                {
                    // Sanity check
                    ASSERT_INFO(lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt) == 0, dest_id, lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt));

                    // Check if the LTK request event is filtered or not
                    if (llm_util_check_evt_mask(LE_LG_TR_KEY_REQ_EVT_BIT))
                    {
                        // Generate the random number before sending the LLCP
                        llc_task_random_gen_request(dest_id);
                        llc_env[conhdl]->rem_proc_state = LLC_REM_WAIT_RANDN_GEN_IND;
                    }
                    else
                    {
                        // If the Host does not provide a Long Term Key, because the event to the
                        // Host was masked out the slave shall:
                        //   1. If the Pause Encryption Procedure was executed before restarting
                        //      the encryption, initiate the Termination Procedure
                        //   2. Otherwise, send an LL_REJECT_IND PDU to abort the Encryption Start
                        //      procedure
                        // In both cases, the reason shall be set to "PIN or key Missing".
                        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_REFRESH_PENDING))
                        {
                            // Key refresh procedure is pending, so initiate the Termination Procedure
                            llc_llcp_terminate_ind_pdu_send(conhdl, COMMON_ERROR_PIN_MISSING);
                        }
                        else
                        {
                            // Check if a local procedure is on-going
                            if (llc_state_chk(state, LLC_LOC_PROC_BUSY))
                            {
                                // Restart the LLCP response timeout timer
                                kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                            }

                            // Initial encryption start procedure is pending, so simply reject
                            llc_llcp_reject_ind_pdu_send(conhdl, LLCP_ENC_REQ_OPCODE, COMMON_ERROR_PIN_MISSING);

                            // return remote procedure in idle mode
                            llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, false);
                            llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, false);
                            llc_env[conhdl]->rem_proc_state = LLC_REM_IDLE;
                        }
                        // Clear Operation
                        llc_util_clear_operation_ptr(conhdl, LLC_OP_ENCRYPT);
                    }
                } break;

                case LLC_REM_WAIT_SK:
                {
                    // If we are slave, send the LL_START_ENC_REQ encrypted
                    llc_llcp_start_enc_req_pdu_send(conhdl);
                    llc_env[conhdl]->rem_proc_state = LLC_REM_WAIT_START_ENC_RSP;
                    // Start the response timeout
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                } break;

                default:
                {
                    /* nothing to do */
                    ASSERT_INFO(0, conhdl,llc_env[conhdl]->rem_proc_state);
                } break;
            }
        }
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the link supervision time out.
 * The handler is in charge to change the state of the current logical link controller
 * task and to notify the cancellation of the link to the higher layers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_link_sup_to_ind_handler(kernel_msg_id_t const msgid,
                                       void const *param,
                                       kernel_task_id_t const dest_id,
                                       kernel_task_id_t const src_id)
{

    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    kernel_state_t state = kernel_state_get(dest_id);

    // check if state is Free
    if(llc_state_chk(state, LLC_FREE))
    {
        // nothing to do
    }
    else
    {
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_CON_TIMEOUT);
    }

    return (KERNEL_MSG_CONSUMED);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the authenticated payload 'nearly' time out.
 * The handler is in charge to change the state of the current logical link controller
 * task and to notify the cancellation of the link to the higher layers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_auth_payl_nearly_to_ind_handler(kernel_msg_id_t const msgid,
                                               void const *param,
                                               kernel_task_id_t const dest_id,
                                               kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t state = kernel_state_get(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
    {
        if(llc_env[conhdl]->loc_proc_state != LLC_LOC_WAIT_PING_RSP)
        {
            // process this message later
            msg_status = KERNEL_MSG_SAVED;
        }
        // else ignore message
    }
    // check if traffic is paused
    else if(llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        // check if LLCP RSP timeout timer is started
        if (!kernel_timer_active(LLC_LLCP_RSP_TO, dest_id))
        {
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
            llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_PING_RSP;

            llc_llcp_ping_req_pdu_send(conhdl);
            // Start the LLCP Response TO
            kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the authenticated payload 'real' time out.
 * The handler is in charge to change the state of the current logical link controller
 * task and to notify the cancellation of the link to the higher layers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_auth_payl_real_to_ind_handler(kernel_msg_id_t const msgid,
                                             void const *param,
                                             kernel_task_id_t const dest_id,
                                             kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    else
    {
        kernel_timer_set(LLC_AUTH_PAYL_NEARLY_TO, dest_id, llc_env[conhdl]->auth_payl_to
                     - llc_env[conhdl]->auth_payl_to_margin);
        kernel_timer_set(LLC_AUTH_PAYL_REAL_TO, dest_id, llc_env[conhdl]->auth_payl_to);

        // allocate the event message
        struct hci_auth_payl_to_exp_evt *event =
                KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_PAYL_TO_EXP_EVT_CODE, hci_auth_payl_to_exp_evt);

        // gets connection handle
        event->conhdl= common_htobs(conhdl);

        // send the message
        hci_send_2_host(event);
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif //#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the LLCP response time out.
 * The handler is in charge to change the state of the current logical link controller
 * task and to notify the cancellation of the link to the higher layers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_llcp_rsp_to_ind_handler(kernel_msg_id_t const msgid,
                                       void const *param,
                                       kernel_task_id_t const dest_id,
                                       kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);

    llc_util_dicon_procedure(conhdl, COMMON_ERROR_LMP_RSP_TIMEOUT);

    return (KERNEL_MSG_CONSUMED);
}


#if (BLE_CHNL_ASSESS)
/**
 ****************************************************************************************
 * @brief
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_chnl_assess_timer_handler(kernel_msg_id_t const msgid,
                                         void const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    //Gets environment link to the conhdl
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    DBG_SWDIAG(ASSESS_MECH, ATIMER, 1);

    // Message status
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // nothing to do
    }
    // wait instant on-going
    else if(llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_MAP_UPD_INSTANT)
    {
        //Restart assessment timer
        kernel_timer_set(LLC_CHNL_ASSESS_TO, dest_id, llm_util_ch_assess_get_assess_timer());
    }
    else
    {
        // New channel map to be used
        struct le_chnl_map new_map;
        // Host channel map
        struct le_chnl_map host_map;

        uint8_t nb_chgood = 0;

        // Gets the host channel map
        llm_util_get_channel_map(&host_map);

        // Gets the assessment channel map
        nb_chgood = llc_ch_assess_get_local_ch_map(conhdl, &new_map, &host_map);

        // Decrement the channel reassessment counter
        llc_env_ptr->chnl_assess.reassess_count--;

        /*
         * To avoid removing for ever the channel after a a number of assessment timer
         * we reallocate all the channels
         */
        if (!llc_env_ptr->chnl_assess.reassess_count || (nb_chgood < 2))
        {
            // reassess channels
            llc_ch_assess_reass_ch(conhdl, &new_map, &host_map, nb_chgood);
        }

        // set new channel map
        memcpy(&llc_env_ptr->n_ch_map, &new_map, LE_CHNL_MAP_LEN);

        // handle to send the channel map update
        kernel_msg_send_basic(LLC_CHMAP_UPDATE_REQ_IND, dest_id, dest_id);

        //Restart assessment timer
        kernel_timer_set(LLC_CHNL_ASSESS_TO, dest_id, llm_util_ch_assess_get_assess_timer());
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif //(BLE_CHNL_ASSESS)


/**
 ****************************************************************************************
 * @brief Handles the parameters or connection update request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_con_upd_req_ind_handler(kernel_msg_id_t const msgid,
                                       struct llc_con_upd_req_ind *param,
                                       kernel_task_id_t const dest_id,
                                       kernel_task_id_t const src_id)
{
	//UART_PRINTF("%s\r\n", __func__);
	
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);


    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // Nothing to do
    }
    // check if local procedure is on-going
    else if(   (llc_state_chk(state, LLC_LOC_PROC_BUSY) && (param != llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD)))
            // check if remote connection update not already received
            || (llc_state_chk(state, LLC_REM_PROC_BUSY) && (llc_env[conhdl]->rem_proc_state == LLC_REM_WAIT_CON_UPD_INSTANT))
            // check if traffic is paused
            || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

        switch(param->operation)
        {
            case LLC_CON_UP_MOVE_ANCHOR:
            {
                // retrieve supervision timeout
                param->superv_to = llc_env[conhdl]->sup_to;
            }
            // no break
            case LLC_CON_UP_HCI_REQ:
            {
                #if !(BLE_QUALIF)
                // forced to no prefered periodicity
                param->pref_period = 0;
                if(!(   GETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH)
                     && (!(llc_env_ptr->feats_used.feats[0] & BLE_CON_PARAM_REQ_PROC_FEATURE))
                     && (llm_util_check_evt_mask(LE_REM_CON_PARA_REQ_EVT_BIT))))
                {
                    if(param->operation != LLC_CON_UP_MOVE_ANCHOR)
                    {
                        // Connection parameter update is requested by the Host
                        SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ, true);

                        #if (BLE_TESTER)
                        if (llc_env_ptr->tester_params.tester_feats & LLC_TESTER_SURCHARGE_PARAM_REQ)
                        {
                            param->pref_period = llc_env_ptr->tester_params.pref_period;
                            param->offset0     = llc_env_ptr->tester_params.offset0;
                            param->offset1     = llc_env_ptr->tester_params.offset1;
                            param->offset2     = llc_env_ptr->tester_params.offset2;
                            param->offset3     = llc_env_ptr->tester_params.offset3;
                            param->offset4     = llc_env_ptr->tester_params.offset4;
                            param->offset5     = llc_env_ptr->tester_params.offset5;
                        }
                        #endif // (BLE_TESTER)

                        lld_con_param_req(conhdl, llc_env_ptr->elt, param);
                    }

                    llc_llcp_con_param_req_pdu_send(conhdl, param);

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
                    llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_CON_PARAM_RSP;

                    // Start the LLCP Response TO
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);

                    llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD, (void*)param);

                    msg_status = KERNEL_MSG_NO_FREE;
                    break;
                }
                else
                #endif // !(BLE_QUALIF)
                {
                    // force interval parameter that will be applied during connection update
                    param->interval_min = param->con_intv_min;
                    param->interval_max = param->con_intv_max;
                }
            }
            // no break
            case LLC_CON_UP_FORCE:
            {
                #if (BLE_CENTRAL)
                if (lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
                {
                    // Send connection update request
                    struct llcp_con_upd_ind con_upd_pdu;

                    // Give to the driver the instant parameter and wait the update of the interval parameter
                    lld_con_update_req(llc_env_ptr->elt, param, &con_upd_pdu);

                    // update the environment variable
                    llc_env_ptr->n_sup_to = param->superv_to;

                    // request to the driver to send the pdu
                    llc_llcp_con_update_pdu_send(conhdl, &con_upd_pdu);

                    // store parameter used
                    param->interval_min = con_upd_pdu.interv;
                    param->con_latency  = con_upd_pdu.latency;

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
                    llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_CON_UPD_INSTANT;
                    // Clear Operation message free by the kernel
                    llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD, (void*)param);
                    msg_status = KERNEL_MSG_NO_FREE;
                }
                #endif //(BLE_CENTRAL)
            } break;

            #if !(BLE_QUALIF)
            //#if (BLE_CENTRAL) //fix 4.2 updata param bug 2017.04.7 sean
            case LLC_CON_UP_PEER_REQ:
            {
                // allocate the event message
                struct hci_le_rem_con_param_req_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_rem_con_param_req_evt);

                // fill event parameters
                event->subcode      = HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE;
                event->conhdl       = common_htobs(conhdl);
                event->interval_min = param->interval_min;
                event->interval_max = param->interval_max;
                event->latency      = param->con_latency;
                event->timeout      = param->superv_to;

                // send the message
                hci_send_2_host(event);
                //Save the request for future used (when a positive command is sent by the host)
                llc_util_set_operation_ptr(conhdl, LLC_OP_REM_PARAM_UPD, (void*)param);

                llc_env_ptr->rem_proc_state = LLC_REM_WAIT_CON_PARAM_HOST_RSP;
                llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
                msg_status = KERNEL_MSG_NO_FREE;
            }break;
            case LLC_CON_UP_LOC_REQ:
            {

                // Send connection update request
                struct llcp_con_upd_ind con_upd_pdu;

                 lld_con_update_after_param_req(conhdl, llc_env_ptr->elt, param, &con_upd_pdu, true);

                 /*
                  * Give to the driver the instant parameter and wait the update of the interval
                  * parameter
                  */
                 // update the environment variable
                 llc_env_ptr->n_sup_to = param->superv_to;

                 // request to the driver to send the pdu
                 llc_llcp_con_update_pdu_send(conhdl, &con_upd_pdu);

                 // store parameter used
                 param->interval_min = con_upd_pdu.interv;
                 param->con_latency  = con_upd_pdu.latency;

                 // Set the state of the LLC
                 llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
                 llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_CON_UPD_INSTANT;
                 // Clear Operation message free by the kernel
                 llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD, (void*)param);
                 msg_status = KERNEL_MSG_NO_FREE;

            } break;
           //#endif //(BLE_CENTRAL)
            #endif //#if !(BLE_QUALIF)

            default:
            {
            } break;
        }
    }

    return(msg_status);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the automatic length request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_length_req_ind_handler(kernel_msg_id_t const msgid,
                                      void const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {
        // LLC is IDLE, discard the message
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
        struct hci_le_set_data_len_cmd *length_req_ptr = (struct hci_le_set_data_len_cmd*)llc_util_get_operation_ptr(conhdl,LLC_OP_DLE_UPD);
        struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
        if(length_req_ptr != NULL)//Case where the host has started automatically the procedure
        {
            llc_env_ptr->data_len_ext_info.conn_max_tx_octets = length_req_ptr->tx_octets;
            llc_env_ptr->data_len_ext_info.conn_max_tx_time = length_req_ptr->tx_time;

        }
        llc_util_clear_operation_ptr(conhdl, LLC_OP_DLE_UPD);

        //If the procedure is on going or done, skip this phase
        if(!(GETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_REQ_RCVD)))
        {
            // Set the state of the LLC
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
            llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_LENGTH_RSP;
            //Send length request to the peer
            llc_llcp_length_req_pdu_send(conhdl);

            // Start the LLCP Response TO
            kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
        }
    }

    return(msg_status);
}
#if (BLE_2MBPS)
/**
 ****************************************************************************************
 * @brief Handles the automatic length request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_phy_upd_req_ind_handler(kernel_msg_id_t const msgid,
                                      struct llc_phy_upd_req_ind *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY)|| (!llm_util_check_evt_mask(LE_PHY_UPD_CMP_EVT_BIT)))
    {
        // LLC is IDLE, discard the message
    }
    // check if another local procedure is on-going
    else if( (llc_state_chk(state, LLC_LOC_PROC_BUSY) && !llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD))
            // check if traffic is paused
            || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        switch(param->operation)
        {
            case LLC_PHY_UPD_HCI_REQ:
            {
                // a remote procedure for PHY update is on-going
                if(llc_state_chk(state, LLC_REM_PROC_BUSY)
                   && (   (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PHY_UPD_REQ)
                       || (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PHY_UPD_INSTANT)
                       || (llc_env_ptr->rem_proc_state == LLC_REM_PHY_NO_INSTANT)))
                {
                    // process this message later
                    msg_status = KERNEL_MSG_SAVED;
                }
                else
                {
                    struct llcp_phy_req phy_req;

                    uint8_t rx_phy_dft, tx_phy_dft;

                    llm_util_get_default_phy(&tx_phy_dft, &rx_phy_dft);

                    if (!(param->all_phys & ALL_PHYS_TX_NO_PREF))
                    {
                        phy_req.tx_phys = (param->tx_phys & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                    }
                    else
                    {
                        phy_req.tx_phys = (tx_phy_dft  & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                    }

                    if (!(param->all_phys & ALL_PHYS_RX_NO_PREF))
                    {
                        phy_req.rx_phys = (param->rx_phys  & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                    }
                    else
                    {
                        phy_req.rx_phys = (rx_phy_dft  & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                    }

                    // Update the value by the one choosen
                    param->tx_phys = phy_req.tx_phys;
                    param->rx_phys = phy_req.rx_phys;

                    llc_llcp_phy_req_pdu_send(conhdl, &phy_req);

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);

                    if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
                    {
                        llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_PHY_RSP;
                    }
                    else
                    {
                        llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_PHY_UPD_REQ;
                    }

                    // Start the LLCP Response TO
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);

                    llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD, (void*)param);

                    msg_status = KERNEL_MSG_NO_FREE;
                }
            }break;
            case LLC_PHY_UPD_PEER_RSP:
            {
                // LLCP phy_upd_req to be sent
                struct  llcp_phy_upd_req llcp;
                /**
                 * Get the response from the peer merge with the local, compute the instant and send the update phy request
                 */
                struct llc_phy_upd_req_ind *phy_local_update = (struct llc_phy_upd_req_ind *)llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD);
                uint8_t tx_phy, rx_phy;

                ASSERT_ERR(lld_get_mode(conhdl) == LLD_EVT_MST_MODE);
                ASSERT_ERR(llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_PHY_RSP);

                // Init phys
                llcp.m_to_s_phy = PHYS_NO_PREF;
                llcp.s_to_m_phy = PHYS_NO_PREF;

                if (!(phy_local_update->all_phys & ALL_PHYS_TX_NO_PREF))
                {
                    llcp.m_to_s_phy = ((phy_local_update->tx_phys & param->rx_phys) & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                }
                if (!(phy_local_update->all_phys & ALL_PHYS_RX_NO_PREF))
                {
                    llcp.s_to_m_phy = ((phy_local_update->rx_phys & param->tx_phys) & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                }

                lld_util_get_phys(llc_env_ptr->elt, &tx_phy, &rx_phy);

                //If both the M_TO_S_PHY and S_TO_M_PHY fields are zero then there is no Instant
                if(((rx_phy == llcp.s_to_m_phy) || (llcp.s_to_m_phy == PHYS_NO_PREF))
                        && ((tx_phy == llcp.m_to_s_phy) || (llcp.m_to_s_phy == PHYS_NO_PREF)))
                {
                    llcp.instant = 0;
                    llcp.m_to_s_phy = PHYS_NO_PREF;
                    llcp.s_to_m_phy = PHYS_NO_PREF;

                    llc_env[conhdl]->loc_proc_state = LLC_LOC_PHY_NO_INSTANT;
                    // Get the
                    // Set state
                    llc_phy_update_finished(conhdl , tx_phy , rx_phy, COMMON_ERROR_NO_ERROR,phy_local_update->operation);
                    //clear operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD);
                }
                else
                {
                    uint8_t m_to_s, s_to_m;
                    m_to_s = llcp.m_to_s_phy;
                    s_to_m = llcp.s_to_m_phy;
                    if(tx_phy == llcp.m_to_s_phy)
                    {
                        llcp.m_to_s_phy = PHYS_NO_PREF;
                    }
                    else if (rx_phy == llcp.s_to_m_phy)
                    {
                        llcp.s_to_m_phy = PHYS_NO_PREF;
                    }
                    // Give to the driver the instant parameter and wait the update of the phy
                    llcp.instant = lld_util_instant_get((void*)LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt), (uint8_t)LLD_UTIL_PHY_UPDATE);
                    // Set the Phys in the environment
                    lld_util_phy_update_req(llc_env_ptr->elt, llcp.instant, m_to_s, s_to_m);
                    // Set state
                    llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_PHY_UPD_INSTANT;
                }
                // Compute instant and send the LLCP
                llc_llcp_phy_upd_ind_pdu_send(conhdl, &llcp);
            }break;
            case LLC_PHY_UPD_PEER_REQ:
            {
                struct  llcp_phy_upd_req llcp;
                uint8_t tx_phy, rx_phy;
                // Set the state of the LLC
                llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
                // release remote procedure currently locked
                llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, false);
                llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD, (void*)param);
                msg_status = KERNEL_MSG_NO_FREE;

                lld_util_get_phys(llc_env_ptr->elt, &tx_phy, &rx_phy);

                //Gets local information of the phys
                llm_util_get_default_phy(&llcp.m_to_s_phy, &llcp.s_to_m_phy);

                llcp.m_to_s_phy = ((llcp.m_to_s_phy & param->rx_phys) & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;
                llcp.s_to_m_phy = ((llcp.s_to_m_phy & param->tx_phys) & PHYS_2MBPS_PREF)?PHYS_2MBPS_PREF:PHYS_1MBPS_PREF;

                //If both the M_TO_S_PHY and S_TO_M_PHY fields are zero then there is no Instant
                if((llcp.m_to_s_phy == PHYS_NO_PREF) && (llcp.s_to_m_phy == PHYS_NO_PREF))
                {
                    llcp.instant = 0;
                    // No wait
                    llc_env[conhdl]->loc_proc_state = LLC_LOC_PHY_NO_INSTANT;
                }
                else
                {
                    //If a PHY is remaining unchanged, then the corresponding field shall be set to 0
                    if((tx_phy == llcp.m_to_s_phy) && (rx_phy == llcp.s_to_m_phy))
                    {
                        // No wait
                        llc_env[conhdl]->loc_proc_state = LLC_LOC_PHY_NO_INSTANT;
                        //No the switch instant
                        llcp.instant = 0;
                        llcp.m_to_s_phy = PHYS_NO_PREF;
                        llcp.s_to_m_phy = PHYS_NO_PREF;
                    }
                    else
                    {
                        uint8_t m_to_s, s_to_m;
                        m_to_s = llcp.m_to_s_phy;
                        s_to_m = llcp.s_to_m_phy;
                        if(tx_phy == llcp.m_to_s_phy)
                        {
                            llcp.m_to_s_phy = PHYS_NO_PREF;
                        }
                        else if (rx_phy == llcp.s_to_m_phy)
                        {
                            llcp.s_to_m_phy = PHYS_NO_PREF;
                        }

                        //Wait the switch instant
                        llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_PHY_UPD_INSTANT;

                        llcp.instant    = lld_util_instant_get((void*)LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt), (uint8_t)LLD_UTIL_PHY_UPDATE);
                        // Set the state of the LLC
                        lld_util_phy_update_req(llc_env_ptr->elt, llcp.instant, m_to_s, s_to_m);
                    }
                }
                llc_llcp_phy_upd_ind_pdu_send(conhdl, &llcp);

                // operation is finished
                if(llc_env[conhdl]->loc_proc_state == LLC_LOC_PHY_NO_INSTANT)
                {
                    //clear operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD);
                    llc_phy_update_finished(conhdl , tx_phy , rx_phy, COMMON_ERROR_NO_ERROR, LLC_PHY_UPD_PEER_REQ);
                }
            }break;
            case LLC_PHY_UPD_TERMINATE:
            {
                /**
                 * Get the response from the peer merge with the local, compute the instant and send the update phy request
                 */
                struct llc_phy_upd_req_ind *phy_local_update = (struct llc_phy_upd_req_ind *)llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD);
                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);
                uint8_t operation;

                // Check if the procedure has been started by the controller
                if(phy_local_update)
                {
                    operation = phy_local_update->operation;
                }
                else
                {
                    operation = LLC_PHY_UPD_TERMINATE;
                }


                //Send or not the event to the host and initialize the states
                llc_phy_update_finished(conhdl , evt->evt.conn.tx_phy, evt->evt.conn.rx_phy, param->status, operation);

                // Clear Operation without check, in the clear function the memory free is only done if operation is allocated
                llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD);

            }break;
            default:break;
        }
    }

    return(msg_status);
}
#endif // (BLE_2MBPS)
#endif//#if !(BLE_QUALIF)

#if (BLE_CHNL_ASSESS)
/**
 ****************************************************************************************
 * @brief Handles the channel map update request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_chmap_update_req_ind_handler(kernel_msg_id_t const msgid,
                                      void const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY)
       // check if traffic is paused --> ignore request for the moment
       || llc_state_chk(state, LLC_TRAFFIC_PAUSED_BUSY))
    {
        // discard the message
    }
    // check if local procedure is on-going
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
    {
        // process this message later
        msg_status = KERNEL_MSG_SAVED;
    }
    else
    {
        struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
        // Channel map currently used
        struct le_chnl_map *current_map = llc_ch_assess_get_current_ch_map(conhdl);

        // Check if the new map is not equal to the current
        if (memcmp(current_map, &llc_env_ptr->n_ch_map, LE_CHNL_MAP_LEN))
        {
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, true);
            llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_MAP_UPD_INSTANT;

            // Send the Channel Map Update PDU
            llc_llcp_ch_map_update_pdu_send(conhdl);
        }
    }

    return(msg_status);
}
#endif // (BLE_CHNL_ASSESS)

/**
 ****************************************************************************************
 * @brief Handles the AES128 encrypted data received.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_enc_ind_handler(kernel_msg_id_t const msgid,
                               struct llm_enc_ind const *param,
                               kernel_task_id_t const dest_id,
                               kernel_task_id_t const src_id)
{

    uint16_t conhdl = KERNEL_IDX_GET(dest_id);
    uint8_t state = kernel_state_get(dest_id);
    uint8_t idx;
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint8_t i;



    // check if state is Free or in disconnected state
    if(llc_state_chk(state, LLC_DISC_BUSY))
    {

    }
    else if( ((llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_RANDN_GEN_IND) && (lld_get_mode(conhdl) == LLD_EVT_MST_MODE))
            || ((llc_env_ptr->rem_proc_state == LLC_REM_WAIT_RANDN_GEN_IND) && (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)))
    {
        // Nothing to do, ignore the message
        memcpy(&llc_env_ptr->encrypt.randn[0], &param->encrypted_data[0], KEY_LEN);
        // device ready to continue encryption management
        kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
    }
    else if(  ((lld_get_mode(conhdl) == LLD_EVT_SLV_MODE) && llc_state_chk(state, LLC_REM_PROC_BUSY)
                && (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_SK))
           || ((lld_get_mode(conhdl) == LLD_EVT_MST_MODE) && llc_state_chk(state, LLC_LOC_PROC_BUSY)
                && ((llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_SK_AND_START_ENC_REQ) || (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_SK))))
    {
        // SK saves in the CS
        for (idx = 0, i=15; idx < 8; idx++)
        {
            // encrypted_data is LSB first and  SK is MSB first
            ble_sk_setf(conhdl, idx, (param->encrypted_data[i-1] << 8) | param->encrypted_data[i]);
            i -= 2;
        }
        // device ready to continue encryption management
        kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
    }
    else
    {
        // Trash indication we are no more in the good state to received it (procedure rejected)
    }

    return(KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the AES128 encrypted data received.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int lld_stop_ind_handler(kernel_msg_id_t const msgid,
                                void const *param,
                                kernel_task_id_t const dest_id,
                                kernel_task_id_t const src_id)
{
    if (kernel_state_get(dest_id) != LLC_FREE)
    {
        // Get Connection Handle
        uint16_t conhdl = KERNEL_IDX_GET(dest_id);
        // Clear encryption environment, reset the encryption bits
        ble_link_set(conhdl, ble_link_get(conhdl) & ~(BLE_TXCRYPT_EN_BIT | BLE_RXCRYPT_EN_BIT));

        struct llc_env_tag* llc_env_ptr = llc_env[conhdl];
        uint8_t reason = llc_env_ptr->disc_reason;
        uint8_t state = kernel_state_get(dest_id);

        if (state != LLC_DISC_BUSY)
        {
            // we go there only if we detect a window widening issue ==> which is considered as connection timeout.
            reason = (GETF(llc_env[conhdl]->llc_status, LLC_STAT_SYNC_FOUND))?COMMON_ERROR_CON_TIMEOUT:COMMON_ERROR_CONN_FAILED_TO_BE_EST;;
        }

        if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
        {

			// Check state
            switch (llc_env_ptr->loc_proc_state)
            {
                case LLC_LOC_WAIT_FEAT_RSP: // feature request
                {
                    // Send the meta event with the values received from the peer
                    llc_feats_rd_event_send(reason, conhdl, &(llc_env_ptr->feats_used));
                }
                break;
                case LLC_LOC_WAIT_VERS_IND: // version exchange
                {
                    llc_version_rd_event_send(reason, conhdl);
                }
                break;
                // Encryption procedure
                case LLC_LOC_WAIT_TRAFFIC_PAUSED:
                case LLC_LOC_WAIT_PAUSE_ENC_RSP:
                case LLC_LOC_WAIT_PAUSE_ENC_RSP_SENT:
                case LLC_LOC_WAIT_ENC_RSP:
                case LLC_LOC_WAIT_SK_AND_START_ENC_REQ:
                case LLC_LOC_WAIT_SK:
                case LLC_LOC_WAIT_START_ENC_REQ:
                case LLC_LOC_SEND_START_ENC_RSP:
                case LLC_LOC_WAIT_START_ENC_RSP:
                case LLC_LOC_WAIT_RANDN_GEN_IND:
                {
                    // Send the appropriate event to the host
                    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_REFRESH_PENDING))
                    {
                        // Send key refresh complete event
                        llc_common_enc_key_ref_comp_evt_send(conhdl, reason);
                    }
                    else
                    {
                        // Send encryption change event
                        llc_common_enc_change_evt_send(conhdl, 0, reason);
                    }
                } break;

                // parameter update
                case LLC_LOC_WAIT_CON_PARAM_RSP:
                case LLC_LOC_WAIT_CON_UPD_REQ:
                case LLC_LOC_WAIT_CON_UPD_INSTANT:
                {
                    // Checks if the event is not filtered
                    if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT))
                    {
                        // check if requested by host else ignore
                        if (GETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ))
                        {
                            // Send the command complete event
                            llc_con_update_complete_send(reason, conhdl, NULL);
                        }
                    }
                } break;

                case LLC_LOC_WAIT_LENGTH_RSP:
                {
                    // nothing to do
                } break;

                #if (BLE_2MBPS)
                case LLC_LOC_WAIT_PHY_RSP:
                case LLC_LOC_WAIT_PHY_UPD_REQ:
                case LLC_LOC_WAIT_PHY_UPD_INSTANT:
                case LLC_LOC_PHY_NO_INSTANT:
                {
                    llc_phy_update_complete_send(reason, conhdl, 0, 0);
                } break;
                #endif// (BLE_2MBPS)

                default: /* Nothing to do */ break;
            }
        }


        // The Link Layer notifies the host of the loss of connection.
        llc_discon_event_complete_send(dest_id, COMMON_ERROR_NO_ERROR, conhdl, reason);

        // Clear pending timers
        kernel_timer_clear(LLC_LE_LINK_SUP_TO, dest_id);
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
        kernel_timer_clear(LLC_AUTH_PAYL_NEARLY_TO, dest_id);
        kernel_timer_clear(LLC_AUTH_PAYL_REAL_TO, dest_id);

        // Stop LLC task
        llc_stop(conhdl);
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_data_ind_handler(kernel_msg_id_t const msgid,
                                struct llc_data_ind const *param,
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
        // LLC is IDLE or STOPPING, discard the data (simply free it)
        em_buf_rx_free(param->rx_hdl);
    }
    // Check if data reception is allowed
    else if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // Data flow is not allowed, so terminate the connection with reason
        // "MIC Failure"
        // Free the received buffer
        em_buf_rx_free(param->rx_hdl);


        // Initiate the termination procedure
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else
    {
        struct kernel_msg * msg = kernel_param2msg(param);

        // Data reception allowed, so forward the message to host
        msg->id = HCI_ACL_DATA_RX;
        msg->dest_id = conhdl;
        hci_send_2_host((void*) param);
        msg_status = KERNEL_MSG_NO_FREE;
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles reception of unexpected messages
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_dft_handler(kernel_msg_id_t const msgid,
                           void *param,
                           kernel_task_id_t const dest_id,
                           kernel_task_id_t const src_id)
{
    // Message is not expected, drop the message
    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles any LLCP message
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_llcp_recv_ind_handler(kernel_msg_id_t const msgid,
                                     struct llc_llcp_recv_ind *ind,
                                     kernel_task_id_t dest_id,
                                     kernel_task_id_t src_id)
{
    // use the default llcp handler
    return llc_llcp_recv_handler(dest_id, ind->status, &(ind->pdu), false);
}



/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
static const struct kernel_msg_handler llc_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KERNEL_MSG_DEFAULT_HANDLER,    (kernel_msg_func_t)llc_dft_handler},

    // save the indication if not in the good state
    {LLD_STOP_IND,              (kernel_msg_func_t)lld_stop_ind_handler},

    /*
     * ************** Msg HCI->LLC ****************
     */
    {HCI_COMMAND,               (kernel_msg_func_t)llc_hci_command_handler},
    // Data sent to the peer
    {HCI_ACL_DATA_TX,           (kernel_msg_func_t)llc_hci_acl_data_tx_handler},

    /*
     * ************** Msg LLC->HL *****************
     */
    // Data received from the peer
    {LLC_DATA_IND,              (kernel_msg_func_t)llc_data_ind_handler},

    /*
     * ************** Msg LLC->LLC ****************
     */
    // Time out and termination TO
    {LLC_LE_LINK_SUP_TO,        (kernel_msg_func_t)llc_link_sup_to_ind_handler},
    #if !(BLE_QUALIF)
    // Authenticated Payload Timeout
    {LLC_AUTH_PAYL_NEARLY_TO,   (kernel_msg_func_t)llc_auth_payl_nearly_to_ind_handler},
    // Authenticated Payload Timeout
    {LLC_AUTH_PAYL_REAL_TO,     (kernel_msg_func_t)llc_auth_payl_real_to_ind_handler},
    #endif //#if !(BLE_QUALIF)
    // Response Timeout
    {LLC_LLCP_RSP_TO,           (kernel_msg_func_t)llc_llcp_rsp_to_ind_handler},
    #if (BLE_CHNL_ASSESS)
    // Channel assessment timer expiration
    {LLC_CHNL_ASSESS_TO,        (kernel_msg_func_t)llc_chnl_assess_timer_handler},
    #endif //(BLE_CHNL_ASSESS)
    // A pause resume mechanism is on going
    {LLC_ENC_MGT_IND,           (kernel_msg_func_t)llc_enc_mgt_ind_handler},
    #if !(BLE_QUALIF)
    // indication that a length_req is needed
    {LLC_LENGTH_REQ_IND,        (kernel_msg_func_t)llc_length_req_ind_handler},
    #endif //#if !(BLE_QUALIF)
    // indication that a length_req is needed
    {LLC_CON_UPD_REQ_IND,       (kernel_msg_func_t)llc_con_upd_req_ind_handler},
    #if (BLE_CHNL_ASSESS)
    // Channel assessment timer expiration
    {LLC_CHMAP_UPDATE_REQ_IND,  (kernel_msg_func_t)llc_chmap_update_req_ind_handler},
    #endif //(BLE_CHNL_ASSESS)
    #if ((BLE_2MBPS) && !(BLE_QUALIF))
    {LLC_PHY_UPD_REQ_IND,       (kernel_msg_func_t)llc_phy_upd_req_ind_handler},
    #endif //(BLE_2MBPS)

    /*
     * ************** LLCP messages **************
     */
    {LLC_LLCP_RECV_IND,         (kernel_msg_func_t)llc_llcp_recv_ind_handler},

    /*
     * ************** Msg LLM->LLC **************
     */
    // Data has been encrypted by the AES
    {LLM_ENC_IND,               (kernel_msg_func_t)llm_enc_ind_handler},
};


/// Specifies the message handlers that are common to all states.
const struct kernel_state_handler llc_default_handler = KERNEL_STATE_HANDLER(llc_default_state);

/// Defines the placeholder for the states of all the task instances.
kernel_state_t llc_state[LLC_IDX_MAX];

#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCTASK
