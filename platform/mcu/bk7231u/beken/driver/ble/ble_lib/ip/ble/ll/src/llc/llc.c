/**
 ****************************************************************************************
 *
 * @file llc.c
 *
 * @brief Definition of the functions used by the logical link controller
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLC
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ble_compiler.h"
#include "em_buf.h"
#include "llc_llcp.h"
#include "lld.h"
#include "lld_pdu.h"
#include "lld_util.h"
#include "llm.h"
#include "reg_ble_em_cs.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_ble_em_ral.h"
#include "llc_task.h"
#include "llc_util.h"
#include "llcontrl.h"

#include "common_bt.h"
#include "common_endian.h"
#include "kernel_mem.h"
#include "kernel_task.h"
#include "kernel_timer.h"
#include "llm_util.h"
#if (HCI_PRESENT)
#include "hci.h"
#endif //(HCI_PRESENT)
#if (BLE_CHNL_ASSESS && NVDS_SUPPORT)
#include "nvds.h"
#endif //(BLE_CHNL_ASSESS && NVDS_SUPPORT)
#include "dbg_swdiag.h"


#if (BLE_PERIPHERAL || BLE_CENTRAL)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// LLC environment
struct llc_env_tag* llc_env[BLE_CONNECTION_MAX];

/// LLC task descriptor
static const struct kernel_task_desc TASK_DESC_LLC = {NULL, &llc_default_handler, llc_state, LLC_STATE_MAX, LLC_IDX_MAX};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void llc_init(void)
{
    uint16_t conhdl;

    // Create LLC task
    kernel_task_create(TASK_LLC, &TASK_DESC_LLC);

    // free all llc process
    for (conhdl=0; conhdl < BLE_CONNECTION_MAX; conhdl++)
    {
        // set the state to free
        kernel_state_set(KERNEL_BUILD_ID(TASK_LLC, conhdl), LLC_FREE);
    }

    // Initialize LLC environment
    memset(&llc_env, 0, sizeof(llc_env));
}

void llc_reset(void)
{
    uint16_t conhdl;

    // free all llc process
    for (conhdl=0; conhdl < BLE_CONNECTION_MAX; conhdl++)
    {
        if(kernel_state_get(KERNEL_BUILD_ID(TASK_LLC, conhdl))!= LLC_FREE)
        {
            // Stop LLC task for the link
            llc_stop(conhdl);
        }
    }
}

void llc_start(struct llc_create_con_req_ind *param, struct ea_elt_tag *elt)
{
    // Associted BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    struct llc_env_tag *llc_env_ptr;
    uint16_t conhdl = evt->conhdl;
    kernel_task_id_t llc_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    ASSERT_INFO(llc_env[conhdl] == NULL, conhdl, 0);

    // Check if this llc_env is already used
    if (llc_env[conhdl] == NULL)
    {
        // Allocate a LLC environment structure for the link
        llc_env[conhdl] = (struct llc_env_tag*)kernel_malloc(sizeof(struct llc_env_tag), KERNEL_MEM_ENV);
    }

    llc_env_ptr = llc_env[conhdl];

    // Clear peer version info
    memset(&llc_env_ptr->peer_version, 0, sizeof(struct rem_version));
    #if (BLE_CENTRAL)
    // Initialize channel map
    llm_util_get_channel_map(&llc_env_ptr->ch_map);
    llm_util_get_channel_map(&llc_env_ptr->n_ch_map);
    #if (BLE_CHNL_ASSESS)
    // Channel assessment for Master only
    if(evt->mode == LLD_EVT_MST_MODE)
    {
        // Initialize packet counters
        memset(&llc_env_ptr->chnl_assess, 0, sizeof(struct llc_ch_asses_tag));
        // Set the reassessment counter
        llc_env_ptr->chnl_assess.reassess_count = llm_util_ch_assess_get_reass_cnt();
        // Start timer
        kernel_timer_set(LLC_CHNL_ASSESS_TO, llc_id,llm_util_ch_assess_get_assess_timer());
        //Set if latency enable
        if(evt->evt.conn.latency > 1)
        {
            llc_env_ptr->chnl_assess.latency_en = true;
        }
        else
        {
            llc_env_ptr->chnl_assess.latency_en = false;
        }
    }
    #endif //(BLE_CHNL_ASSESS)
    #endif //(BLE_CENTRAL)
    llc_env_ptr->elt = elt;
    llc_env_ptr->sup_to = param->sup_to;

    // If the Link Layer connection supervision timer reaches 6 * connInterval before
    // the connection is established, the connection shall be considered lost.
    kernel_timer_set(LLC_LE_LINK_SUP_TO, llc_id, llc_env_ptr->sup_to);

    llc_env_ptr->llc_status = 0;
    SETF(llc_env_ptr->llc_status, LLC_STAT_DISC_REM_REQ, false);
    #if (BLE_2MBPS)
    //Enable PHY update procedure
    SETF(llc_env_ptr->llc_status, LLC_STAT_PHY_ENABLED, true);
    #endif // (BLE_2MBPS)

    // send the command complete event
    llc_le_con_cmp_evt_send(COMMON_ERROR_NO_ERROR, conhdl, param);

    // Init Disconnection reason
    // by default LLD can requests disconnection
    // Disconnection is due to WindowWidening becoming too high
    llc_env_ptr->disc_reason = COMMON_ERROR_CON_TIMEOUT;
    llc_env_ptr->disc_event_sent = false;

    // Get the supported features
    llm_util_get_supp_features(&llc_env_ptr->feats_used);

    // Reset Operation
    uint8_t i = 0;
    for( i = 0; i < LLC_OP_MAX; i++)
    {
        llc_env_ptr->operation[i] = NULL;
    }
    // set the current llc task in connected state
    kernel_state_set(llc_id, LLC_CONNECTED);
    llc_env[conhdl]->loc_proc_state = LLC_LOC_IDLE;
    llc_env[conhdl]->rem_proc_state  = LLC_REM_IDLE;

    // Set the default value for the authenticated payload timeout
    llc_env[conhdl]->auth_payl_to = LLC_DFT_AUTH_PAYL_TO;

    #if !(BLE_QUALIF)
    /**
     * LE PING
     * Time out initialization
     */
    // Set an appropriate margin for the authenticated payload timeout
    llc_util_set_auth_payl_to_margin(evt);
    #endif // !(BLE_QUALIF)
    /**
     * DLE
     * Data length extension initialization (core chapter 4.5.10 Data PDU Length Management)
     */
    //By default the request can be sent
    llc_env_ptr->data_len_ext_info.send_req_not_allowed = false;
    //Set data length extension default values
    #if (RW_DEBUG)
    llc_env_ptr->data_len_ext_info.conn_max_tx_octets       = llm_le_env.data_len_val.conn_initial_max_tx_octets;
    llc_env_ptr->data_len_ext_info.conn_max_rx_octets       = llm_le_env.data_len_val.suppted_max_rx_octets;
    llc_env_ptr->data_len_ext_info.conn_max_tx_time         = llm_le_env.data_len_val.conn_initial_max_tx_time;
    llc_env_ptr->data_len_ext_info.conn_max_rx_time         = llm_le_env.data_len_val.suppted_max_rx_time;
    #else
    llc_env_ptr->data_len_ext_info.conn_max_tx_octets       = llm_le_env.data_len_val.conn_initial_max_tx_octets;
    llc_env_ptr->data_len_ext_info.conn_max_rx_octets       = BLE_MAX_OCTETS;
    llc_env_ptr->data_len_ext_info.conn_max_tx_time         = llm_le_env.data_len_val.conn_initial_max_tx_time;
    llc_env_ptr->data_len_ext_info.conn_max_rx_time         = BLE_MAX_TIME;
    #endif

    llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets   = BLE_MIN_OCTETS;//27 bytes
    llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets   = BLE_MIN_OCTETS;//27 bytes
    llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time     = BLE_MIN_TIME;//328 us
    llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time     = BLE_MIN_TIME;//328 us

    SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, true);
    SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_REQ_RCVD, false);

    /**
     * Clear the operation
     */
    llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD, NULL);
    llc_util_set_operation_ptr(conhdl, LLC_OP_REM_PARAM_UPD, NULL);
    llc_util_set_operation_ptr(conhdl, LLC_OP_ENCRYPT, NULL);
    llc_util_set_operation_ptr(conhdl, LLC_OP_DLE_UPD, NULL);
    #if (BLE_2MBPS)
    llc_util_set_operation_ptr(conhdl, LLC_OP_LOC_PHY_UPD, NULL);
    #endif // (BLE_2MBPS)
    /**
     * Initialize encryption/flow dedicated fields
     */
    LLC_UTIL_ENC_STATE_SET(conhdl, LLC_ENC_DISABLED);

    #if (BLE_TESTER)
    // No tester features enabled by default
    llc_env_ptr->tester_params.tester_feats = 0x00;
    /// by default, disable LLCP pass through mechanism
    llc_env_ptr->llcp_pass_through_enable = false;
    #endif // (BLE_TESTER)
}

void llc_stop(uint16_t conhdl)
{
    // Set the state to free
    kernel_state_set(KERNEL_BUILD_ID(TASK_LLC, conhdl), LLC_FREE);

    ASSERT_INFO(llc_env[conhdl] != NULL, conhdl, 0);

    // Check if this llc_env is already used
    if (llc_env[conhdl] != NULL)
    {
        // Free all stored operation messages
    	uint8_t i = 0;
        for ( i = 0; i < LLC_OP_MAX; i++)
        {
            if (llc_env[conhdl]->operation[i])
            {
                // Free the operation message
                kernel_msg_free(kernel_param2msg(llc_env[conhdl]->operation[i]));
            }
        }

        // Free the LLC environment structure for the link
        kernel_free((void *)llc_env[conhdl]);

        // Clear stored pointer
        llc_env[conhdl] = NULL;

        // Clear the bd address in the list for this connection handle
        llm_util_bl_rem(conhdl);
    }
}

void llc_discon_event_complete_send(kernel_task_id_t src_id, uint8_t status, uint8_t conhdl, uint8_t reason)
{
    // allocate the complete event message
    struct hci_disc_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);
    // update the status
    event->status = status;
    // update the reason
    event->reason = reason;
    // update the link handle
    event->conhdl = common_htobs(conhdl);
    // send the message
    hci_send_2_host(event);
}
/*
 * META event
 */

void llc_le_con_cmp_evt_send(uint8_t status, uint16_t conhdl, struct llc_create_con_req_ind *param)
{
    // checks if the event is not filtered
    if(llm_util_check_evt_mask(LE_CON_CMP_EVT_BIT) || llm_util_check_evt_mask(LE_ENH_CON_CMP_EVT_BIT))
    {
        // Send the LE Connection Completed message if LE enhanced connection completed message filtered
        if(!llm_util_check_evt_mask(LE_ENH_CON_CMP_EVT_BIT))
        {
            // allocate the status event message
            struct hci_le_con_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_con_cmp_evt);
            // gets event subcode
            event->subcode = HCI_LE_CON_CMP_EVT_SUBCODE;
            // gets the status
            event->status = status;
            // gets connection handle
            event->conhdl = common_htobs(conhdl);

            if(status == COMMON_ERROR_NO_ERROR)
            {
                bool peer_address_updated = false;
                // gets the role
                event->role = (lld_get_mode(conhdl) == LLD_EVT_MST_MODE) ? ROLE_MASTER : ROLE_SLAVE;

                if(param->ral_ptr != 0)
                {
                    uint8_t ral_idx = (param->ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;

                    // copy peer resolvable address if present
                    if(ble_peer_irk_valid_getf(ral_idx))
                    {
                        if((param->filter_policy == INIT_FILT_IGNORE_WLST) || ble_peer_rpa_valid_getf(ral_idx))
                        {
                            event->peer_addr_type = ADDR_RAND;
                            // Do a burst read in the exchange memory
                            em_rd(&(event->peer_addr), param->ral_ptr + BLE_RAL_PEER_RPA_INDEX*2,  BD_ADDR_LEN);
                            peer_address_updated = true;

                        }
                        else
                        {
                            // Assert if peer RPA invalid
                            ASSERT_ERR(ble_peer_rpa_valid_getf(ral_idx));
                            memset(&(event->peer_addr), 0, sizeof(struct bd_addr));
                        }
                    }
                }

                // check that peer address updated
                if(!peer_address_updated)
                {
                    // gets peer address type
                    event->peer_addr_type = param->peer_addr_type & ADDR_MASK;
                    // gets peer address
                    memcpy(&(event->peer_addr), &(param->peer_addr), BD_ADDR_LEN);
                }

                // gets connection interval
                event->con_interval = common_htobs(param->con_int);
                // gets connection latency
                event->con_latency = common_htobs(param->con_lat);
                // gets supervision time out
                event->sup_to = common_htobs(param->sup_to);
                // gets clock accuracy
                if(event->role == ROLE_MASTER)
                {
                    event->clk_accuracy = 0;
                }
                else
                {
                    event->clk_accuracy = param->sleep_clk_acc;
                }
            }
            else if (status == COMMON_ERROR_DIRECT_ADV_TO)
            {
                // gets the role
                event->role = ROLE_SLAVE;
                // gets peer address type
                event->peer_addr_type = param->peer_addr_type;
                // gets connection interval
                event->con_interval = 0;
                // gets connection latency
                event->con_latency = 0;
                // gets supervision time out
                event->sup_to = 0;
                event->conhdl = 0;
                memcpy(&(event->peer_addr),&(param->peer_addr),BD_ADDR_LEN);
            }

            // send the message
            hci_send_2_host(event);
        }
        // Send the LE Enhanced Connection Completed message
        else
        {
            // allocate the status event message
            struct hci_le_enh_con_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_enh_con_cmp_evt);
            // gets event subcode
            event->subcode = HCI_LE_ENH_CON_CMP_EVT_SUBCODE;

            // gets the status
            event->status = status;
            // gets connection handle
            event->conhdl = common_htobs(conhdl);

            if(status == COMMON_ERROR_NO_ERROR)
            {
                // gets the role
                event->role = (lld_get_mode(conhdl) == LLD_EVT_MST_MODE) ? ROLE_MASTER : ROLE_SLAVE;

                if(param->ral_ptr != 0)
                {
                    uint8_t ral_idx = (param->ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;

                    // copy peer resolvable address if present
                    if(ble_peer_irk_valid_getf(ral_idx))
                    {
                        if((param->filter_policy == INIT_FILT_IGNORE_WLST) || ble_peer_rpa_valid_getf(ral_idx))
                        {
                            // Do a burst read in the exchange memory
                            em_rd(&(event->peer_rslv_priv_addr), param->ral_ptr + BLE_RAL_PEER_RPA_INDEX*2,  BD_ADDR_LEN);

                        }
                        else
                        {
                            // Assert if peer RPA invalid
                            ASSERT_ERR(ble_peer_rpa_valid_getf(ral_idx));
                            memset(&(event->peer_rslv_priv_addr), 0, sizeof(struct bd_addr));
                        }

                    }
                    else
                    {
                        memset(&(event->peer_rslv_priv_addr), 0, sizeof(struct bd_addr));
                    }

                    // copy local resolvable address if present
                    if(ble_local_irk_valid_getf(ral_idx))
                    {
                        // Assert if local RPA invalid
                        ASSERT_ERR(ble_local_rpa_valid_getf(ral_idx));
                        // Do a burst read in the exchange memory
                        em_rd(&(event->loc_rslv_priv_addr), param->ral_ptr + BLE_RAL_LOCAL_RPA_INDEX*2,  BD_ADDR_LEN);
                    }
                    else
                    {
                        memset(&(event->loc_rslv_priv_addr), 0, sizeof(struct bd_addr));
                    }
                }
                else
                {
                    memset(&(event->loc_rslv_priv_addr), 0, sizeof(struct bd_addr));
                    memset(&(event->peer_rslv_priv_addr), 0, sizeof(struct bd_addr));
                }

                // gets peer address type
                event->peer_addr_type = param->peer_addr_type;
                // gets peer address
                memcpy(&event->peer_addr.addr[0],&param->peer_addr.addr[0],BD_ADDR_LEN);

                // gets connection interval
                event->con_interval = common_htobs(param->con_int);
                // gets connection latency
                event->con_latency = common_htobs(param->con_lat);
                // gets supervision time out
                event->sup_to = common_htobs(param->sup_to);
                // gets clock accuracy
                if(event->role == ROLE_MASTER)
                {
                    event->clk_accuracy = 0;
                }
                else
                {
                    event->clk_accuracy = param->sleep_clk_acc;
                }
            }
            else if (status == COMMON_ERROR_DIRECT_ADV_TO)
            {
                // gets the role
                event->role = ROLE_SLAVE;
                // gets peer address type
                event->peer_addr_type = param->peer_addr_type;
                // gets connection interval
                event->con_interval = 0;
                // gets connection latency
                event->con_latency = 0;
                // gets supervision time out
                event->sup_to = 0;
                event->conhdl = 0;
                memcpy(&event->peer_addr.addr[0],&param->peer_addr.addr[0],BD_ADDR_LEN);

                memset(&(event->peer_rslv_priv_addr), 0, sizeof(struct bd_addr));
                memset(&(event->loc_rslv_priv_addr), 0, sizeof(struct bd_addr));
            }

            // send the message
            hci_send_2_host(event);
        }
    }
}

void llc_con_update_complete_send(uint8_t status, uint16_t conhdl, struct lld_evt_tag *evt)
{	
    // Allocate the status event message
    struct hci_le_con_update_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_con_update_cmp_evt);

    // Gets event subcode
    event->subcode      = HCI_LE_CON_UPDATE_CMP_EVT_SUBCODE;
    // Gets the status
    event->status       = status;
    // Gets connection handle
    event->conhdl       = common_htobs(conhdl);

    if(evt != NULL)
    {
        // Gets connection interval
        event->con_interval = common_htobs(evt->interval >> 1);
        // Gets connection latency
        event->con_latency  = common_htobs(evt->evt.conn.latency - 1);
        // Gets link supervision timeout
        event->sup_to       = common_htobs(llc_env[conhdl]->sup_to);
    }
    else
    {
        struct llc_con_upd_req_ind * param =  llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);

        if(param != NULL)
        {
            event->con_interval = param->interval_min;
            event->con_latency  = param->con_latency;
            event->sup_to       = common_htobs(param->superv_to);
        }
        else
        {
            event->con_interval = 0;
            event->con_latency  = 0;
            // Gets link supervision timeout
            event->sup_to       = 0;
        }
    }

    // send the message
    hci_send_2_host(event);
}

void llc_ltk_req_send(uint16_t conhdl, struct llcp_enc_req const *param)
{
    // allocate the status event message
    struct hci_le_ltk_request_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_ltk_request_evt);
    // gets event subcode
    event->subcode = HCI_LE_LTK_REQUEST_EVT_SUBCODE;
    // gets connection handle
    event->conhdl = common_htobs(conhdl);
    // gets encrypted diversifier
    event->ediv = common_read16p(param->ediv);
    // gets random number
    memcpy(&event->rand.nb[0], &param->rand.nb[0], RAND_NB_LEN);
    // send the message
    hci_send_2_host(event);
}

void llc_feats_rd_event_send(uint8_t status,
                             uint16_t conhdl,
                             struct le_features const *feats)
{
    // checks if the event is not filtered
    if (llm_util_check_evt_mask(LE_CON_RD_REM_FEAT_EVT_BIT))
    {
        // sends the complete meta event
        // allocate the status event message
        struct hci_le_rd_rem_used_feats_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_rd_rem_used_feats_cmd_cmp_evt);
        // gets event subcode
        event->subcode = HCI_LE_RD_REM_USED_FEATS_CMP_EVT_SUBCODE;
        // gets the status
        event->status = status;
        // gets connection handle
        event->conhdl = conhdl;
        // set the features to transmit
        memcpy(&event->feats_used.feats[0], &feats->feats[0], LE_FEATS_LEN);
        // send the message
        hci_send_2_host(event);
    }
}

/*
 * End of the META events
 */

void llc_version_rd_event_send(uint8_t status, uint16_t conhdl)
{
    // allocate the status event message
    struct hci_rd_rem_ver_info_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_VER_INFO_CMP_EVT_CODE, hci_rd_rem_ver_info_cmp_evt);

    // gets the status
    event->status = status;
    // gets connection handle
    event->conhdl = conhdl;
    // check if version is valid or not
    event->compid = common_btohs(llc_env[conhdl]->peer_version.compid);
    // gets the sub-version
    event->subvers = common_btohs(llc_env[conhdl]->peer_version.subvers);
    // gets the version
    event->vers = llc_env[conhdl]->peer_version.vers;
    // send the message
    hci_send_2_host(event);
}

void llc_common_cmd_complete_send(uint16_t opcode, uint8_t status, uint16_t conhdl)
{
    // allocate the complete event message
    struct hci_basic_conhdl_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // update the status
    event->status = status;
    event->conhdl = common_htobs(conhdl);
    // send the message
    hci_send_2_host(event);
}

void llc_common_cmd_status_send(uint16_t opcode, uint8_t status, uint16_t conhdl)
{
    // allocate the status event message
    struct hci_cmd_stat_event *event = KERNEL_MSG_ALLOC(HCI_CMD_STAT_EVENT, conhdl, opcode, hci_cmd_stat_event);
    // update the status
    event->status = status;
    // send the message
    hci_send_2_host(event);
}

void llc_common_flush_occurred_send(uint16_t conhdl)
{
    // allocates the message to send
    struct hci_flush_occurred_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_FLUSH_OCCURRED_EVT_CODE, hci_flush_occurred_evt);
    event->conhdl= conhdl;
    hci_send_2_host(event);
}

void llc_common_enc_key_ref_comp_evt_send(uint16_t conhdl, uint8_t status)
{
    // allocates the message to send
    struct hci_enc_key_ref_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_KEY_REFRESH_CMP_EVT_CODE, hci_enc_key_ref_cmp_evt);
    event->conhdl= conhdl;
    event->status= status;
    hci_send_2_host(event);
}

void llc_common_enc_change_evt_send(uint16_t conhdl, uint8_t enc_status, uint8_t status)
{
    #if !(BLE_QUALIF)
    kernel_task_id_t llc_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    // Check if the authenticated payload timeout needs to be started/stopped
    if (enc_status)
    {
        kernel_timer_set(LLC_AUTH_PAYL_NEARLY_TO, llc_id, llc_env[conhdl]->auth_payl_to - llc_env[conhdl]->auth_payl_to_margin);
        kernel_timer_set(LLC_AUTH_PAYL_REAL_TO, llc_id, llc_env[conhdl]->auth_payl_to);
    }
    else
    {
        kernel_timer_clear(LLC_AUTH_PAYL_NEARLY_TO, llc_id);
        kernel_timer_clear(LLC_AUTH_PAYL_REAL_TO, llc_id);
    }
    #endif // !(BLE_QUALIF)
    // allocates the message to send
    struct hci_enc_change_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_ENC_CHG_EVT_CODE, hci_enc_change_evt);
    event->conhdl= common_htobs(conhdl);
    event->enc_stat = enc_status;
    event->status = status;

    hci_send_2_host(event);
}

void llc_common_nb_of_pkt_comp_evt_send(uint16_t conhdl, uint8_t nb_of_pkt)
{
    // allocates the message to send
    struct hci_nb_cmp_pkts_evt *event = KERNEL_MSG_ALLOC(HCI_EVENT, conhdl, HCI_NB_CMP_PKTS_EVT_CODE, hci_nb_cmp_pkts_evt);
    // gets the connection handle used
    event->conhdl[0] = common_htobs(conhdl);
    // gets the number of packet sent
    event->nb_comp_pkt[0] = common_htobs(nb_of_pkt);
    // processed handle by handle
    event->nb_of_hdl = 1;
    // send the message
    hci_send_2_host(event);
}

void llc_con_update_ind(uint16_t conhdl, struct ea_elt_tag *elt_new)
{
    /*
     * Check if we have to send the command complete event. It is done only if the update
     * was requested by the host or if any of the connection parameters has changed
     */
    if (GETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_HOST_REQ) || GETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_EVT_SENT))
    {
        SETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_HOST_REQ, false);
        SETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_EVT_SENT, false);

        // Checks if the event is not filtered
        if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT))
        {
            // Send the command complete event
            llc_con_update_complete_send(COMMON_ERROR_NO_ERROR, conhdl, LLD_EVT_ENV_ADDR_GET(elt_new));
        }
    }

    // If channel assessment is compiled take care of the latency
    #if (BLE_CHNL_ASSESS)
    //Set if latency enable
    if(LLD_EVT_ENV_ADDR_GET(elt_new)->evt.conn.latency > 1)
    {
        llc_env[conhdl]->chnl_assess.latency_en = true;
    }
    else
    {
        llc_env[conhdl]->chnl_assess.latency_en = false;
    }
    #endif // (BLE_CHNL_ASSESS)
}

void llc_lsto_con_update(uint16_t conhdl)
{
    // Get LLC task environment
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    // Get new Supervision Timeout value
    uint16_t to = common_max(llc_env_ptr->sup_to, llc_env_ptr->n_sup_to) << 1;
    // update supervision timeout with a temporary value
    llc_env_ptr->sup_to = to;

    // Force restart the link supervision timeout
    kernel_timer_set(LLC_LE_LINK_SUP_TO, KERNEL_BUILD_ID(TASK_LLC, conhdl), to); 

    // Indicate that the update is pending, so that supervision timeout is not reset
    SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_PENDING, true);
}


void llc_map_update_ind(uint16_t conhdl)
{
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint8_t nb_good_chnl = llm_util_check_map_validity(&llc_env_ptr->n_ch_map.map[0], LE_CHNL_MAP_LEN);

    ASSERT_ERR(nb_good_chnl > 1);

    ble_chmap0_set(conhdl,common_read16p(&llc_env_ptr->n_ch_map.map[0]));
    ble_chmap1_set(conhdl,common_read16p(&llc_env_ptr->n_ch_map.map[2]));
    ble_chmap2_set(conhdl, (uint16_t)(nb_good_chnl << 8) | llc_env_ptr->n_ch_map.map[4]);

    // Save the new channel map in the current one
    llc_env_ptr->ch_map = llc_env_ptr->n_ch_map;
}



void llc_con_update_finished(uint16_t conhdl)
{
    // Connection Parameter Update procedure is now finished,
    kernel_task_id_t task_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    uint8_t state = kernel_state_get(task_id);

    // check if state is Free or in disconnected state
    if(!llc_state_chk(state, LLC_DISC_BUSY))
    {
        if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
        {
            ASSERT_INFO(llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_CON_UPD_INSTANT, (conhdl << 8) | state, llc_env[conhdl]->loc_proc_state);

            llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);

            llc_state_update(task_id, &state, LLC_LOC_PROC_BUSY, false);
            llc_env[conhdl]->loc_proc_state = LLC_LOC_IDLE;
        }
        else
        {
            if(GETF(llc_env[conhdl]->llc_status, LLC_STAT_LLCP_INSTANT_EXTRACTED))
            {
                ASSERT_INFO(llc_env[conhdl]->rem_proc_state == LLC_REM_WAIT_CON_UPD_INSTANT, (conhdl << 8) | state, llc_env[conhdl]->rem_proc_state);

                llc_state_update(task_id, &state, LLC_REM_PROC_BUSY, false);
                llc_env[conhdl]->rem_proc_state = LLC_REM_IDLE;
                SETF(llc_env[conhdl]->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 0);
            }
            else
            {
                SETF(llc_env[conhdl]->llc_status,  LLC_STAT_INSTANT_PROCEED, 1);
            }
        }
    }

}

void llc_map_update_finished(uint16_t conhdl)
{
    // Channel map Update procedure is now finished,
    kernel_task_id_t task_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    uint8_t state = kernel_state_get(task_id);

    // check if state is Free or in disconnected state
    if(!llc_state_chk(state, LLC_DISC_BUSY))
    {
        if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
        {
            ASSERT_INFO(llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_MAP_UPD_INSTANT, (conhdl << 8) | state, llc_env[conhdl]->loc_proc_state);

            llc_state_update(task_id, &state, LLC_LOC_PROC_BUSY, false);
            llc_env[conhdl]->loc_proc_state = LLC_LOC_IDLE;
        }
        else
        {
            if(GETF(llc_env[conhdl]->llc_status, LLC_STAT_LLCP_INSTANT_EXTRACTED))
            {
                ASSERT_INFO(llc_env[conhdl]->rem_proc_state == LLC_REM_WAIT_MAP_UPD_INSTANT, (conhdl << 8) | state, llc_env[conhdl]->rem_proc_state);

                llc_state_update(task_id, &state, LLC_REM_PROC_BUSY, false);
                llc_env[conhdl]->rem_proc_state = LLC_REM_IDLE;
                SETF(llc_env[conhdl]->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 0);
            }
            else
            {
                SETF(llc_env[conhdl]->llc_status,  LLC_STAT_INSTANT_PROCEED, 1);
            }
        }
    }
}

#if (BLE_2MBPS)
void llc_phy_update_complete_send(uint8_t status, uint16_t conhdl , uint8_t tx_phy, uint8_t rx_phy)
{
    // Allocate the status event message
    struct hci_le_phy_update_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_phy_update_cmp_evt);

    // Gets event subcode
    event->subcode  = HCI_LE_PHY_UPD_CMP_EVT_SUBCODE;
    // Gets the status
    event->status   = status;
    // Gets connection handle
    event->conhdl   = common_htobs(conhdl);
    // Gets TX PHY
    event->tx_phy   = tx_phy;
    // Gets RX PHY
    event->rx_phy   = rx_phy;

    // send the message
    hci_send_2_host(event);
}

void llc_phy_update_finished(uint16_t conhdl , uint8_t tx_phy, uint8_t rx_phy, uint8_t status, uint8_t operation)
{
    // Channel map Update procedure is now finished,
    kernel_task_id_t task_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    uint8_t state = kernel_state_get(task_id);
    bool send_evt = false;
    // check if state is Free or in disconnected state
    if(!llc_state_chk(state, LLC_DISC_BUSY))
    {
        if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                && ((llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_PHY_UPD_INSTANT)
                        || (llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_PHY_UPD_REQ)
                        || (llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_PHY_RSP)
                        || (llc_env[conhdl]->loc_proc_state == LLC_LOC_PHY_NO_INSTANT)))
        {
            llc_state_update(task_id, &state, LLC_LOC_PROC_BUSY, false);
            // If the Local automatic procedure is started and there is a collision or no change, do not send event
            if((operation == LLC_PHY_UPD_HCI_REQ) || (llc_env[conhdl]->loc_proc_state == LLC_LOC_WAIT_PHY_UPD_INSTANT))
            {
                send_evt = true;
            }
            llc_env[conhdl]->loc_proc_state = LLC_LOC_IDLE;
        }
        else if(llc_state_chk(state, LLC_REM_PROC_BUSY)
                && ((llc_env[conhdl]->rem_proc_state == LLC_REM_WAIT_PHY_UPD_REQ)
                        || (llc_env[conhdl]->rem_proc_state == LLC_REM_WAIT_PHY_UPD_INSTANT)
                        || (llc_env[conhdl]->rem_proc_state == LLC_REM_PHY_NO_INSTANT)))
        {
            llc_state_update(task_id, &state, LLC_REM_PROC_BUSY, false);
            if(!(llc_env[conhdl]->rem_proc_state == LLC_REM_PHY_NO_INSTANT))
            {
                send_evt = true;
            }
            llc_env[conhdl]->rem_proc_state = LLC_REM_IDLE;
        }
        else
        {
            ASSERT_INFO(0, llc_env[conhdl]->loc_proc_state, llc_env[conhdl]->rem_proc_state);
        }

        if(send_evt)
        {
            llc_phy_update_complete_send(status, conhdl, tx_phy, rx_phy);
        }
    }
}
#endif // (BLE_2MBPS)
#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLC
