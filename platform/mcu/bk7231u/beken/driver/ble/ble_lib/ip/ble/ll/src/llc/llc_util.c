/**
 ****************************************************************************************
 *
 * @file llc_util.c
 *
 * @brief Link layer controller utilities declaration
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCUTIL
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_PERIPHERAL || BLE_CENTRAL)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "kernel_timer.h"
#include "common_bt.h"
#include "common_error.h"
#include "reg_ble_em_cs.h"
#include "llcontrl.h"
#include "lld.h"
#include "lld_util.h"
#include "llc_util.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


static void llc_check_trafic_paused(uint8_t conhdl)
{
    // Check if traffic expect to be paused
    if(GETF(llc_env[conhdl]->llc_status, LLC_STAT_WAIT_TRAFFIC_PAUSED))
    {
        uint8_t tx_pkt_cnt;

        // Suppress the packet(s) sent
        GLOBAL_INT_DIS();
        tx_pkt_cnt = lld_util_get_tx_pkt_cnt(llc_env[conhdl]->elt);
        GLOBAL_INT_RES();


        // traffic is paused
        if(tx_pkt_cnt == 0)
        {
            // builds the message id associated to the conhdl
            kernel_task_id_t task_id      = KERNEL_BUILD_ID(TASK_LLC, conhdl);
            SETF(llc_env[conhdl]->llc_status, LLC_STAT_WAIT_TRAFFIC_PAUSED, false);
            // device ready to continue starting encryption
            kernel_msg_send_basic(LLC_ENC_MGT_IND,task_id,task_id);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Allocates the handle for the new connection
 *
 ****************************************************************************************
 */
uint8_t llc_util_get_free_conhdl(uint16_t *conhdl)
{
	uint16_t idx = 0;
    for ( idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        // Build a task ID dedicated to the conhdl
        kernel_task_id_t llc_free = KERNEL_BUILD_ID(TASK_LLC, idx);

        // Check if task instance already exists
        if (kernel_state_get(llc_free) == LLC_FREE)
        {
            // Return the connection handle
            *conhdl = idx;

            return (COMMON_ERROR_NO_ERROR);
        }
    }

    return (COMMON_ERROR_CON_LIMIT_EXCEED);
}

/**
 ****************************************************************************************
 * @brief Allocates the handle for the new connection
 *
 ****************************************************************************************
 */
uint8_t llc_util_get_nb_active_link(void)
{
    uint8_t nb_active_link = 0;
    uint16_t idx = 0;
    for ( idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        // Build a task ID dedicated to the conhdl
        kernel_task_id_t llc_free = KERNEL_BUILD_ID(TASK_LLC, idx);

        // Check if task instance already exists
        if (kernel_state_get(llc_free) != LLC_FREE)
        {
            nb_active_link++;
        }
    }

    return (nb_active_link);
}

/**
 ****************************************************************************************
 * @brief Stops the data flow and reset the status before disconnecting
 *
 *
 ****************************************************************************************
 */
void llc_util_dicon_procedure(uint16_t conhdl, uint8_t reason)
{
    // Save the reason
    llc_env[conhdl]->disc_reason    = reason;

    // Set the state to stopping
    kernel_state_set(KERNEL_BUILD_ID(TASK_LLC, conhdl), LLC_DISC_BUSY);
    #if (BLE_CHNL_ASSESS)
    // Stop timer
    kernel_timer_clear(LLC_CHNL_ASSESS_TO, KERNEL_BUILD_ID(TASK_LLC, conhdl));
    #endif

    GLOBAL_INT_DIS();
    // Connection is considered lost, the Link Layer not sends any further packets
    lld_con_stop(llc_env[conhdl]->elt);
    GLOBAL_INT_RES();
}

/**
 ****************************************************************************************
 * @brief Updates channel mapping
 *
 ****************************************************************************************
 */
void llc_util_update_channel_map(uint16_t conhdl, struct le_chnl_map *map)
{
    // Copy channel map to link environment
    memcpy(&llc_env[conhdl]->ch_map.map[0], &map->map[0], LE_CHNL_MAP_LEN);
}

/**
 ****************************************************************************************
 * @brief Sets LLCP discard var to enable
 *
 ****************************************************************************************
 */
void llc_util_set_llcp_discard_enable(uint16_t conhdl, bool enable)
{
    if (enable)
    {
        SETF(llc_env[conhdl]->llc_status, LLC_STAT_LLCP_DISCARD, true);
    }
    else
    {
        SETF(llc_env[conhdl]->llc_status, LLC_STAT_LLCP_DISCARD, false);
    }
}

/**
 ****************************************************************************************
 * @brief Calculates and sets an appropriate margin for the authenticated payload timeout
 *
 ****************************************************************************************
 */
void llc_util_set_auth_payl_to_margin(struct lld_evt_tag *evt)
{
    uint32_t margin, step;

    /*
     * Step is expressed in units of 625 us (like the interval)
     * llc_env[conhdl]->evt->latency is lm latency plus 1
     */
    step = (evt->interval * evt->evt.conn.latency);
    // Allow 8 steps for the margin initially
    margin = (step << 3);

    // the authenticated payload timeout is expressed in units of 10 ms, so multiply it by 16 to get units of 625 us
    while (margin > (llc_env[evt->conhdl]->auth_payl_to << 4))
        margin -= step;

    // now convert to units of 10 ms by dividing by 16
    margin >>= 4;
    if (margin <= 0)
        margin = 1;

    llc_env[evt->conhdl]->auth_payl_to_margin = (uint16_t) margin;
}


void llc_util_clear_operation_ptr(uint16_t conhdl, uint8_t op_type)
{
    void *ptr =  llc_util_get_operation_ptr(conhdl, op_type);
    if(ptr != NULL)
    {
        // Clear Operation
        llc_util_set_operation_ptr(conhdl, op_type, NULL);
        // Free the operation message
        kernel_msg_free(kernel_param2msg(ptr));
    }
}

void llc_util_bw_mgt(uint16_t conhdl)
{
    if (llc_env[conhdl] != NULL)
    {
        // Get associated element
        struct ea_elt_tag *elt_connect = llc_env[conhdl]->elt;
        // Get associated event
        struct lld_evt_tag *evt_connect = LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt);
        // Add the max rx time
        uint16_t bw_needed = llc_env[conhdl]->data_len_ext_info.conn_eff_max_rx_time;
        // Add the max tx time
        bw_needed += llc_env[conhdl]->data_len_ext_info.conn_eff_max_tx_time;
        // Add the IF between rx/tx (peripheral) or tx/rx (central)
        bw_needed += LLD_EVT_IFS_DURATION;

        if(bw_needed > (uint16_t)(evt_connect->interval_elt->bandwidth_used * SLOT_SIZE))
        {
            evt_connect->interval_elt->bandwidth_used = ((bw_needed + (SLOT_SIZE-1)) / SLOT_SIZE);

            // on master side, save needed bandwidth in duration min
            // this is recalculated each event on slave side
            if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
            {
                elt_connect->duration_min = bw_needed;
            }

            if(llc_util_get_nb_active_link() != 1)
            {
                lld_util_anchor_point_move(elt_connect);
            }
        }
    }
}


void llc_end_evt_defer(uint16_t conhdl)
{
    ASSERT_INFO(conhdl < BLE_CONNECTION_MAX, conhdl, 0);

    if (llc_env[conhdl] != NULL)
    {
        // builds the message id associated to the conhdl
        kernel_task_id_t task_id      = KERNEL_BUILD_ID(TASK_LLC, conhdl);

        /*
         *************************************************************************************
         *   DISCONNECTION MANAGEMENT                                                        *
         *************************************************************************************
         */

        // check if state is Free or in disconnected state
        if(llc_state_chk(kernel_state_get(task_id), LLC_DISC_BUSY) && GETF(llc_env[conhdl]->llc_status, LLC_STAT_DISC_REM_REQ))
        {
            SETF(llc_env[conhdl]->llc_status, LLC_STAT_DISC_REM_REQ, false);
            llc_util_dicon_procedure(conhdl, llc_env[conhdl]->disc_reason);
        }

        /******** BW ALLOCATION MANAGEMENT ********/
        llc_util_bw_mgt(conhdl);
    }
}


void llc_pdu_llcp_tx_ack_defer(uint16_t conhdl, uint8_t opcode)
{
    ASSERT_INFO(conhdl < BLE_CONNECTION_MAX, conhdl, opcode);

    uint16_t task_id = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    uint8_t state = kernel_state_get(task_id);
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Link is free, do nothing
    if(llc_state_chk(state, LLC_FREE))
    {
        // nothing to do
    }
    else
    {
        // check if disconnect is on-going
        if(llc_state_chk(kernel_state_get(task_id), LLC_DISC_BUSY))
        {
            // check if ACK is for terminaison opcode
            if (!GETF(llc_env[conhdl]->llc_status, LLC_STAT_DISC_REM_REQ) && (opcode == LLCP_TERMINATE_IND_OPCODE))
            {
                llc_util_dicon_procedure(conhdl, llc_env[conhdl]->disc_reason);
            }
            // else ignore ACK
        }

        else if(llc_state_chk(state, LLC_REM_PROC_BUSY)
                && (   ((llc_env_ptr->rem_proc_state == LLC_REM_WAIT_START_ENC_RSP_ACK) && (opcode == LLCP_START_ENC_RSP_OPCODE))
                    || ((llc_env_ptr->rem_proc_state == LLC_REM_WAIT_ENC_REJECT_ACK) && (opcode == LLCP_REJECT_IND_OPCODE)))
                )
        {
            llc_state_update(task_id, &state, LLC_REM_PROC_BUSY, false);
            llc_state_update(task_id, &state, LLC_TRAFFIC_PAUSED_BUSY, false);
            llc_env_ptr->rem_proc_state = LLC_REM_IDLE;
            GLOBAL_INT_DIS();
            // TX flow can now be restarted
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_TX_FLOW_OFF | LLC_ENC_PAUSE_PENDING, false);
            GLOBAL_INT_RES();
        }
        else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                && ((llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_PAUSE_ENC_RSP_SENT) && (opcode == LLCP_PAUSE_ENC_RSP_OPCODE)))
        {
            // Request to continue the encryption
            kernel_msg_send_basic(LLC_ENC_MGT_IND,task_id, task_id);
        }

        llc_check_trafic_paused(conhdl);
    }
}


void llc_pdu_acl_tx_ack_defer(uint16_t conhdl, uint8_t tx_cnt)
{
    ASSERT_INFO(conhdl < BLE_CONNECTION_MAX, conhdl, tx_cnt);
    if (llc_env[conhdl] != NULL)
    {
        /*
         *************************************************************************************
         *   TX PACKET ACKNOWLEDGEMENT MANAGEMENT                                            *
         *************************************************************************************
         */
        // Send the number of completed data packets to the host if required
        if (tx_cnt)
        {
            llc_common_nb_of_pkt_comp_evt_send(conhdl, tx_cnt);
        }

        llc_check_trafic_paused(conhdl);
    }
}

void llc_pdu_defer(uint16_t conhdl, uint16_t status, uint8_t rssi, uint8_t channel, uint8_t length)
{
    ASSERT_INFO(conhdl < BLE_CONNECTION_MAX, conhdl, status);
    if (llc_env[conhdl] != NULL)
    {
        #if !(BLE_QUALIF)
        // builds the message id associated to the conhdl
        kernel_task_id_t task_id      = KERNEL_BUILD_ID(TASK_LLC, conhdl);
        #endif // #if !(BLE_QUALIF)
        /******** RX RSSI PACKET MANAGEMENT ********/
        llc_env[conhdl]->rssi = rwip_rf.rssi_convert(rssi);

        /************ CHANNEL ASSESSMENT ***********/
        #if (BLE_CHNL_ASSESS)
        llc_ch_assess_local(conhdl, status, llc_env[conhdl]->rssi, channel);
        #endif //(BLE_CHNL_ASSESS)

        /************** ERROR CHECKING ****************/
        if (status & (BLE_MIC_ERR_BIT | BLE_CRC_ERR_BIT | BLE_LEN_ERR_BIT | BLE_SYNC_ERR_BIT))
        {
            if(status & BLE_MIC_ERR_BIT)
            {
                // MIC error, transition to the standby state immediately
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
            }
        }
        else
        {
            // Restart the link supervision timeout
            kernel_timer_set(LLC_LE_LINK_SUP_TO, KERNEL_BUILD_ID(TASK_LLC, conhdl), (llc_env[conhdl]->sup_to));

            /******** DUPLICATE PACKET FILTERING or RXTIME ERROR ********/
            if ((status & BLE_SN_ERR_BIT) || (status & BLE_RXTIMEERR_BIT) )
            {
                // nothing to do
            }
            /******** RX EMPTY PACKET MANAGEMENT ********/
            else if(length == 0)
            {
                // nothing to do
            }
            #if !(BLE_QUALIF)
            // MIC was ok, reset the authenticated payload timeout if encryption is enabled
            else if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_ENABLE) && (!GETF(llc_env[conhdl]->llc_status, LLC_STAT_LLCP_DISCARD)))
            {
                kernel_timer_set(LLC_AUTH_PAYL_NEARLY_TO, task_id, llc_env[conhdl]->auth_payl_to - llc_env[conhdl]->auth_payl_to_margin);
                kernel_timer_set(LLC_AUTH_PAYL_REAL_TO, task_id, llc_env[conhdl]->auth_payl_to);
            }
            #endif // !(BLE_QUALIF)
        }

        // check that timeout has not been updated by the driver due to an update
        if (GETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_PENDING))
        {
            SETF(llc_env[conhdl]->llc_status, LLC_STAT_UPDATE_PENDING, false);
            // Update the supervision timeout with the new one
            llc_env[conhdl]->sup_to = llc_env[conhdl]->n_sup_to;
        }
    }
}

#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCUTIL
