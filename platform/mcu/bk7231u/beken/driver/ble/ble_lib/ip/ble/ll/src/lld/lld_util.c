/**
 ****************************************************************************************
 *
 * @file lld_util.c
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
 * @addtogroup LLDUTIL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include "rwip_config.h"
#include "rwble.h"

#include "lld_util.h"

#include "common_endian.h"
#include "common_utils.h"
#include "kernel_mem.h"
#include "lld_evt.h"
#include "lld.h"
#include "llcontrl.h"
#include "llc_util.h"
#include "llm_util.h"
#include "reg_blecore.h"
#include "reg_ble_em_cs.h"
#include "reg_ble_em_ral.h"
/*
 * DEFINES
 ****************************************************************************************
 */
// Size in us of the empty packet (8bits(preamble)+32bits(access code)+16bits(header)+24bits(CRC))
#define EMPTY_PKT_SIZE  (80)
// Size in us of the MIC (4 bytes)
#define MIC_SIZE  (32)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t lld_util_instant_get(void *event, uint8_t action)
{
    struct lld_evt_tag * evt = (struct lld_evt_tag *)event;
    // The instant will be 6 wake-up times after the next event
    uint16_t count_to_inst  = evt->evt.conn.latency * LLD_UTIL_MIN_INSTANT_CON_EVT;
    // Update instant
    uint16_t update_instant = (evt->evt.conn.counter + count_to_inst) & RWBLE_INSTANT_MASK;
    // Program the instant in the event
    evt->evt.conn.instant        = update_instant;
    // Indicate the action that will be performed at instant
    evt->evt.conn.instant_action = action;

    // Return the instant to be put in the frame
    return (update_instant);
}

void lld_util_get_bd_address(struct bd_addr *bd_addr)
{
    // Get the BD address
    common_write32p (&bd_addr->addr[0], common_btohl(ble_bdaddrl_get()));
    common_write16p (&bd_addr->addr[4], common_btohs(ble_bdaddru_get()));
}

void lld_util_set_bd_address(struct bd_addr *bd_addr, uint8_t type)
{
    // Set the BD address
    ble_bdaddrl_set(common_htobl(common_read32p(&bd_addr->addr[0])));
    ble_bdaddru_setf(common_htobs(common_read16p(&bd_addr->addr[4])));

    // Set address type
    ble_priv_npub_setf(type);
}

void lld_util_ral_force_rpa_renew(void)
{
    int16_t cursor;

    // browse the Resolvable Address List
    for(cursor = BLE_RESOL_ADDR_LIST_MAX ; cursor >= 0 ; cursor--)
    {
        // for a valid entry
        if(ble_entry_valid_getf(cursor))
        {
            // force renewal of local and peer RPAs
            if(ble_local_irk_valid_getf(cursor))
            {
                ble_local_rpa_renew_setf(cursor, true);
            }
            if(ble_peer_irk_valid_getf(cursor))
            {
                ble_peer_rpa_renew_setf(cursor, true);
            }
        }
    }
}


uint8_t lld_util_freq2chnl(uint8_t freq)
{
    uint8_t chnl = 0;

    switch (freq)
    {
        case (0):
        {
            freq = 37;
        } break;

        case (12):
        {
            freq = 38;
        } break;

        case (39):
        {
            // Nothing
        } break;

        default:
        {
            if (freq < 12)
            {
                freq -= 1;
            }
            else
            {
                freq -= 2;
            }
        } break;
    }
    chnl = freq;

    return (chnl);
}

uint16_t lld_util_get_local_offset(uint16_t PeerOffset, uint16_t Interval, uint32_t AnchorPoint)
{
    uint16_t temp;

    temp = AnchorPoint % Interval;
    temp = (PeerOffset + temp) % Interval;
    return(temp);
}

uint16_t lld_util_get_peer_offset(uint16_t LocalOffset, uint16_t Interval, uint32_t AnchorPoint)
{
    uint16_t temp;

    temp = AnchorPoint % Interval;
    temp = (LocalOffset + Interval - temp) % Interval;
    return(temp);
}

void lld_util_connection_param_set(struct ea_elt_tag *elt, struct ea_param_output* param)
{
    uint32_t current_offset = 0;
    // Check that requested duration is not too large for event interval
    if (param->duration > (param->interval - elt->start_latency))
    {
        // If too large, then reduce it
        param->duration = param->interval - elt->start_latency;
    }
    if(!param->duration)
    {
        //If the duration is equal to 0 an issue will appear later when the RXwindow is set
        param->duration = EA_BW_USED_DFT_SLOT;
    }

    // Save the duration of the event
    param->duration = param->duration*SLOT_SIZE;

    current_offset = elt->timestamp % param->interval;

    if(param->offset >= current_offset)
    {
        if((current_offset == 0) && (param->offset == 0))
        {
            param->offset = param->interval;
        }
        else
        {
            param->offset = (param->offset - current_offset) & BLE_BASETIMECNT_MASK;
        }
    }
    else
    {
        param->offset = (param->interval  + (param->offset - current_offset)) & BLE_BASETIMECNT_MASK;
    }

}

#if (BLE_PERIPHERAL || BLE_CENTRAL)
void lld_util_dle_set_cs_fields(uint16_t conhdl)
{
    ble_rxmaxbuf_setf(conhdl, llc_env[conhdl]->data_len_ext_info.conn_eff_max_rx_octets);
    ble_rxmaxtime_set(conhdl, llc_env[conhdl]->data_len_ext_info.conn_eff_max_rx_time);
}


void lld_util_anchor_point_move(struct ea_elt_tag *elt_connect)
{
    // Get associated event
    struct lld_evt_tag *evt_connect = LLD_EVT_ENV_ADDR_GET(elt_connect);
    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param = {0,0,0};

    // Compute the scheduling parameters
    input_param.action = EA_PARAM_REQ_GET;
    input_param.conhdl = evt_connect->conhdl;
    input_param.role = (evt_connect->mode == LLD_EVT_MST_MODE)? MASTER_ROLE : SLAVE_ROLE;
    input_param.odd_offset = false;
    input_param.linkid = REG_BLE_EM_CS_ADDR_GET(evt_connect->conhdl);
    set_param.interval = evt_connect->interval;
    set_param.duration = (elt_connect->duration_min + (SLOT_SIZE-1))/SLOT_SIZE;

    // If the offset cannot be updated, keep the previous one
    if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
    {
        // Allocate a LLD_PARAM_REQ_IND message
        struct llc_con_upd_req_ind *msg = KERNEL_MSG_ALLOC(LLC_CON_UPD_REQ_IND, KERNEL_BUILD_ID(TASK_LLC, evt_connect->conhdl), TASK_LLD, llc_con_upd_req_ind);

        msg->operation           = LLC_CON_UP_MOVE_ANCHOR;
        msg->con_intv_min        = set_param.interval >> 1; // In units of 1.25 ms
        msg->con_intv_max        = set_param.interval >> 1; // In units of 1.25 ms
        msg->interval_min        = set_param.interval >> 1; // In units of 1.25 ms
        msg->interval_max        = set_param.interval >> 1; // In units of 1.25 ms
        msg->con_latency         = evt_connect->evt.conn.latency - 1; // In units of number of connection events
        msg->superv_to           = 3000; // In units of 10 ms
        msg->pref_period         = 0; // In units of 1.25 ms
        msg->ref_con_event_count = evt_connect->evt.conn.counter;
        msg->ce_len_min          = evt_connect->evt.conn.counter;
        msg->ce_len_max          = evt_connect->evt.conn.counter;
        msg->offset0             = lld_util_get_peer_offset(set_param.offset, set_param.interval, elt_connect->timestamp) >> 1; // In units of 1.25 ms
        msg->offset1             = 0xFFFF;
        msg->offset2             = 0xFFFF;
        msg->offset3             = 0xFFFF;
        msg->offset4             = 0xFFFF;
        msg->offset5             = 0xFFFF;

        // Send the message
        kernel_msg_send(msg);
    }
}

#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
void lld_util_flush_list(struct common_list *list)
{
    while (1)
    {
        // Pop the first element from the list
        void *elt = (void  *)common_list_pop_front(list);
        // If we reach the end of the list, then we exit the loop
        if (elt == NULL)
        {
            break;
        }

        // Free the element
        kernel_free(elt);
    }
}
#if (BLE_PERIPHERAL || BLE_CENTRAL)
bool lld_util_instant_ongoing(struct ea_elt_tag *elt)
{
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    return( (evt->evt.conn.instant_action != LLD_UTIL_NO_ACTION)?true:false);
}

void lld_util_compute_ce_max(struct ea_elt_tag *elt, uint16_t tx_time, uint16_t rx_time)
{
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    uint32_t frame_size = 0;
    uint32_t nb_frame_by_interval = 0;
    uint32_t ce_max = 0;
    //
    uint32_t max_bw = (evt->interval - elt->start_latency) * SLOT_SIZE - SLOT_SIZE ;
    uint16_t encryption = ble_link_get(evt->conhdl);
    uint16_t win_size = 0;

    //Check if we are slave or master to take in account the sync window
    if(evt->mode == LLD_EVT_MST_MODE)
    {
        win_size = ble_rxwinszdef_getf();
    }
    else
    {
        win_size = ble_rxwincntl_get(evt->conhdl);
    }
    //Convert the sync window in us if needed
    if(win_size & BLE_RXWIDE_BIT)
    {
        // Multiple of 625us
        max_bw -=  (((win_size & BLE_RXWINSZ_MASK) >> 1) * SLOT_SIZE);
    }
    else
    {
        //multiple of 1us
        max_bw -=  (win_size & BLE_RXWINSZ_MASK) >> 1;
    }
    //Check if the encryption is started
    if(encryption & BLE_TXCRYPT_EN_BIT)
    {
        frame_size = tx_time + 2 * LLD_EVT_IFS_DURATION + EMPTY_PKT_SIZE;
    }
    else
    {
        frame_size =  (tx_time - MIC_SIZE) + 2 * LLD_EVT_IFS_DURATION + EMPTY_PKT_SIZE;
    }
    //Compute the number of frame available in one connection interval
    nb_frame_by_interval = (max_bw / frame_size);
    //Like the ce_max lets the end of current frame, remove half of the latest frame to be sure that no more frame
    //will be started and convert in slot (ce_max field is in slot)
    ce_max = (nb_frame_by_interval * frame_size - (frame_size/2)) / SLOT_SIZE;

    if( (int32_t)((ce_max * frame_size)-(ce_max * SLOT_SIZE)) > LLD_EVT_IFS_DURATION)
    {
        ce_max-= 1;
    }
    //Set the ce_max event
    ble_maxevtime_set(evt->conhdl, ce_max);
    //Check if the new CE max is smaller than the CE min
    if(ble_minevtime_get(evt->conhdl)>ce_max)
    {
        ble_minevtime_set(evt->conhdl, ce_max);
    }


}
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

bool lld_util_elt_programmed(struct ea_elt_tag *elt)
{
    bool status = false;

    uint32_t current_time = ea_time_get_halfslot_rounded();
    uint8_t clk_diff = CLK_DIFF(current_time, elt->timestamp);
    if(clk_diff >= elt->start_latency)
    {
        //Element already programmed
        status = true;
    }
    return (status);
}

void lld_util_priority_set(struct ea_elt_tag *elt, uint8_t priority_index)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Get type of link
    switch (priority_index)
    {
        #if (BLE_BROADCASTER || BLE_PERIPHERAL)
        case RWIP_PRIO_ADV_IDX:
        {
            if(llm_util_get_adv_type() == LLD_LD_ADVERTISER)
            {
               evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_ADV_IDX].value;
            }
            else
            {
                evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_ADV_HDC_IDX].value;
            }
        } break;
        #endif
        case RWIP_PRIO_ADV_HDC_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_ADV_HDC_IDX].value;
        } break;

        case RWIP_PRIO_SCAN_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_SCAN_IDX].value;
        } break;

        case RWIP_PRIO_LE_CON_IDLE_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value;
        } break;

        case RWIP_PRIO_LE_CON_ACT_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_LE_CON_ACT_IDX].value;
        } break;

        case RWIP_PRIO_INIT_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_INIT_IDX].value;
        } break;

         case RWIP_PRIO_LE_ESTAB_IDX:
        {
            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_LE_ESTAB_IDX].value;
        } break;

        default:
        {
            // Should not happen, use lowest priority
            ASSERT_ERR(0);

            evt->default_prio = elt->current_prio = rwip_priority[RWIP_PRIO_ADV_IDX].value;
        } break;
    }
}

void lld_util_priority_update(struct ea_elt_tag *elt , uint8_t value)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    evt->default_prio = value;
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t lld_util_get_tx_pkt_cnt(struct ea_elt_tag *elt)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    return evt->evt.conn.tx_prog_pkt_cnt;
}
#if (BLE_2MBPS)
void lld_util_get_phys(struct ea_elt_tag *elt, uint8_t *tx_phy, uint8_t *rx_phy)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Get Current rate used
    *rx_phy = ble_rxrate_getf(evt->conhdl);
    *rx_phy = (*rx_phy == 0)? PHYS_1MBPS_PREF : PHYS_2MBPS_PREF;
    *tx_phy = ble_txrate_getf(evt->conhdl);
    *tx_phy = (*tx_phy == 0)? PHYS_1MBPS_PREF : PHYS_2MBPS_PREF;

}

void lld_util_phy_update_req(struct ea_elt_tag *elt, uint16_t instant, uint8_t tx_phy, uint8_t rx_phy)
{
    // Get associated event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Create an event to handle the connection update
    evt->evt.conn.instant           = common_btohs(instant);
    evt->evt.conn.instant_action    = LLD_UTIL_PHY_UPDATE;
    if(tx_phy)
    {
        evt->evt.conn.tx_phy = tx_phy;
    }
    if(rx_phy)
    {
        evt->evt.conn.rx_phy = rx_phy;
    }

    /**
     * (Instant T0-1) management
     */
    if (evt->evt.conn.counter == ((instant - 1) & 0xFFFF))
    {
        // Reset waiting instant flag
        LLD_EVT_FLAG_SET(evt, WAITING_INSTANT);
    }
    /**
     * (Instant T0)  management
     */
    else if(evt->evt.conn.counter == instant)
    {
        // Apply immediately new PHY
        lld_util_phy_update_ind(elt);

        // Reset waiting instant flag
        LLD_EVT_FLAG_SET(evt, WAITING_INSTANT);
    }
}

void lld_util_phy_update_ind(struct ea_elt_tag *elt)
{
    // Get associated event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    uint8_t tx_phy=0;
    uint8_t rx_phy =0;
    //If 1MBPS
    if(evt->evt.conn.rx_phy == PHYS_2MBPS_PREF)
    {
        evt->evt.conn.rx_win_off_dft = LLD_EVT_RX_WIN_DFT_OFF_2MBPS;
        evt->evt.conn.rx_win_pathdly_comp = LLD_EVT_RX_WIN_PATHDLY_COMP_2MBPS;
    }
    else //2MPBS
    {
        evt->evt.conn.rx_win_off_dft = LLD_EVT_RX_WIN_DFT_OFF_1MBPS;
        evt->evt.conn.rx_win_pathdly_comp = LLD_EVT_RX_WIN_PATHDLY_COMP_1MBPS;
    }
    /**
     * CS-field converted
     * Receive Rate:
     *  00: 1Mbps uncoded PHY
     *  01: 2Mbps uncoded PHY
     *  1x: Reserved
     */
    if(evt->evt.conn.rx_phy != PHYS_NO_PREF)
    {
        rx_phy = (evt->evt.conn.rx_phy == PHYS_2MBPS_PREF)? 0x1 : 0x0;
        // Set the CS-fields rx rate
        ble_rxrate_setf(evt->conhdl, rx_phy);
    }

    if(evt->evt.conn.tx_phy != PHYS_NO_PREF)
    {
        tx_phy = (evt->evt.conn.tx_phy == PHYS_2MBPS_PREF)? 0x1 : 0x0;
        // Set the CS-fields tx rate
        ble_txrate_setf(evt->conhdl, tx_phy);
    }

}
#endif // (BLE_2MBPS)
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

void lld_util_eff_tx_time_set(struct ea_elt_tag *elt, uint16_t max_tx_time, uint16_t max_tx_size)
{
    // Get associated event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    evt->evt.conn.eff_max_tx_time = max_tx_time;
    evt->evt.conn.eff_max_tx_size = max_tx_size;
}

/// @} LLDUTIL
