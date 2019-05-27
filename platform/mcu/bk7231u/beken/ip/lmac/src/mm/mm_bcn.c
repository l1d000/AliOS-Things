/**
 ****************************************************************************************
 *
 * @file mm_bcn.c
 *
 * @brief MAC Beacon management module implementation.
 *
 * Copyright (C) RivieraWaves 2011-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MM_BCN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "mac_defs.h"
#include "mac_frame.h"
#include "mm.h"
#include "mm_timer.h"
#include "co_endian.h"
#include "sta_mgmt.h"
#include "vif_mgmt.h"
#include "phy.h"
#include "rd.h"
#include "ps.h"
#include "txl_cntrl.h"
#include "txl_frame.h"
#include "rxl_cntrl.h"
#include "hal_machw.h"
#include "scan.h"
#include "chan.h"
#include "mm_bcn.h"
#include "hal_dma.h"
#include "tpc.h"
#include "reg_mac_core.h"
#include "reg_mac_pl.h"
#include "me_utils.h"
#include "apm.h"

#include "arm_arch.h"
#include "uart_pub.h"
#include "mem_pub.h"
#include "rw_pub.h"
#include "wlan_ui_pub.h"

#if CFG_ROLE_LAUNCH
#include "role_launch.h"
#endif

#if NX_BCN_AUTONOMOUS_TX

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
///  Global data for maintaining beacon information

/**  LMAC MM BCN Context variable, used to store MM BCN Context data
 */
struct mm_bcn_env_tag mm_bcn_env;
IND_CALLBACK_T csa_event_cb = {0};

static void mm_bcn_update(struct mm_bcn_change_req const *param);

void bcn_tx_cfm_add(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    mm_bcn_env.tx_cfm ++;
    GLOBAL_INT_RESTORE();
}

void bcn_tx_cfm_dec(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    
    mm_bcn_env.tx_cfm --;
    GLOBAL_INT_RESTORE();    
}

void bcn_tx_cfm_set(int32 value)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    
    mm_bcn_env.tx_cfm = value;
    GLOBAL_INT_RESTORE();    
}

UINT32 bcn_tx_cfm_get(void )
{
    int cfm;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    cfm = mm_bcn_env.tx_cfm;
    GLOBAL_INT_RESTORE(); 
    return cfm;
    
}

void mm_csa_event_cb(FUNC_2PARAM_PTR ind_cb, void *ctxt)
{
    csa_event_cb.cb = ind_cb;
    csa_event_cb.ctxt_arg = ctxt;
}

void mm_csa_finish_event(void)
{
    if(csa_event_cb.cb)
    {
        (*csa_event_cb.cb)(csa_event_cb.ctxt_arg, 0);
    }
}

static void mm_tim_update_proceed(struct mm_tim_update_req const *param)
{
    struct vif_info_tag *vif_entry = &vif_info_tab[param->inst_nbr];

    // No beacon transmission ongoing, proceed immediately to the update
    if (param->aid == 0)
    {
        // Update BC/MC status bit
        vif_entry->u.ap.bc_mc_status = param->tx_avail;
    }
    else
    {
        do
        {
            struct tx_pbd *pbd_tim = &txl_tim_desc[param->inst_nbr][0];
            struct tx_pbd *pbd_bmp = &txl_tim_desc[param->inst_nbr][1];
            uint32_t tim_ie = CPU2HW(&txl_tim_ie_pool[param->inst_nbr][0]);
            uint32_t tim_bmp = CPU2HW(&txl_tim_bitmap_pool[param->inst_nbr][0]);

            // Compute the byte and bit numbers for this AID
            uint8_t n = param->aid / 8;
            uint8_t mask = CO_BIT(param->aid % 8);
            uint8_t val = co_read8p(tim_bmp + n);

            // Check if we have to set or reset a bit
            if (param->tx_avail)
            {
                // Check if the bit is already set
                if (val & mask)
                    break;

                // Set the bit in the bitmap
                co_write8p(tim_bmp + n, val | mask);

                // One more bit is set
                vif_entry->u.ap.tim_bitmap_set++;

                // Check if Lower limit of the partial virtual bitmap needs to be updated
                if (n < vif_entry->u.ap.tim_n1)
                {
                    // Lower limit shall be an even number
                    vif_entry->u.ap.tim_n1 = n & 0xFE;

                    // Update the start pointer in the TBD
                    pbd_bmp->datastartptr = tim_bmp + vif_entry->u.ap.tim_n1;
                }

                // Check if Upper limit of the partial virtual bitmap needs to be updated
                if (n > vif_entry->u.ap.tim_n2)
                {
                    // Update Upper limit
                    vif_entry->u.ap.tim_n2 = n;

                    // Update the end pointer in the TBD
                    pbd_bmp->dataendptr = tim_bmp + vif_entry->u.ap.tim_n2;
                }

                // Update the TIM length
                vif_entry->u.ap.tim_len = vif_entry->u.ap.tim_n2 - vif_entry->u.ap.tim_n1 + 6;

                // Update the TIM PBD
                co_write8p(tim_ie + MAC_TIM_LEN_OFT, vif_entry->u.ap.tim_len - 2);
                co_write8p(tim_ie + MAC_TIM_BMPC_OFT, vif_entry->u.ap.tim_n1);
                pbd_tim->dataendptr = tim_ie + MAC_TIM_BMPC_OFT;
                pbd_tim->next = CPU2HW(pbd_bmp);
            }
            else
            {
                // Check if the bit is already reset
                if (!(val & mask))
                    break;

                // Reset the bit in the bitmap
                co_write8p(tim_bmp + n, val & ~mask);

                // One more bit is reset
                vif_entry->u.ap.tim_bitmap_set--;

                // Check if the bitmap has still some bits set
                if (vif_entry->u.ap.tim_bitmap_set)
                {
                    // Check if we need to update the Lower Limit
                    if ((n & 0xFE) == vif_entry->u.ap.tim_n1)
                    {
                        // Search for the new Lower Limit
                        while ((vif_entry->u.ap.tim_n1 != MAC_TIM_SIZE) &&
                               (co_read8p(tim_bmp + vif_entry->u.ap.tim_n1) == 0))
                        {
                            vif_entry->u.ap.tim_n1++;
                        }

                        // Lower Limit shall be an even number
                        vif_entry->u.ap.tim_n1 &= 0xFE;

                        // Update the start pointer in the TBD
                        pbd_bmp->datastartptr = tim_bmp + vif_entry->u.ap.tim_n1;
                    }

                    // Check if we need to update the Upper Limit
                    if (n == vif_entry->u.ap.tim_n2)
                    {
                        // Search for the new Upper Limit
                        while ((vif_entry->u.ap.tim_n2 != 0) &&
                               (co_read8p(tim_bmp + vif_entry->u.ap.tim_n2) == 0))
                        {
                            vif_entry->u.ap.tim_n2--;
                        }

                        // Update the end pointer in the TBD
                        pbd_bmp->dataendptr = tim_bmp + vif_entry->u.ap.tim_n2;
                    }

                    // Update the TIM length
                    vif_entry->u.ap.tim_len = vif_entry->u.ap.tim_n2 - vif_entry->u.ap.tim_n1 + 6;

                    // Update the TIM PBD
                    co_write8p(tim_ie + MAC_TIM_LEN_OFT, vif_entry->u.ap.tim_len - 2);
                    co_write8p(tim_ie + MAC_TIM_BMPC_OFT, vif_entry->u.ap.tim_n1);
                }
                else
                {
                    // Update the TIM
                    vif_entry->u.ap.tim_len = MAC_TIM_BMP_OFT + 1;
                    vif_entry->u.ap.tim_n1 = (uint8_t)-1;
                    vif_entry->u.ap.tim_n2 = 0;
                    co_write8p(tim_ie + MAC_TIM_LEN_OFT, vif_entry->u.ap.tim_len - 2);
                    co_write8p(tim_ie + MAC_TIM_BMPC_OFT, 0);
                    // Update the TIM PBD
                    pbd_tim->dataendptr = tim_ie + MAC_TIM_BMP_OFT;
                    pbd_tim->next = CPU2HW(&txl_bcn_end_desc[param->inst_nbr]);
                    // Update the end pointer in the TBD
                    pbd_bmp->dataendptr = tim_bmp + vif_entry->u.ap.tim_n2;
                }
            }
        } while(0);
    }

    // Confirm the TIM update to the host
    ke_msg_send_basic(MM_TIM_UPDATE_CFM, ke_param2msg(param)->src_id, TASK_MM);

    // Free the TIM parameters
    ke_msg_free(ke_param2msg(param));
}

static void mm_bcn_desc_prep(struct vif_info_tag *vif_entry,
                             struct mm_bcn_change_req const *param)
{
    struct txl_frame_desc_tag *frame = &vif_entry->u.ap.bcn_desc;
    struct tx_hd *thd = &frame->txdesc.lmac.hw_desc->thd;
    struct tx_pbd *pbd_bcn = &txl_bcn_end_desc[vif_entry->index];
    uint32_t tim_bcn = thd->datastartptr + param->tim_oft;
    uint32_t tim_ie = CPU2HW(&txl_tim_ie_pool[vif_entry->index][0]);
    struct tx_policy_tbl *pol;
    uint8_t band;
    uint32_t bcn_len = param->bcn_len - param->tim_len;
    #if !NX_CHNL_CTXT
    struct phy_channel_info info;
    #endif

    // Store the current beacon length for later update of the THD
    vif_entry->u.ap.bcn_len = bcn_len;

    // Verify the protection and bandwidth status
    me_beacon_check(vif_entry->index, param->bcn_len, thd->datastartptr);

    // Fill-up the TX header descriptor
    thd->dataendptr = thd->datastartptr + param->tim_oft - 1;
    pbd_bcn->datastartptr = thd->dataendptr + param->tim_len + 1;
    pbd_bcn->dataendptr = pbd_bcn->datastartptr + bcn_len - param->tim_oft - 1;
    pbd_bcn->bufctrlinfo = 0;

    // Get band
    #if NX_CHNL_CTXT
    band = vif_entry->chan_ctxt->channel.band;
    #else
    phy_get_channel(&info, PHY_PRIM);
    band = info.info1 & 0xFF;
    #endif

    #if (NX_P2P_GO)
    // If the interface is a P2P interface we cannot use default 2.4GhHz policy table (11b rate forbidden)
    if (vif_entry->p2p)
    {
        pol = &txl_buffer_control_5G.policy_tbl;
    }
    else
    #endif //(NX_P2P_GO)
    {
        pol = (band == PHY_BAND_2G4) ? &txl_buffer_control_24G.policy_tbl : &txl_buffer_control_5G.policy_tbl;
    }

    // Set TX power
    pol->powercntrlinfo[0] = nxmac_ofdm_max_pwr_level_getf() << TX_PWR_LEVEL_PT_RCX_OFT;
    thd->policyentryaddr = CPU2HW(pol);
    thd->phyctrlinfo = 0;
    thd->macctrlinfo2 = 0;
    thd->first_pbd_ptr = CPU2HW(&txl_tim_desc[vif_entry->index][0]);

    // The beacon is now configured for this VIF
    vif_entry->u.ap.bcn_configured = true;
    co_write8p(tim_ie + MAC_TIM_PERIOD_OFT, co_read8p(tim_bcn + MAC_TIM_PERIOD_OFT));
}

static void mm_bcn_csa_init(struct vif_info_tag *vif_entry,
                            struct mm_bcn_change_req const *param)
{
    struct txl_frame_desc_tag *frame = &vif_entry->u.ap.bcn_desc;
    struct tx_hd *thd = &frame->txdesc.lmac.hw_desc->thd;
    uint8_t i;

    // reset value
    vif_entry->u.ap.csa_count = 0;
    for (i = 0; i < BCN_MAX_CSA_CPT; i++)
    {
        vif_entry->u.ap.csa_oft[i] = param->csa_oft[i];
    }

    if (param->csa_oft[0] > 0)
    {
        vif_entry->u.ap.csa_count = co_read8p(thd->datastartptr +
                                              param->csa_oft[0]);
        vif_entry->u.ap.csa_count++;
    }
}

void mm_csa_beacon_after_change(void)
{
    struct mm_bcn_change_req *param;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    param = mm_bcn_env.param_csa_after;
    mm_bcn_env.param = param;
    mm_bcn_env.param_csa_after = 0;
    GLOBAL_INT_RESTORE();
	
    os_printf("mm_csa_beacon_after_change\r\n");
    if(param)
    {
        mm_bcn_change(param);
    }
}

void mm_csa_beacon_flush(void)
{
    struct mm_bcn_change_req *param_csa;
    struct mm_bcn_change_req *param;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    param_csa = mm_bcn_env.param_csa_after;
    mm_bcn_env.param_csa_after = 0;

    if(param_csa)
    {
        struct ke_msg *msg = ke_param2msg(param_csa);
        if(param_csa->bcn_ptr_malloc_flag)
        {
            os_free((void *)param_csa->bcn_ptr);
            param_csa->bcn_ptr = 0;
            param_csa->bcn_ptr_malloc_flag = 0;
        }
        ke_msg_free(msg);
    }
    GLOBAL_INT_RESTORE();    
}

void mm_csa_beacon_change(struct mm_bcn_change_req *param,
    struct mm_bcn_change_req *param_csa_after)
{
    GLOBAL_INT_DECLARATION();

    os_printf("mm_csa_beacon_change\r\n");
    
    // Save the message for later handling
    GLOBAL_INT_DISABLE();
    mm_bcn_env.param = param;
    mm_bcn_env.param_csa_after = param_csa_after;
    GLOBAL_INT_RESTORE();

    // Check if the beacon transmission is already ongoing
    if (bcn_tx_cfm_get())
    {        
        mm_bcn_env.update_pending = true;
    }
    else
    {
        // No beacon transmission ongoing, proceed immediately to the update 
        mm_bcn_update(param);
    }
}

void mm_channel_switch_init(struct vif_info_tag *vif_entry, uint32_t freq,
                                    uint32_t csa_count)
{    
    struct mm_chan_ctxt_add_req *csa_channel;

    csa_channel = &vif_entry->csa_channel;
    vif_entry->u.ap.csa_count = csa_count;
    vif_entry->u.ap.csa_action_count = CSA_ACTION_COUNT;
    
    csa_channel->band = PHY_BAND_2G4;
    csa_channel->prim20_freq = freq;
    csa_channel->center1_freq = freq;
    csa_channel->center2_freq = 0;
    csa_channel->tx_power = 0;
    csa_channel->type = PHY_CHNL_BW_20;
}

static void mm_bcn_send_csa_counter_ind(uint8_t vif_index, uint8_t csa_count)
{
    // Allocate the indication structure
    struct mm_csa_counter_ind *ind = KE_MSG_ALLOC(MM_CSA_COUNTER_IND, TASK_API,
                                                  TASK_MM, mm_csa_counter_ind);

    ind->vif_index = vif_index;
    ind->csa_count = csa_count;

    // Send the message
    ke_msg_send(ind);
}

static uint8_t mm_bcn_build(struct vif_info_tag *vif_entry)
{
    struct txl_frame_desc_tag *frame = &vif_entry->u.ap.bcn_desc;
    struct tx_hd *thd = &frame->txdesc.lmac.hw_desc->thd;
    uint32_t tim_ie = CPU2HW(&txl_tim_ie_pool[vif_entry->index][0]);
    uint8_t bmpc = co_read8p(tim_ie + MAC_TIM_BMPC_OFT);

    // Update the THD length
    thd->frmlen = vif_entry->u.ap.bcn_len + vif_entry->u.ap.tim_len + MAC_FCS_LEN;

    #if (NX_P2P_GO)
    if (vif_entry->p2p)
    {
        // Length of NOA Information Element
        uint8_t noa_len = p2p_go_bcn_get_noa_len(vif_entry->p2p_index);

        if (noa_len)
        {
            // Get TX payload buffer descriptor
            struct tx_pbd *p_noa_pbd = &txl_p2p_noa_desc[vif_entry->index];

            // Add length of P2P NOA Information Element if present
            thd->frmlen += noa_len;

            // Update NOA Payload descriptor Data End pointer
            p_noa_pbd->dataendptr = p_noa_pbd->datastartptr + noa_len - 1;
        }
    }
    #endif //(NX_P2P_GO)

    #if (RW_UMESH_EN)
    if (vif_entry->type == VIF_MESH_POINT)
    {
        uint8_t mesh_ies_len = mesh_get_vendor_ies_len(vif_entry->mvif_idx);

        // Update dynamic fields (Number of Peerings, ...)
        mesh_update_beacon(vif_entry);

        // Check if IEs have to be added at the end of the beacon
        if (mesh_ies_len)
        {
            // Get TX payload buffer descriptor
            struct tx_pbd *p_add_ies_pbd = &txl_mesh_add_ies_desc[vif_entry->mvif_idx];

            // Update frame length
            thd->frmlen += mesh_ies_len;

            // Update NOA Payload descriptor Data End pointer
            p_add_ies_pbd->dataendptr = p_add_ies_pbd->datastartptr + mesh_ies_len - 1;

            // Add the additional element at beacon end
            txl_bcn_end_desc[vif_entry->index].next = CPU2HW(p_add_ies_pbd);
        }
        else
        {
            txl_bcn_end_desc[vif_entry->index].next = CPU2HW(NULL);
        }
    }
    #endif //(RW_UMESH_EN)

    // Update the sequence number in the beacon
    co_write16(HW2CPU(thd->datastartptr + MAC_HEAD_CTRL_OFT), txl_get_seq_ctrl());
    // Update the DTIM count and period
    co_write8p(tim_ie + MAC_TIM_CNT_OFT, vif_entry->u.ap.dtim_count);

    // Update DTIM count for next beacon
    if (vif_entry->u.ap.dtim_count == 0)
    {
        if (vif_entry->u.ap.bc_mc_status)
            bmpc |= MAC_TIM_BCMC_PRESENT;
        else
            bmpc &= ~MAC_TIM_BCMC_PRESENT;
        vif_entry->u.ap.dtim_count = co_read8p(tim_ie + MAC_TIM_PERIOD_OFT);
    }
    else
    {
         bmpc &= ~MAC_TIM_BCMC_PRESENT;
    }
    co_write8p(tim_ie + MAC_TIM_BMPC_OFT, bmpc);
    vif_entry->u.ap.dtim_count--;

    // Update CSA counter in the beacon
    if (vif_entry->u.ap.csa_count)
    {
        uint8_t i;
        vif_entry->u.ap.csa_count --;
        for (i = 0; i < BCN_MAX_CSA_CPT; i++)
        {
            if (vif_entry->u.ap.csa_oft[i] == 0)
                break;
            co_write8p((uint32_t)HW2CPU(thd->datastartptr) + vif_entry->u.ap.csa_oft[i],
                       vif_entry->u.ap.csa_count);
        }
        if (vif_entry->u.ap.csa_count)
        {
            mm_bcn_send_csa_counter_ind(vif_entry->index, vif_entry->u.ap.csa_count);
        }
        // keep csa_count to 1 until beacon is successfully transmistted
        if (vif_entry->u.ap.csa_count == 0)
        {
            vif_entry->u.ap.csa_count = 1;
            os_printf("mm_switch_channel\r\n");
            mm_switch_channel(vif_entry);
        }
    }
    
    if (vif_entry->u.ap.csa_action_count)
    {
        vif_entry->u.ap.csa_action_count --;
        
        apm_send_action_csa_frame(vif_entry);
    }

    // update Tx power in policy table
    tpc_update_frame_tx_power(vif_entry, frame);

    return(bmpc & MAC_TIM_BCMC_PRESENT);
}


void mm_bcn_flush(void)
{
    bcn_tx_cfm_set(0);
}

void mm_csa_set_channel(struct vif_info_tag *p_vif_entry)
{    
    uint8_t channel;
    struct vif_info_tag *vif = p_vif_entry;
    struct mm_chan_ctxt_add_req *csa_channel;

    os_printf("mm_csa_set_channel\r\n");
    csa_channel = &vif->csa_channel;

    channel = rw_ieee80211_get_chan_id(csa_channel->prim20_freq);
    bk_wlan_ap_set_channel_config(channel);
    
    phy_set_channel(csa_channel->band, csa_channel->type, 
                    csa_channel->prim20_freq, csa_channel->center1_freq, 
                    csa_channel->center2_freq, PHY_PRIM);
}

void mm_switch_channel(struct vif_info_tag *p_vif_entry)
{
    vif_mgmt_switch_channel(p_vif_entry);

    mm_csa_set_channel(p_vif_entry);
    
#if CFG_ROLE_LAUNCH
    rl_pre_sta_set_status(RL_STATUS_STA_CHANNEL_SWITCHED);
#endif

    mm_csa_beacon_after_change(); 
    
    mm_bcn_env.update_ongoing = false;
    
    mm_csa_finish_event();    
}

static void mm_bcn_transmitted(void *env, uint32_t status)
{
    // Attached env is the VIF Entry on which beacon has been sent, get VIF index
    struct vif_info_tag *p_vif_entry = (struct vif_info_tag *)env;
    
    // Sanity check - We shall be still waiting for at least one confirmation
    ASSERT_ERR(bcn_tx_cfm_get());

    // One confirmation less to be expected
    bcn_tx_cfm_dec();

    // Check if all confirmations have been received
    if (bcn_tx_cfm_get() == 0)
    {
        // Check if we need to update a beacon
        if (mm_bcn_env.update_pending)
            mm_bcn_update(mm_bcn_env.param);

        // Check if we need to update the TIM
        while (!co_list_is_empty(&mm_bcn_env.tim_list))
        {
            mm_tim_update_proceed(ke_msg2param((struct ke_msg *)
                                                co_list_pop_front(&mm_bcn_env.tim_list)));
        }

        #if (NX_P2P_GO)
        // Check if we have a pending NOA update
        if (mm_bcn_env.p2p_noa_req[p_vif_entry->index] != P2P_BCN_UPD_OP_NONE)
        {
            mm_bcn_update_p2p_noa(p_vif_entry->index, mm_bcn_env.p2p_noa_req[p_vif_entry->index]);
        }
        #endif //(NX_P2P_GO)

        #if (RW_UMESH_EN)
        mesh_ps_beacon_cfm_handle(p_vif_entry);
        #endif //(RW_UMESH_EN)

#if CFG_ROLE_LAUNCH
        if(rl_pre_ap_set_status(RL_STATUS_AP_TRANSMITTED_BCN))
        {
            rl_pre_ap_disable_autobcn();
            
            return;
        }
#endif
    }
}

static void mm_bcn_updated(void *env, int dma_queue)
{
    struct vif_info_tag *vif_entry = (struct vif_info_tag *) env;
    struct ke_msg *msg = ke_param2msg(mm_bcn_env.param);

    // Confirm the beacon update to the host
    ke_msg_send_basic(MM_BCN_CHANGE_CFM, msg->src_id, TASK_MM);

    // Update the frame descriptor
    mm_bcn_desc_prep(vif_entry, mm_bcn_env.param);

    // Init CSA counter
    mm_bcn_csa_init(vif_entry, mm_bcn_env.param);

    mm_bcn_env.update_ongoing = false;
    
    // Check if the beacons need to be sent immediately
    if (mm_bcn_env.tx_pending)
    {
        mm_bcn_env.tx_pending = false;        
        mm_bcn_transmit();        
    }

    // Free the beacon parameters
    ke_msg_free(msg);
}

static void mm_bcn_update(struct mm_bcn_change_req const *param)
{
    struct hal_dma_desc_tag *dma;
    struct txl_buffer_tag *buffer;
    struct vif_info_tag *vif_entry;
    void *bcn_ptr;
    //GLOBAL_INT_DECLARATION();

    vif_entry = &vif_info_tab[param->inst_nbr];

    #if (RW_MESH_EN)
    /*
     * If the beacon is provided for a Mesh Point VIF, it does not come from the host
     * memory. It has been filled by UMAC's Mesh Module and hence does not require
     * any DMA transfer.
     */
    if (vif_entry->type == VIF_MESH_POINT)
    {
        mm_bcn_updated(vif_entry, DMA_DL);
    }
    else
    #endif //(RW_MESH_EN)
    {
        mm_bcn_env.update_pending = false;
        mm_bcn_env.update_ongoing = true;
        bcn_ptr = (void *)param->bcn_ptr;
        os_printf("update_ongoing_1_bcn_update\r\n");
        
        dma = &mm_bcn_env.dma;
        buffer = (struct txl_buffer_tag *)&txl_bcn_pool[param->inst_nbr][0];

        // Fill in the DMA descriptor
        dma->env = vif_entry;
        dma->dma_desc->src = (uint32_t)param->bcn_ptr;
        dma->dma_desc->dest = CPU2HW(buffer->payload);
        dma->dma_desc->length = param->bcn_len;

        // Program the download
        hal_dma_push(dma, DMA_DL);

    	if(param->bcn_ptr_malloc_flag)
    	{
    		os_free(bcn_ptr);
    	}
    }
}

#if (NX_P2P_GO)
static void mm_bcn_init_p2p_noa(uint8_t vif_index)
{
    // Get TX payload buffer descriptor
    struct tx_pbd *p_pbd = &txl_p2p_noa_desc[vif_index];
    // Get payload address
    uint32_t p2p_noa_ie = CPU2HW(&txl_p2p_noa_ie_pool[vif_index][0]);

    // Prepare the descriptor
    p_pbd->upatterntx   = TX_PAYLOAD_DESC_PATTERN;
    p_pbd->datastartptr = p2p_noa_ie;
    p_pbd->next         = (uint32_t)NULL;
    p_pbd->bufctrlinfo  = 0;
}
#endif //(NX_P2P_GO)

#if (RW_UMESH_EN)
static void mm_bcn_init_mesh_add_ies_desc(void)
{
    for (int i = 0; i < RW_MESH_VIF_NB; i++)
    {
        // Get TX payload buffer descriptor
        struct tx_pbd *p_pbd = &txl_mesh_add_ies_desc[i];
        // Get payload address
        uint32_t mesh_add_ie = CPU2HW(&txl_mesh_add_ies[i].buf[0]);

        // Prepare the descriptor
        p_pbd->upatterntx   = TX_PAYLOAD_DESC_PATTERN;
        p_pbd->datastartptr = mesh_add_ie;
        p_pbd->next         = (uint32_t)NULL;
        p_pbd->bufctrlinfo  = 0;
    }
}
#endif //(RW_UMESH_EN)

void mm_bcn_init(void)
{
    #if (NX_P2P_GO)
    uint8_t counter;
    #endif //(NX_P2P_GO)

    mm_hw_ap_disable();
    mm_csa_beacon_flush();
    
    memset(&mm_bcn_env, 0, sizeof(mm_bcn_env));

    mm_bcn_env.dma.dma_desc = &bcn_dwnld_desc;
    mm_bcn_env.dma.cb = mm_bcn_updated;
    co_list_init(&mm_bcn_env.tim_list);

    #if (NX_P2P_GO)
    for (counter = 0; counter < NX_VIRT_DEV_MAX; counter++)
    {
        // Initialize NOA Beacon payload
        p2p_go_bcn_init_noa_pyld(CPU2HW(&txl_p2p_noa_ie_pool[counter][0]));
    }
    #endif //(NX_P2P_GO)

    #if (RW_UMESH_EN)
    mm_bcn_init_mesh_add_ies_desc();
    #endif //(RW_UMESH_EN)
}

int mm_bcn_get_tx_cfm(void)
{
	return bcn_tx_cfm_get();
}

void mm_bcn_init_tim(struct vif_info_tag *vif_entry)
{
    uint8_t inst_nbr = vif_entry->index;
    struct tx_pbd *pbd = &txl_tim_desc[inst_nbr][0];
    uint32_t tim_ie = CPU2HW(&txl_tim_ie_pool[inst_nbr][0]);
    uint32_t tim_bmp = CPU2HW(&txl_tim_bitmap_pool[inst_nbr][0]);

    // Initialize the DTIM count
    vif_entry->u.ap.dtim_count = 0;
    vif_entry->u.ap.tim_len = MAC_TIM_BMP_OFT + 1;
    vif_entry->u.ap.tim_bitmap_set = 0;
    vif_entry->u.ap.tim_n1 = (uint8_t)-1;
    vif_entry->u.ap.tim_n2 = 0;
    vif_entry->u.ap.bc_mc_status = 0;

    // First part of the TIM
    pbd->upatterntx = TX_PAYLOAD_DESC_PATTERN;
    pbd->datastartptr = tim_ie + MAC_TIM_ID_OFT;
    pbd->dataendptr = tim_ie + MAC_TIM_BMP_OFT;
    pbd->next = CPU2HW(&txl_bcn_end_desc[inst_nbr]);
    pbd->bufctrlinfo = 0;
    co_write8p(tim_ie + MAC_TIM_ID_OFT, MAC_ELTID_TIM);
    co_write8p(tim_ie + MAC_TIM_LEN_OFT, 4);
    co_write8p(tim_ie + MAC_TIM_CNT_OFT, vif_entry->u.ap.dtim_count);
    co_write8p(tim_ie + MAC_TIM_PERIOD_OFT, 1); // Will be updated when receiving the beacon from host
    co_write8p(tim_ie + MAC_TIM_BMPC_OFT, 0);
	
    #if !CFG_USE_AP_PS
    co_write8p(tim_ie + MAC_TIM_BMP_OFT, 0x06);  // aid1 & aid2  for ap prevent sta goto sleep
    #else
    co_write8p(tim_ie + MAC_TIM_BMP_OFT, 0x0);
    #endif

    // Reset the TIM virtual bitmap
    pbd = &txl_tim_desc[inst_nbr][1];
    pbd->upatterntx = TX_PAYLOAD_DESC_PATTERN;
    pbd->dataendptr = tim_bmp + vif_entry->u.ap.tim_n2;
    pbd->next = CPU2HW(&txl_bcn_end_desc[inst_nbr]);
    memset(txl_tim_bitmap_pool[inst_nbr], 0, sizeof(txl_tim_bitmap_pool[inst_nbr]));

    // Initialize post-TIM TX PBD
    pbd = &txl_bcn_end_desc[inst_nbr];
    pbd->upatterntx = TX_PAYLOAD_DESC_PATTERN;
    pbd->next = 0;
    pbd->bufctrlinfo = 0;
}

void mm_bcn_init_vif(struct vif_info_tag *vif_entry)
{
    struct txl_frame_desc_tag *frame = &vif_entry->u.ap.bcn_desc;
    struct txl_buffer_tag *buffer = (struct txl_buffer_tag *)&txl_bcn_pool[vif_entry->index][0];
    struct tx_hw_desc_s *hwdesc = &txl_bcn_hwdesc_pool[vif_entry->index];
    struct txl_buffer_control *bufctrl = &txl_bcn_buf_ctrl[vif_entry->index];
    struct tx_hd *thd;

    // Initialize the frame descriptor
    txl_frame_init_desc(frame, buffer, hwdesc, bufctrl);

    // Initialize the TIM buffer
    mm_bcn_init_tim(vif_entry);

    #if (NX_P2P_GO)
    if (vif_entry->p2p)
    {
        mm_bcn_init_p2p_noa(vif_entry->index);
    }
    #endif //(NX_P2P_GO)

    // Initialize some static fields of the THD
    thd = &frame->txdesc.lmac.hw_desc->thd;
    thd->phyctrlinfo = 0;
    thd->macctrlinfo2 = 0;
    thd->first_pbd_ptr = 0;

    // Initialize callback function
    frame->cfm.cfm_func = mm_bcn_transmitted;
    frame->cfm.env = vif_entry;
}


void mm_bcn_change(struct mm_bcn_change_req const *param)
{
    // Save the message for later handling
    mm_bcn_env.param = param;

    // Check if the beacon transmission is already ongoing
    if (bcn_tx_cfm_get())
    {
        mm_bcn_env.update_pending = true;
    }
    else
    {
        // No beacon transmission ongoing, proceed immediately to the update
        mm_bcn_update(param);
    }
}

void mm_tim_update(struct mm_tim_update_req const *param)
{
    // Check if the beacon transmission is already ongoing
    if (bcn_tx_cfm_get())
    {
        // Push the request into the list for later handling
        co_list_push_back(&mm_bcn_env.tim_list, &ke_param2msg(param)->hdr);
    }
    else
    {
        // Proceed immediately to the request
        mm_tim_update_proceed(param);
    }
}

void mm_bcn_transmit(void)
{
    struct vif_info_tag *vif_entry = vif_mgmt_first_used();
    
#if CFG_ROLE_LAUNCH
    if(rl_pre_ap_disable_autobcn())
    {
        if(0 == bcn_tx_cfm_get())
        {
            rl_pre_ap_set_status(RL_STATUS_AP_LAUNCHED);
        }     
        return;
    }      
#endif     
    
    // Sanity check - All previously chained beacons shall be transmitted
    ASSERT_ERR(!bcn_tx_cfm_get());
    
    // Check if a beacon update is ongoing
    if (mm_bcn_env.update_ongoing)
    {
        mm_bcn_env.tx_pending = true;
        return;
    }
    
    // Go through the VIFs and send the beacons of the AP ones
    while (vif_entry != NULL)
    {
        // Check if VIF is an AP one
        if (((vif_entry->type == VIF_AP)
            #if (RW_MESH_EN)
                || (vif_entry->type == VIF_MESH_POINT)
            #endif //(RW_MESH_EN)
                ) && (vif_entry->u.ap.bcn_configured)
                  && (vif_entry->u.ap.bcn_tbtt_cnt == vif_entry->u.ap.bcn_tbtt_ratio)
           )
        {
            uint8_t tmp;

            // Update the frame
            tmp = mm_bcn_build(vif_entry);
            if (tmp)
            {
                mm_traffic_req_ind(VIF_TO_BCMC_IDX(vif_entry->index), 0, false);
            }

            #if (NX_CHNL_CTXT || NX_P2P_GO)
            // Set VIF and STA indexes
            vif_entry->u.ap.bcn_desc.txdesc.host.vif_idx = vif_entry->index;
            vif_entry->u.ap.bcn_desc.txdesc.host.staid   = 0xFF;

            // Push the TX frame descriptor
            if (txl_frame_push(&vif_entry->u.ap.bcn_desc, AC_BCN))
            {
                bcn_tx_cfm_add();
            }
            #else
            txl_frame_push(&vif_entry->u.ap.bcn_desc, AC_BCN);
            bcn_tx_cfm_add();
            #endif //(NX_CHNL_CTXT || NX_P2P_GO)
        }

        // Go to next VIF
        vif_entry = vif_mgmt_next(vif_entry);
    }

}

#if (NX_P2P_GO)
void mm_bcn_update_p2p_noa(uint8_t vif_index, uint8_t operation)
{
    // Check if the beacon transmission is already ongoing
    if (bcn_tx_cfm_get())
    {
        // Save the message for later handling
        mm_bcn_env.p2p_noa_req[vif_index] = operation;
    }
    else
    {
        // Get VIF Info entry
        struct vif_info_tag *p_vif_entry = &vif_info_tab[vif_index];

        // React accordingly with the operation code
        switch (operation)
        {
            case (P2P_BCN_UPD_OP_NOA_ADD):
            {
                txl_bcn_end_desc[vif_index].next = CPU2HW(&txl_p2p_noa_desc[vif_index]);
            } break;

            case (P2P_BCN_UPD_OP_NOA_RMV):
            {
                txl_bcn_end_desc[vif_index].next = (uint32_t)NULL;
            } break;

            case (P2P_BCN_UPD_OP_NOA_UPD):
            {
                // Let the P2P module fulfill the payload part
                p2p_go_bcn_upd_noa_pyld(p_vif_entry->p2p_index, CPU2HW(&txl_p2p_noa_ie_pool[vif_index][0]));
            } break;

            default:
            {
                ASSERT_ERR(0);
            }
        }

        // No P2P NOA update is pending
        mm_bcn_env.p2p_noa_req[vif_index] = P2P_BCN_UPD_OP_NONE;

        // Inform the P2P module about operation completion
        p2p_go_bcn_op_done(p_vif_entry->p2p_index, operation);
    }
}
#endif //(NX_P2P_GO)

#if (RW_MESH_EN)
struct txl_buffer_tag *mm_bcn_get_buffer(uint8_t vif_index)
{
    return (struct txl_buffer_tag *)&txl_bcn_pool[vif_index][0];
}
#endif //(RW_MESH_EN)

#endif //(NX_BCN_AUTONOMOUS_TX)

/// @} end of group
