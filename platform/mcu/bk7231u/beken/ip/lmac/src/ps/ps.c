/**
 ****************************************************************************************
 *
 * @file ps.c
 *
 * @brief Power-Save mode implementation.
 *
 * Copyright (C) RivieraWaves 2011-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup PS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ps.h"
#include "mac_ie.h"
#include "sta_mgmt.h"
#include "vif_mgmt.h"
#include "mm.h"
#include "co_utils.h"
#include "co_status.h"
#include "txl_frame.h"
#include "phy.h"
#include "tpc.h"
#include "uart_pub.h"
#include "power_save_pub.h"
#if (NX_DPSM)
#include "td.h"
#endif //(NX_DPSM)
#include "temp_detect_pub.h"

#if CFG_USE_STA_PS
#include "arm_arch.h"
#include "target_util_pub.h"
#endif
#if NX_POWERSAVE

/**
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of TX attempts for a NULL frame indicating PS mode change
#define PS_TX_ERROR_MAX             (10)

/**
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
struct ps_env_tag ps_env;

/**
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
extern uint32_t rwnxl_get_reseting_flag(void);

void ps_set_data_prevent(void)
{
    txl_cntrl_inc_pck_cnt();
}

void ps_set_key_prevent(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    ps_env.prevent_sleep |= PS_WAITING_ADD_KEY;
    GLOBAL_INT_RESTORE();    
}

void ps_clear_key_prevent(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    ps_env.prevent_sleep &= ~PS_WAITING_ADD_KEY;
    GLOBAL_INT_RESTORE();        
}

static uint8_t ps_send_pspoll(struct vif_info_tag *vif_entry)
{
    struct txl_frame_desc_tag *frame;
    struct mac_hdr_ctrl *buf;
    struct tx_hd *thd;
    struct sta_info_tag *sta_entry = &sta_info_tab[vif_entry->u.sta.ap_id];
    struct phy_channel_info phy_info;
    uint8_t band;
    int txtype;

#if CFG_USE_STA_PS
	PS_PRT("ps s poll\r\n");
#endif
    // Get the RF band on which we are to know which rate to use
    phy_get_channel(&phy_info, PHY_PRIM);

    #if (NX_P2P)
    if (vif_entry->p2p)
    {
        // If P2P interface, 11b rates are forbidden
        txtype = TX_DEFAULT_5G;
    }
    else
    #endif //(NX_P2P)
    {
        band = phy_info.info1 & 0xFF;
        // Chose the right rate according to the band
        txtype = (band == PHY_BAND_2G4)?TX_DEFAULT_24G:TX_DEFAULT_5G;
    }

    // Allocate a frame descriptor from the TX path
    frame = txl_frame_get(txtype, MAC_PSPOLL_FRAME_SIZE);
    if (frame == NULL)
        return CO_FAIL;

    // update Tx power in policy table
    tpc_update_frame_tx_power(vif_entry, frame);

    // Get the buffer pointer
    #if NX_AMSDU_TX
    buf = (struct mac_hdr_ctrl *)frame->txdesc.lmac.buffer[0]->payload;
    #else
    buf = (struct mac_hdr_ctrl *)frame->txdesc.lmac.buffer->payload;
    #endif

    // Fill-in the frame
    buf->fctl = MAC_FCTRL_PSPOLL;
    buf->durid = (0x3 << 14) | sta_entry->aid;
    buf->addr1 = sta_entry->mac_addr;
    buf->addr2 = vif_entry->mac_addr;

    // HW shall not modify the DUR/AID field for a PS-poll
    thd = &frame->txdesc.lmac.hw_desc->thd;
    thd->macctrlinfo2 |= DONT_TOUCH_DUR | TS_VALID_BIT | (MAC_FCTRL_PSPOLL >> 1);

    #if (NX_CHNL_CTXT || NX_P2P)
    // Set VIF and STA indexes
    frame->txdesc.host.vif_idx = sta_entry->inst_nbr;
    frame->txdesc.host.staid   = sta_entry->staid;
    #endif //(NX_CHNL_CTXT || NX_P2P)

    // Push the frame for TX
    txl_frame_push(frame, AC_VO);

    return CO_OK;
}

/**
 ****************************************************************************************
 * @brief Check if NULL frame indicating a PS mode change has been acknowledged by the peer
 * AP.
 * Up to 3 (PS_TX_ERROR_MAX) retries are allowed. If this limit is reached, we consider
 * the connection with the AP has been lost and we continue processing the confirmation
 * properly so that we can continue the mode update.
 *
 * @param[in] p_vif_entry   VIF on which the NULL frame has been set
 * @param[in] status        Transmission status
 * @param[in] cfm           Confirmation callback if we need to resend a NULL frame.
 ****************************************************************************************
 */
static uint32_t ps_check_tx_status(struct vif_info_tag *p_vif_entry, uint32_t status,
                               cfm_func_ptr cfm)
{
    // Indicate if confirmation can be handled
    uint32_t handle_cfm = true;

    // Check if an error has occured during the NULL frame transmission
    if (!(status & FRAME_SUCCESSFUL_TX_BIT))
    {
        if(true == rwnxl_get_reseting_flag() || (!txl_cntrl_tx_check(p_vif_entry)))
        {   //if reset ,process as succes
            handle_cfm = 2;
            return (handle_cfm);
        }
        //return false;
        // Increase number of errors
        p_vif_entry->u.sta.ps_retry++;

        // Check if maximum number of retries has been reached
        if (p_vif_entry->u.sta.ps_retry >= PS_TX_ERROR_MAX)
        {
            #if 0//(NX_CONNECTION_MONITOR)
            //remove ps disconnect
            // Inform the host that the link has been lost
            mm_send_connection_loss_ind(p_vif_entry);
            #else
            PS_PRT("prs\r\n");
            handle_cfm = 2;
            return (handle_cfm);
            #endif //(NX_CONNECTION_MONITOR)
        }
        else
        {
            // Send a new NULL frame
            if(txl_frame_send_null_frame(p_vif_entry->u.sta.ap_id, cfm, p_vif_entry))
            {
            	ps_env.cfm_cnt--;
            }
            
            if (ps_env.cfm_cnt == 0)
            {   //if fail ,process as succes
                handle_cfm = 2;
                return (handle_cfm);
            }
            
            // Wait for next confirmation
            handle_cfm = false;
        }
    }

    return (handle_cfm);
}

 void ps_enable_cfm(void *env, uint32_t status)
{
    uint32_t check;
    do
    {
        // Check if an error has occurred during the NULL frame transmission
        check = ps_check_tx_status((struct vif_info_tag *)env, status, ps_enable_cfm);
       
        if (! check)
        {
            break;
        }

        if(check == 2)
        {
            PS_DPSM_STATE_CLEAR(RESUMING);
            ps_env.prevent_sleep |= PS_PSM_PAUSED;
            nxmac_pwr_mgt_setf((uint8_t)0);

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)

            break;
        }
        
        if(mhdr_get_station_status() < MSG_CONN_SUCCESS)
            {
            PS_PRT("dis en cfm\r\n");
            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)

            break;
        }

        if (ps_env.cfm_cnt)
        {
            // Decrease the number of expected confirmations
            ps_env.cfm_cnt--;
        }

        // Check if all confirmations have been received
        if (ps_env.cfm_cnt == 0)
        {
            #if NX_UAPSD
            // Check if UAPSD is in use
            if (ps_env.uapsd_on)
            {
                struct vif_info_tag *vif_entry =
                              (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

                mm_timer_set(&ps_env.uapsd_timer, ke_time() + ps_env.uapsd_timeout);
                ps_env.uapsd_tmr_on = true;
                while (vif_entry != NULL)
                {
                    if ((vif_entry->type == VIF_STA) && vif_entry->active  && vif_entry->u.sta.uapsd_queues)
                        vif_entry->prevent_sleep &= ~PS_VIF_WAITING_EOSP;

                    // Go to next entry
                    vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
                }
            }
            #endif

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(ON) && PS_DPSM_STATE_GET(RESUMING))
            {
                // Resume RESUMING bit
                PS_DPSM_STATE_CLEAR(RESUMING);
                // Clear PAUSE bit
                PS_DPSM_STATE_CLEAR(PAUSE);
            }
            else
            #endif //(NX_DPSM)
            {
                // Set the PS environment variable
                ps_env.ps_on = true;                
            }

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)
        }
    } while (0);
}

 void ps_disable_cfm(void *env, uint32_t status)
{
    uint32_t check;
	
    do
    {
        // Check if an error has occurred during the NULL frame transmission
        check = ps_check_tx_status((struct vif_info_tag *)env, status, ps_disable_cfm);

        if (! check)
        {
            break;
        }

        if(check == 2)
        {
            PS_DPSM_STATE_CLEAR(PAUSING);
            ps_env.prevent_sleep &= ~PS_PSM_PAUSED;
            nxmac_pwr_mgt_setf((uint8_t)1);

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)

            break;
        }

        if (ps_env.cfm_cnt)
        {
            // Decrease the number of expected confirmations
            ps_env.cfm_cnt--;
        }

        // Check if all confirmations have been received
        if (ps_env.cfm_cnt == 0)
        {
            #if NX_UAPSD
            // Clear the UAPSD timer
            mm_timer_clear(&ps_env.uapsd_timer);
            ps_env.uapsd_tmr_on = false;
            #endif

            #if (NX_DPSM)
            // Check if DPSM is ON and if Pause was requested
            if (PS_DPSM_STATE_GET(ON) && PS_DPSM_STATE_GET(PAUSING))
            {
                // Resume PAUSING bit
                PS_DPSM_STATE_CLEAR(PAUSING);
                // Set PAUSE bit
                PS_DPSM_STATE_SET(PAUSE);
                //start PAUSE exit ckeck
#if CFG_USE_STA_PS
                power_save_delay_sleep_check();
#endif
            }
            else
            #endif //(NX_DPSM)
            {
                // Set the PS environment variable
                ps_env.ps_on = false;
            }

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)
        }
    } while (0);
}

#if (NX_DPSM)
static void ps_dpsm_update(bool pause)
{
    // VIF Entry variable used to parse list of created VIFs
    struct vif_info_tag *vif_entry;
    // Function called when sent NULL frame is ACKed by AP
    cfm_func_ptr cfm ;

    // Request to pause Power Save Mode
    if (pause)
    {
        cfm = ps_disable_cfm;

        // Keep in mind that pause has been requested
        PS_DPSM_STATE_SET(PAUSING);

        #if 1
        // Disallow channel switching
        ps_env.prevent_sleep |= PS_PSM_PAUSED;
        #endif
    }
    // Request to resume Power Save Mode
    else
    {
        cfm = ps_enable_cfm;

        // Keep in mind that resume has been requested
        PS_DPSM_STATE_SET(RESUMING);

        #if 1
        // Allow channel switching
        ps_env.prevent_sleep &= ~PS_PSM_PAUSED;
        #endif
    }

    // Update macCntrl1Reg.pwrMgt bit
    nxmac_pwr_mgt_setf((uint8_t)(pause == false));

    // Reset the confirmation counter
    ps_env.cfm_cnt = 0;

    // Go through the list of STA VIF
    vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

    while (vif_entry)
    {
        if ((vif_entry->type == VIF_STA) && vif_entry->active
            #if (NX_CHNL_CTXT)
            && chan_is_on_channel(vif_entry)
            #endif //(NX_CHNL_CTXT)
           )
        {
            // Reset number of retries for this transmission
            vif_entry->u.sta.ps_retry = 0;
            // We expect one more confirmation
            ps_env.cfm_cnt++;

            // Send a NULL frame to indicate to the AP that we are sleeping
            if( txl_frame_send_null_frame(vif_entry->u.sta.ap_id, cfm, vif_entry))
            {
            	ps_env.cfm_cnt--;
            }

        }

        // Go to next entry
        vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
    }

    // If no NULL frame has been sent, directly call the cfm function
    if (ps_env.cfm_cnt == 0)
    {
        cfm(NULL, 0x0);
    }
}
#endif //(NX_DPSM)

#if (NX_UAPSD)
/**
 ****************************************************************************************
 * @brief Sends a trigger frame to AP to retrieve the data that are buffered
 ****************************************************************************************
 */
static void ps_uapsd_timer_handle(void *env)
{
    bool sta_active = false;
    // Go through the list of STA VIF
    struct vif_info_tag *vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

    while (vif_entry != NULL)
    {
        if ((vif_entry->type == VIF_STA) 
                && (vif_entry->active)
                && (vif_entry->u.sta.uapsd_queues)
                #if NX_CHNL_CTXT
                && chan_is_on_channel(vif_entry)
                #endif
           )
        {
            // Send a trigger frame to all APs with which U-APSD is used
            sta_active = true;

            // Check if we need to send the frame
            if (ke_time_past(vif_entry->u.sta.uapsd_last_rxtx + ps_env.uapsd_timeout/2)
            #if (NX_P2P)
                    && !(vif_entry->p2p 
					&& vif_entry->u.sta.sp_paused)
            #endif //(NX_P2P)
                    )
            {
                vif_entry->prevent_sleep |= PS_VIF_WAITING_EOSP;
                txl_frame_send_qosnull_frame(vif_entry->u.sta.ap_id, 0x7 << MAC_QOSCTRL_UP_OFT,
                                             NULL, NULL);
                vif_entry->u.sta.uapsd_last_rxtx = ke_time();
            }
        }

        // Go to next entry
        vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
    }

    // Check if at least one STA is active
    if (sta_active)
    {
        // Restart the UAPSD Timer
        mm_timer_set(&ps_env.uapsd_timer, ke_time() + ps_env.uapsd_timeout);
    }
    else
    {
        // UAPSD timer is not active anymore
        ps_env.uapsd_tmr_on = false;
    }
}
#endif //(NX_UAPSD)

void ps_init(void)
{
    // Fully reset the PS environment variable
    memset(&ps_env, 0, sizeof(ps_env));

    #if (NX_UAPSD)
    // Initialize mm_timer used for UAPSD
    ps_env.uapsd_timer.cb = ps_uapsd_timer_handle;
    #endif //(NX_UAPSD)
}

#if 1
 void ps_mode_enable_cfm(void *env, uint32_t status)
{
    do
    {
        // Check if an error has occurred during the NULL frame transmission
        if (!ps_check_tx_status((struct vif_info_tag *)env, status, ps_mode_enable_cfm))
        {
            break;
        }

        if (ps_env.cfm_cnt)
        {
            // Decrease the number of expected confirmations
            ps_env.cfm_cnt--;
        }

        // Check if all confirmations have been received
        if (ps_env.cfm_cnt == 0)
        {
            #if NX_UAPSD
            // Check if UAPSD is in use
            if (ps_env.uapsd_on)
            {
                struct vif_info_tag *vif_entry =
                              (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

                mm_timer_set(&ps_env.uapsd_timer, ke_time() + ps_env.uapsd_timeout);
                ps_env.uapsd_tmr_on = true;
                while (vif_entry != NULL)
                {
                    if ((vif_entry->type == VIF_STA) && vif_entry->active  && vif_entry->u.sta.uapsd_queues)
                        vif_entry->prevent_sleep &= ~PS_VIF_WAITING_EOSP;

                    // Go to next entry
                    vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
                }
            }
            #endif

            {
                // Set the PS environment variable
                ps_env.ps_on = true;
                
                #if 1
                // Send the confirmation
                ke_msg_send_basic(MM_SET_PS_MODE_CFM, ps_env.taskid, TASK_MM);
                #else
                #endif
            }

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)
        }
    } while (0);
}

 void ps_mode_disable_cfm(void *env, uint32_t status)
{
    do
    {
    
        // Check if an error has occurred during the NULL frame transmission
        if (!ps_check_tx_status((struct vif_info_tag *)env, status, ps_mode_disable_cfm))
        {
            break;
        }

        if (ps_env.cfm_cnt)
        {
            // Decrease the number of expected confirmations
            ps_env.cfm_cnt--;
        }

        // Check if all confirmations have been received
        if (ps_env.cfm_cnt == 0)
        {
            #if NX_UAPSD
            // Clear the UAPSD timer
            mm_timer_clear(&ps_env.uapsd_timer);
            ps_env.uapsd_tmr_on = false;
            #endif

            {
                // Set the PS environment variable
                ps_env.ps_on = false;

                #if 1
                // Send the confirmation
                ke_msg_send_basic(MM_SET_PS_MODE_CFM, ps_env.taskid, TASK_MM);
                #else
                #endif
            }

            #if (NX_DPSM)
            if (PS_DPSM_STATE_GET(SET_MODE_REQ))
            {
                // Reset SET_MODE_REQ bit
                PS_DPSM_STATE_CLEAR(SET_MODE_REQ);
                // Set PS Mode as required by the application
                ps_set_mode(ps_env.next_mode, ps_env.taskid);
            }
            #endif //(NX_DPSM)
        }
    } while (0);
}
#endif

void ps_set_mode(uint8_t mode, ke_task_id_t taskid)
{
    struct vif_info_tag *vif_entry;
    cfm_func_ptr cfm;

    do
    {
        // Save the ID of the requester
        ps_env.taskid = taskid;

        if(mhdr_get_station_status() < MSG_CONN_SUCCESS)
           {
            if (mode == PS_MODE_OFF)
            {
                #if (NX_DPSM)
                ps_env.dpsm_state = 0;
                #endif
                nxmac_pwr_mgt_setf(0);
                ps_env.ps_on = false;
                ke_msg_send_basic(MM_SET_PS_MODE_CFM, ps_env.taskid, TASK_MM);
            }
            else
            {
                #if (NX_DPSM)
                if (mode == PS_MODE_ON_DYN)
                {
                    PS_DPSM_STATE_SET(ON);
                }
                #endif
                nxmac_pwr_mgt_setf(1);
                ps_env.ps_on = true;
                ke_msg_send_basic(MM_SET_PS_MODE_CFM, ps_env.taskid, TASK_MM);
            }
            ps_env.cfm_cnt = 0;

            // Go through the list of STA VIF
            vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);
            
            while (vif_entry != NULL)
            {
                if ((vif_entry->type == VIF_STA) && vif_entry->active
                    #if NX_CHNL_CTXT
                    && chan_is_on_channel(vif_entry)
                    #endif
                   )
                {
                    vif_entry->u.sta.ps_retry = 0;
                }
                vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
            }

            PS_PRT("dis mode:%x\r\n",mode);
            break;
        }

        #if (NX_DPSM)
        // If update has been required internally, wait for end of procedure
        if (PS_DPSM_STATE_GET(ON) && (PS_DPSM_STATE_GET(PAUSING) || PS_DPSM_STATE_GET(RESUMING)))
        {
            // Keep mode in mind
            ps_env.next_mode = mode;
            // Set SET_MODE_REQ bit to 1
            PS_DPSM_STATE_SET(SET_MODE_REQ);

            // Escape from function
            break;
        }
        #endif //(NX_DPSM)

        if (mode == PS_MODE_OFF)
        {
            // Set the confirmation function pointer
            cfm = ps_mode_disable_cfm;

            #if (NX_DPSM)
            // Clear DPSM state
            ps_env.dpsm_state = 0;
            #endif //(NX_DPSM)

            // PS mode is disabled, then the macCntrl1Reg.pwrMgt bit must be set to 0,
            // in order to reset the PM bit in Frame Control from now on.
            nxmac_pwr_mgt_setf(0);
        }
        else
        {
            // Set the confirmation function pointer
            cfm = ps_mode_enable_cfm;

            #if (NX_DPSM)
            // Check if Dynamic Power Save mode is required
            if (mode == PS_MODE_ON_DYN)
            {
                // Set DPSM ON bit
                PS_DPSM_STATE_SET(ON);
            }
            #endif //(NX_DPSM)

            // PS mode is enabled, then the macCntrl1Reg.pwrMgt bit must be set to 1,
            // all the frames must have their PM bit set to 1 in Frame Control field.
            nxmac_pwr_mgt_setf(1);
        }

        // Reset the confirmation counter
        ps_env.cfm_cnt = 0;
        #if NX_UAPSD
        /// Set the UAPSD ON flag
        ps_env.uapsd_on = false;
        #endif

        // Go through the list of STA VIF
        vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

        while (vif_entry != NULL)
        {
            if ((vif_entry->type == VIF_STA) && vif_entry->active
                #if NX_CHNL_CTXT
                && chan_is_on_channel(vif_entry)
                #endif
               )
            {
                // Reset mon_last_crc
                vif_entry->u.sta.mon_last_crc = 0;
                // Reset number of retries for this transmission
                vif_entry->u.sta.ps_retry = 0;
                // We expect one more confirmation
                ps_env.cfm_cnt++;

                #if NX_UAPSD
                // Check if UAPSD is enabled for this interface
                if (vif_entry->u.sta.uapsd_queues)
                    ps_env.uapsd_on = true;
                #endif

                // Send a NULL frame to indicate to the AP that we are sleeping
                if (txl_frame_send_null_frame(vif_entry->u.sta.ap_id, cfm, vif_entry))
                {
                    ps_env.cfm_cnt--;
                }
            }

            // Go to next entry
            vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
        }
        
        // Check if are waiting for some confirmations before updating our state
        if (ps_env.cfm_cnt == 0)
        {
            // Call confirmation function
            cfm(NULL, 0x0);
        }
    } while (0);
}

/**
 ****************************************************************************************
 * @brief Check if TIM Information Element received from either an AP or a peer Mesh STA if
 * buffered frame targeting us are announced.
 *
 * @param[in] a_tim     Address of the TIM IE in the memory
 * @param[in] aid       Association ID
 ****************************************************************************************
 */
#if (!RW_MESH_EN)
__INLINE
#endif //(!RW_MESH_EN)
bool ps_check_tim(uint32_t a_tim, uint16_t aid)
{
    // Indicate if unicast traffic is announced
    bool uc_present = false;
    // Value used to read the TIM element
    uint8_t mask;
    unsigned int n, n1, n2;

    do
    {
        // Now we check if some unicast traffic is buffered
        // Compute the byte index of this AID
        n = aid / 8;

        // Compute the bit corresponding to this AID inside the byte
        mask  = CO_BIT(aid & 0x7);

        // Now we need to compute the start and end byte indexes present in the TIM
        n1 = co_read8p(a_tim + MAC_TIM_BMPC_OFT) & 0xFE;
        n2 = co_read8p(a_tim + MAC_TIM_LEN_OFT) + n1 - 4;

        // First check if the AID byte is present in the bitmap
        if ((n < n1) || (n > n2))
        {
            break;
        }

        // Byte is present, now computes its offset in the bitmap array
        n -= n1;

        if (co_read8p(a_tim + MAC_TIM_BMP_OFT + n) & mask)
        {
            uc_present = true;
        }
    } while (0);

    return (uc_present);
}

void ps_check_beacon(struct rx_hd *rhd, uint32_t tim, uint16_t len, struct vif_info_tag *vif_entry)
{
    struct rx_pbd *pbd = HW2CPU(rhd->first_pbd_ptr);
    struct bcn_frame *bcn = HW2CPU(pbd->datastartptr);
    uint16_t aid = sta_info_tab[vif_entry->u.sta.ap_id].aid;
#if CFG_USE_STA_PS
    power_save_beacon_len_set(len);
#endif
    do
    {
        // Beacon was received
        vif_entry->prevent_sleep &= ~PS_VIF_WAITING_BCN;

        // Check if we are in PS mode
        if (!ps_env.ps_on)
            {
            break;
            }

        #if (NX_DPSM)
        if (PS_DPSM_STATE_GET(PAUSE))
        {
            break;
        }
        #endif //(NX_DPSM)

        // Check if TIM is present
        if (tim == 0)
            {
            break;
            }

        // First check if there is some BC/MC traffic buffered by the AP
        if (!vif_entry->u.sta.dont_wait_bcmc)
        {
            if (co_read8p(tim + MAC_TIM_BMPC_OFT) & 0x01)
                {
                vif_entry->prevent_sleep |= PS_VIF_WAITING_BCMC;
                #if CFG_USE_STA_PS
                PS_PRT("=bc\r\n");
                #endif
                }
            else
                vif_entry->prevent_sleep &= ~PS_VIF_WAITING_BCMC;
        }

        // Check if unicast traffic is present
        if (ps_check_tim(tim, aid))
        {
            #if NX_UAPSD
            if ((vif_entry->u.sta.uapsd_queues & PS_ALL_UAPSD_ACS) == PS_ALL_UAPSD_ACS)
            {
                // All ACs are uapsd-enabled: send a QoS Null
                if (txl_frame_send_qosnull_frame(vif_entry->u.sta.ap_id, 0x7 << MAC_QOSCTRL_UP_OFT,
                                                 NULL, NULL) == CO_OK)
                {
                    vif_entry->prevent_sleep |= PS_VIF_WAITING_EOSP;
                }
            }
            else
            #endif
            {
                #if (NX_DPSM)
                // Update Traffic Detection statistics for PS. Do it at least
                // TD_DEFAULT_PCK_NB_THRES times to ensure DPSM will be triggered
                td_pck_ps_ind(vif_entry->index, true);
                td_pck_ps_ind(vif_entry->index, true);
                td_pck_ps_ind(vif_entry->index, true);
                // Invoke the TD to proceed to the wakeup immediately
                td_timer_end(&td_env[vif_entry->index]);
                #if CFG_USE_STA_PS
                power_save_delay_sleep_check();
                #endif
                #else                
            
                // Since some ACs are not uapsd-enabled the TIM only refers to the
                // ACs configured to use 802.11 Legacy power save mode
                if (ps_send_pspoll(vif_entry) == CO_OK)
                    vif_entry->prevent_sleep |= PS_VIF_WAITING_UC;

                #endif
            }
        }
        else
        {
            #if NX_UAPSD
            if ((vif_entry->u.sta.uapsd_queues & PS_ALL_UAPSD_ACS) == PS_ALL_UAPSD_ACS)
            {
                vif_entry->prevent_sleep &= ~PS_VIF_WAITING_EOSP;
            }
            else
            #endif
                vif_entry->prevent_sleep &= ~PS_VIF_WAITING_UC;
        }

    } while(0);

    {
#if CFG_USE_STA_PS
        power_save_beacon_state_update();
#endif
        extern UINT8 td_traffic;
        td_traffic ++;
    }
    
}

void ps_check_frame(uint8_t *frame, uint32_t statinfo, struct vif_info_tag *vif_entry)
{
    struct mac_hdr *machdr = (struct mac_hdr *)frame;
    uint16_t framectrl = machdr->fctl;

    do
    {
        // Check if we are in PS mode
        if (!ps_env.ps_on)
            break;

        // Check if the frame is individually or group addressed
        if (MAC_ADDR_GROUP(&machdr->addr1))
        {
            // Check if other group frames will follow this one
            if (!(framectrl & MAC_FCTRL_MOREDATA) || vif_entry->u.sta.dont_wait_bcmc)
                // No more group frames, allow going back to sleep
                vif_entry->prevent_sleep &= ~PS_VIF_WAITING_BCMC;
        }
        else
        {
            // Unicast frame - Ensure that the frame is for us
            if (statinfo & RX_HD_ADDRMIS)
                return;

            do
            {
                #if (NX_UAPSD)
                if (ps_env.uapsd_on)
                {
                    // Check if it is a QoS-data frame
                    if ((framectrl & MAC_FCTRL_QOS_DATA) == MAC_FCTRL_QOS_DATA)
                    {
                        uint16_t qos;
                        uint8_t tid;

                        // Compute the QoS control field offset in the frame
                        if ((framectrl & MAC_FCTRL_TODS_FROMDS) == MAC_FCTRL_TODS_FROMDS)
                        {
                            struct mac_hdr_long_qos *hdr = (struct mac_hdr_long_qos *)machdr;
                            qos = hdr->qos;
                        }
                        else
                        {
                            struct mac_hdr_qos *hdr = (struct mac_hdr_qos *)machdr;
                            qos = hdr->qos;
                        }

                        // Get the QoS control field in the frame and the TID
                        tid = (qos & MAC_QOSCTRL_UP_MSK) >> MAC_QOSCTRL_UP_OFT;

                        // Check if UAPSD is enabled for this TID
                        if (vif_entry->u.sta.uapsd_queues & CO_BIT(mac_tid2ac[tid]))
                        {
                            // Save the time of this reception
                            vif_entry->u.sta.uapsd_last_rxtx = ke_time();

                            // UAPSD is enabled for this TID, check if EOSP is set
                            if (qos & MAC_QOSCTRL_EOSP)
                            {
                                // End of service period, we can go to sleep again
                                vif_entry->prevent_sleep &= ~PS_VIF_WAITING_EOSP;
                            }
                            break;
                        }
                    }
                }
                #endif //(NX_UAPSD)

                #if (NX_DPSM)
                // Update Traffic Detection statistics for PS
                td_pck_ps_ind(vif_entry->index, true);
                #endif //(NX_DPSM)

                // Check the more data bit
                if (framectrl & MAC_FCTRL_MOREDATA)
                {
                    #if (NX_DPSM)
                    if (!PS_DPSM_STATE_GET(PAUSE))
                    #endif //(NX_DPSM)
                    {
                        if (ps_send_pspoll(vif_entry) != CO_OK)
                        {
                            // Allow going back to sleep
                            vif_entry->prevent_sleep &= ~PS_VIF_WAITING_UC;
                        }
                    }
                }
                else
                {
                    // Allow going back to sleep
                    vif_entry->prevent_sleep &= ~PS_VIF_WAITING_UC;
                }
            } while (0);
        }
    } while (0);
}

#if (NX_UAPSD || NX_DPSM)
void ps_check_tx_frame(uint8_t staid, uint8_t tid)
{
    struct sta_info_tag *sta_entry;
    struct vif_info_tag *vif_entry;

    do
    {
        // Check if we are in PS mode
        if (!ps_env.ps_on)
            break;

        // Check if destination STA is known
        if (staid == 0xFF)
            break;

        // Check if TID is valid (indicating that the frame is QoS frame)
        if (tid == 0xFF)
            break;

        // Retrieve the associated VIF entry
        sta_entry = &sta_info_tab[staid];
        vif_entry = &vif_info_tab[sta_entry->inst_nbr];

        // Check if the frame is individually or group addressed
        if ((vif_entry->type != VIF_STA) || !vif_entry->active)
            break;

        #if (NX_UAPSD)
        // Check if UAPSD is enabled for this TID
        if (vif_entry->u.sta.uapsd_queues & CO_BIT(mac_tid2ac[tid]))
        {
            // UAPSD is enabled for this TID, we will have to wait for the EOSP
            vif_entry->prevent_sleep |= PS_VIF_WAITING_EOSP;

            // Save the time of this transmission
            vif_entry->u.sta.uapsd_last_rxtx = ke_time();

            break;
        }
        #endif //(NX_UAPSD)

        #if (NX_DPSM)
        // Update Traffic Detection statistics for PS
        td_pck_ps_ind(vif_entry->index, false);
        #endif //(NX_DPSM)
    } while (0);
}
#endif //(NX_UAPSD || NX_DPSM)

#if (NX_UAPSD)
void ps_uapsd_set(struct vif_info_tag *vif_entry, uint8_t hw_queue, bool uapsd)
{
    if (uapsd)
    {
        // Set the corresponding bit in the VIF parameters
        vif_entry->u.sta.uapsd_queues |= CO_BIT(hw_queue);

        // Check if U-APSD timer is already enabled
        if (ps_env.ps_on && !ps_env.uapsd_tmr_on)
        {
            // UAPSD is used
            ps_env.uapsd_on = true;

            mm_timer_set(&ps_env.uapsd_timer, ke_time() + ps_env.uapsd_timeout);
            ps_env.uapsd_tmr_on = true;
        }
    }
    else
    {
        vif_entry->u.sta.uapsd_queues &= ~CO_BIT(hw_queue);
    }
}
#endif //(NX_UAPSD)

#if (NX_DPSM)
void ps_traffic_status_update(uint8_t vif_index, uint8_t new_status)
{
    // VIF Information
    struct vif_info_tag *p_vif_entry;
    // Indicate if some traffic has been detected on one VIF
    bool has_traffic = (new_status != 0);
    // Indicate if PS is currently paused or not
    bool is_ps_paused;

    do
    {
        // Verify that DPSM is OFF
        if (!PS_DPSM_STATE_GET(ON))
        {
            break;
        }

        // Verify that DPSM is not a transition state
        if (PS_DPSM_STATE_GET(PAUSING) || PS_DPSM_STATE_GET(RESUMING))
        {
            break;
        }

        if (!has_traffic)
        {
            // Go through the list of used VIFs in order to know if there is another VIF with traffic
            p_vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);

            while (p_vif_entry)
            {
                // Skip provided VIF
                if (p_vif_entry->index != vif_index)
                {
                    // Consider only active STA VIFs
                    if (p_vif_entry->active && (p_vif_entry->type == VIF_STA))
                    {
                        if (td_get_ps_status(p_vif_entry->index))
                        {
                            has_traffic = true;
                            break;
                        }
                    }
                }

                // Get next VIF
                p_vif_entry = (struct vif_info_tag *)co_list_next(&p_vif_entry->list_hdr);
            };
        }

        // Check current DPSM state
        is_ps_paused = PS_DPSM_STATE_GET(PAUSE);
        // Based on traffic detection status and current DPSM state, pause or not PS mode
        if (has_traffic)
        {
            // Traffic has been detected, pause PS mode if not already done
            if (!is_ps_paused )
            {
                ps_dpsm_update(true);
            }
        }
        else
        {
            // No traffic has been detected, go back to PSM if not already done
            if (is_ps_paused
#if CFG_USE_STA_PS
                && net_if_is_up()
#endif
            )
            {
                ps_dpsm_update(false);
            }
        }
    } while (0);
}
#endif //(NX_DPSM)

#if (NX_P2P && NX_UAPSD)
void ps_p2p_absence_update(struct vif_info_tag *p_vif_entry, bool absent)
{
    do
    {
        if ((p_vif_entry->type != VIF_STA) || !p_vif_entry->active)
        {
            break;
        }

        if (absent)
        {
            // Check if EOSP prevent sleep bit was set
            if (p_vif_entry->prevent_sleep & PS_VIF_WAITING_EOSP)
            {
                p_vif_entry->u.sta.sp_paused = true;

                // Clear EOSP bit
                p_vif_entry->prevent_sleep &= ~PS_VIF_WAITING_EOSP;
            }
        }
        else
        {
            /*
             * When a Service Period is interrupted by a NoA, the CLI should retriggers
             * the GO when the absence period is over.
             */
            if (p_vif_entry->u.sta.sp_paused)
            {
                // Restore EOSP bit
                p_vif_entry->prevent_sleep |= PS_VIF_WAITING_EOSP;
                // Reset paused status
                p_vif_entry->u.sta.sp_paused = false;

                txl_frame_send_qosnull_frame(p_vif_entry->u.sta.ap_id, 0x7 << MAC_QOSCTRL_UP_OFT,
                                             NULL, NULL);

                p_vif_entry->u.sta.uapsd_last_rxtx = ke_time();
            }
        }
    } while (0);
}
#endif //(NX_P2P && NX_UAPSD)

void ps_env_flush(void)
{
#if CFG_USE_STA_PS
    power_save_set_reseted_flag();
#endif
}

void ps_fake_data_rx_check(void)
{
    struct vif_info_tag *vif_entry;
    uint8_t i;
    for(i = 0; i < NX_VIRT_DEV_MAX; i++)
    {
        vif_entry = &vif_info_tab[i];
        if(vif_entry->active && vif_entry->type == VIF_STA)
        {
        td_pck_ps_ind(vif_entry->index, true);
        td_pck_ps_ind(vif_entry->index, true);
        td_pck_ps_ind(vif_entry->index, true);
        // Invoke the TD to proceed to the wakeup immediately
        td_timer_end(&td_env[vif_entry->index]);
        }
    }
}

UINT8 td_traffic = 0;

void ps_run_td_timer(void)
{
    struct vif_info_tag *vif_entry;
    uint8_t i;
    
    for(i = 0; i < NX_VIRT_DEV_MAX; i++)
    {
        vif_entry = &vif_info_tab[i];
        if(vif_entry->active && vif_entry->type == VIF_STA)
        {
            td_timer_end(&td_env[vif_entry->index]);
        }                    
    }
}

/**
 * FUNCTION PROTOTYPES
 ****************************************************************************************
 */
bool ps_sleep_check(void)
{
    struct vif_info_tag *vif_entry;
    bool sleep_allowed = false;

    do
    {    
        #if (NX_P2P_GO_PS)
        if (!p2p_check_ps_mode())
        #endif //(NX_P2P_GO_PS)
        {
            if (!ps_env.ps_on)
                {
                #if CFG_USE_STA_PS
                power_save_forbid_trace(PS_FORBID_NO_ON);
                #endif
                break;
                }
        }


#if CFG_USE_STA_PS            
        if(power_save_if_sleep_first())
            ps_env.prevent_sleep = 0;
#endif

        if (ps_env.prevent_sleep)
            {
            #if CFG_USE_STA_PS
            if(!power_save_forbid_trace(PS_FORBID_PREVENT))
                PS_PRT("pre_s:%x \r\n",ps_env.prevent_sleep); 
            #endif
            break;
			}

        // Go through the list of VIFs
        sleep_allowed = true;
        vif_entry = (struct vif_info_tag *)co_list_pick(&vif_mgmt_env.used_list);
        while (vif_entry != NULL)
        {
            // No sleep if a VIF has a procedure ongoing
            #if NX_CHNL_CTXT
            if (chan_is_on_channel(vif_entry))
            #endif
            {
                if (vif_entry->prevent_sleep)
                {
                    sleep_allowed = false;
                    #if CFG_USE_STA_PS
                    if(!power_save_forbid_trace(PS_FORBID_VIF_PREVENT))
                        PS_PRT("pre_s:%x idx:%x \r\n",vif_entry->prevent_sleep,vif_entry->index);
                    #endif
                    break;
                }
            }

            // Go to next entry
            vif_entry = (struct vif_info_tag *)co_list_next(&vif_entry->list_hdr);
        }
    } while (0);

#if CFG_USE_STA_PS
    if (sleep_allowed)
    {	
		 sleep_allowed = power_save_sleep();
    }
#endif
    return (sleep_allowed);
}

#else
void ps_set_data_prevent(void)
{
}

void ps_set_key_prevent(void)
{  
}

void ps_clear_key_prevent(void)
{
}
#endif

/// @}
