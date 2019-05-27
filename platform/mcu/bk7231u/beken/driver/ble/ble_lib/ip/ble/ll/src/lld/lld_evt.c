/**
 ****************************************************************************************
 *
 * @file lld_evt.c
 *
 * @brief Definition of the functions used for event scheduling
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLDEVT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include "architect.h"
#include "rwble_config.h"
#include "rwble.h"
#include "ea.h"
#include "lld.h"
#include "lld_pdu.h"
#include "lld_evt.h"

#include "common_bt.h"
#include "common_endian.h"
#include "common_list.h"
#include "kernel_event.h"
#include "kernel_mem.h"
#include "kernel_timer.h"
#include "lld_util.h"
#include "reg_common_em_et.h"
#include "reg_ble_em_cs.h"
#include "reg_ble_em_rx_desc.h"
#include "ble_reg_access.h"
#include "llm.h"
#include "rwip.h"
#include "llcontrl.h"
#include "llc_util.h"
#include "llc_task.h"

#include "llm_util.h"

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT
#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
#include "audio.h"
#endif
#include "RomCallFlash.h"
#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */


#define USE_SOLT_WINSIZE      1

#define TEST_MODE_MARGIN     (4)

/// Default BW 1 slot
#define BW_USED_SLAVE_DFT_SLOT   3//(1)// 3
#define BW_USED_SLAVE_DFT_US     (BW_USED_SLAVE_DFT_SLOT*SLOT_SIZE)

/// Default BW 2 slots
#define BW_USED_SCAN_DFT_US       (2*SLOT_SIZE)

/// Default BW 2 slots
#define BW_USED_ADV_DFT_SLOT     6//(2)
#define BW_USED_ADV_DFT_US       (BW_USED_ADV_DFT_SLOT*SLOT_SIZE)



/// packet and field length (in us) (1MBPS)
#define PAYLOAD_HEADER_LENGTH_US    (16)
#define BYTE_LENGTH_US              (8)
#define CRC_LENGTH_US               (24)
#define CONNECT_REQ_LENGTH_US       (352)

/// Minimum number of connection event for instant calculation (6 according to the SIG)
#define MIN_INSTANT_CON_EVT         (6)

/// number of attempt to automatically reschedule activity
#define LLD_SCAN_AUTO_RESCHEDULE_ATTEMPT     5
#define LLD_ADV_HDC_AUTO_RESCHEDULE_ATTEMPT  5
#define LLD_ADV_LDC_AUTO_RESCHEDULE_ATTEMPT  10

/// defer event type
enum lld_evt_defer_type
{
    /// RX interrupt defer
    LLD_DEFER_RX                 = 0,
    /// End of event interrupt defer
    LLD_DEFER_END,
    /// Test End defer
    LLD_DEFER_TEST_END,
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// Connection parameter update instant defer
    LLD_DEFER_CON_UP_INSTANT,
    /// Channel Map update instant defer
    LLD_DEFER_MAP_UP_INSTANT,
    /// Phys update instant defer
    LLD_DEFER_PHY_UP_INSTANT,
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    LLD_DEFER_MAX
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Environment of the LLD module
struct lld_evt_env_tag lld_evt_env;

/*
 ****************************************************************************************
 *
 * PRIVATE FUNCTION DEFINITIONS
 *
 ****************************************************************************************
 */
static void lld_evt_elt_wait_insert(struct ea_elt_tag *elt)
{
    struct lld_evt_wait_tag *new_wait_elt = kernel_malloc(sizeof(struct lld_evt_wait_tag), KERNEL_MEM_ENV);
    ASSERT_ERR(new_wait_elt);
    new_wait_elt->elt_ptr = elt;
    common_list_push_back(&lld_evt_env.elt_wait,&new_wait_elt->hdr);
}


static struct lld_evt_wait_tag * lld_evt_elt_wait_get(struct ea_elt_tag *elt)
{
    if(elt)
    {
        struct lld_evt_wait_tag *scan_wait_list = (struct lld_evt_wait_tag *)common_list_pick(&lld_evt_env.elt_wait);

        while (scan_wait_list != NULL)
        {
            if(scan_wait_list->elt_ptr == elt)
            {
                common_list_extract(&lld_evt_env.elt_wait, &scan_wait_list->hdr, 0);
                return(scan_wait_list);
            }
            scan_wait_list = (struct lld_evt_wait_tag *)scan_wait_list->hdr.next;
        }
        return (scan_wait_list);
    }
    else
    {
        return ((struct lld_evt_wait_tag *)common_list_pop_front(&lld_evt_env.elt_wait));
    }
}
/**
 ****************************************************************************************
 * @brief Gets the MD bit value of the latest descriptor received
 *
 * @param[in] evt               Pointer to the event
 * @param[in] rx_desc_cnt       Number of packet received
 *
 ****************************************************************************************
 */
static void lld_evt_check_md_bit(struct lld_evt_tag *evt, uint8_t rx_desc_cnt)
{
    uint8_t current_rx_hdl = (em_buf_rx_current_get() + (rx_desc_cnt - 1))% BLE_RX_BUFFER_CNT;
    evt->evt.conn.last_md_rx = ble_rxmd_getf(current_rx_hdl);
}

/**
 ****************************************************************************************
 * @brief Try push a deferred element in the list
 *
 * @param[in] elt           Pointer to the element deferred
 *
 ****************************************************************************************
 */
static void lld_evt_deferred_elt_push(struct ea_elt_tag *elt, uint8_t type, uint8_t rx_desc_cnt)
{

        // Allocate a deferred element structure
        struct lld_evt_deferred_tag *elt_deferred
                    = (struct lld_evt_deferred_tag *)kernel_malloc(sizeof(struct lld_evt_deferred_tag), KERNEL_MEM_ENV);

        elt_deferred->elt_ptr     = elt;
        elt_deferred->type        = type;
        elt_deferred->rx_desc_cnt = rx_desc_cnt;
        // Push the element at the end of the deferred list
        common_list_push_back(&lld_evt_env.elt_deferred, &elt_deferred->hdr);

}

/**
 ****************************************************************************************
 * @brief Handle all the element to be deleted
 *
 *
 ****************************************************************************************
 */
static void lld_evt_delete_elt_handler(void)
{
    // Deferred element
    struct ea_elt_tag *elt;
    // Get first deferred element
    struct lld_evt_delete_tag *to_be_deleted = NULL;
    //Status flag to check if the element can be deleted
    bool deleted = true;


    // Clear kernel event
    kernel_event_clear(KERNEL_EVENT_BLE_EVT_DELETE);

    // First deferred element
    GLOBAL_INT_DIS();
    to_be_deleted = (struct lld_evt_delete_tag *)common_list_pick(&lld_evt_env.elt_to_be_deleted);
    GLOBAL_INT_RES();
    while (to_be_deleted)
    {
        elt     = to_be_deleted->elt_ptr;
        if(elt)
        {
            deleted = lld_evt_elt_delete(elt, to_be_deleted->flush, to_be_deleted->send_ind);
        }
        #if (RW_DEBUG)
        else
        {
            ASSERT_ERR(0);
        }
        #endif

        if(deleted)
        {
            struct lld_evt_delete_tag *to_be_deleted_next = NULL;
            GLOBAL_INT_DIS();
            to_be_deleted_next = (struct lld_evt_delete_tag *)to_be_deleted->hdr.next;
            common_list_extract(&lld_evt_env.elt_to_be_deleted, &to_be_deleted->hdr, 0);
            GLOBAL_INT_RES();
            // Free the deferred element
            kernel_free(to_be_deleted);

            to_be_deleted = to_be_deleted_next;
        }
        else
        {
            GLOBAL_INT_DIS();
            to_be_deleted = (struct lld_evt_delete_tag *)to_be_deleted->hdr.next;
            GLOBAL_INT_RES();
        }
    }

    GLOBAL_INT_DIS();
    #if (DEEP_SLEEP)
    if(common_list_is_empty(&lld_evt_env.elt_to_be_deleted))
    {
        // Clear prevent bit
        rwip_prevent_sleep_clear(RW_DELETE_ELT_ONGOING);
			//	rom_env.rwip_prevent_sleep_clear(RW_DELETE_ELT_ONGOING);
    }
    #endif // #if (DEEP_SLEEP)
    GLOBAL_INT_RES();
}

/**
 ****************************************************************************************
 * @brief Try to get an element in the deferred queue
 *
 * @param[in] elt           Pointer to the element to be inserted
 *
 * @return Pointer to the  element deferred
 ****************************************************************************************
 */
static struct ea_elt_tag *lld_evt_deferred_elt_pop(uint8_t *type, uint8_t *rx_desc_cnt)
{
    // Get first deferred element
    struct lld_evt_deferred_tag *elt_deferred = NULL;
    // Store linked element address
    struct ea_elt_tag *elt = NULL;

    GLOBAL_INT_DIS();
    elt_deferred = (struct lld_evt_deferred_tag *)common_list_pop_front(&lld_evt_env.elt_deferred);
    GLOBAL_INT_RES();

    if (elt_deferred)
    {
        elt = elt_deferred->elt_ptr;

        // Update type value
        *type        = elt_deferred->type;
        // Update RX desc cnt value
        *rx_desc_cnt = elt_deferred->rx_desc_cnt;

        // Free the deferred element
        kernel_free(elt_deferred);
    }

    // Return the linked element
    return (elt);
}


#if (BLE_OBSERVER || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Compute the window offset for initiating mode
 *
 * @param[in] elt           Pointer to the scan element
 *
 * @return Offset to set in the CS-WINOFFSET.
 ****************************************************************************************
 */
static uint16_t lld_evt_compute_winoffset(struct ea_elt_tag *elt)
{
    // Get associated BLE event environment
    struct lld_evt_tag *evt_connected = LLD_EVT_ENV_ADDR_GET(elt->linked_element);
    struct ea_elt_tag *elt_connect = elt->linked_element;
    uint16_t output_offset = 0;

    output_offset = (evt_connected->interval -
            ((elt->timestamp - elt_connect->timestamp)  & BLE_BASETIMECNT_MASK) % evt_connected->interval);

    // If the offset is too close
    if(output_offset < 6)
    {
        // Else set the offset to the next anchor point
        output_offset += evt_connected->interval;
    }

    return (output_offset);

}
#endif //(BLE_OBSERVER || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Reads the low power clock drift from NVDS, compute the resulting SCA and store
 * it in the environment.
 *
 ****************************************************************************************
 */
static void lld_evt_local_sca_init(void)
{
    uint8_t sca;
    uint16_t drift;

#if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_LPCLK_DRIFT;

    // Get the sleep clock accuracy from the storage
    if (nvds_get(NVDS_TAG_LPCLK_DRIFT, &length, (uint8_t *)&drift) != NVDS_OK)
#endif //(NVDS_SUPPORT)
    {
        // If no low power clock drift is found in NVDS, put the default value
#if (BLE_STD_MODE)
        drift = DRIFT_BLE_DFT;
#else //(BLE_STD_MODE)
        drift = DRIFT_BT_DFT;
#endif //(BLE_STD_MODE)
    }

    // Deduce the SCA from the drift
    if (drift < 21)
        sca = SCA_20PPM;
    else if (drift < 31)
        sca = SCA_30PPM;
    else if (drift < 51)
        sca = SCA_50PPM;
    else if (drift < 76)
        sca = SCA_75PPM;
    else if (drift < 101)
        sca = SCA_100PPM;
    else if (drift < 151)
        sca = SCA_150PPM;
    else if (drift < 251)
        sca = SCA_250PPM;
    else
        sca = SCA_500PPM;

    // Put the SCA in the environment
    lld_evt_env.sca = sca;
}

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Computes parameters of the parameter update (instant, offset, winsize)
 *
 * @param[in]  evt_old      Pointer to the old event (before connection update)
 * @param[in]  evt_new      Pointer to the new event (after connection update)
 * @param[out] upd_par      Pointer to the structure containing the parameters for the
 *                          connection update
 *
 ****************************************************************************************
 */
static void lld_evt_update_param_compute(struct ea_elt_tag *elt_old,
        struct ea_elt_tag *new_elt,
        struct lld_evt_update_tag *upd_par)

{
    // Get address of the BLE specific event environment
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);
    // Get address of the BLE specific event environment
    struct lld_evt_tag *evt_new = LLD_EVT_ENV_ADDR_GET(new_elt);

    // The instant will be 6 wake-up times after the next event
    uint16_t count_to_inst = evt_old->evt.conn.latency * MIN_INSTANT_CON_EVT;
    // Compute update instant
    uint16_t instant  = (evt_old->evt.conn.counter + count_to_inst) & RWBLE_INSTANT_MASK;
    // Compute the old event time at instant
    uint32_t time_old = (elt_old->timestamp + (evt_old->interval * count_to_inst)) & BLE_BASETIMECNT_MASK;

    // Compute time between event time instant and latest anchor point timestamp
    int32_t time_diff = CLK_DIFF(new_elt->timestamp, time_old);

    ASSERT_INFO(time_diff > 0, time_old, time_diff);

    // compute number of connection interval between previous anchor point and time instant: nb_interval= ceil(time_diff / interval_time)
    uint32_t nb_interval = ((time_diff + (evt_new->interval-1))/ (evt_new->interval));
    // compute next interval time : timestamp +=  nb_interval * interval_time
    new_elt->timestamp = (new_elt->timestamp + (nb_interval)*evt_new->interval) & BLE_BASETIMECNT_MASK;

    // The tx window size is set to 1.25 ms
    upd_par->win_size   = 1;

    // Program the instant in the old event
    evt_old->evt.conn.instant = instant;

    // Compute the time difference between new time and old time to get the offset
    upd_par->win_offset = (uint16_t)(((new_elt->timestamp - time_old) & BLE_BASETIMECNT_MASK) / 2);
    upd_par->instant    = instant;
}
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Correct the fine time computed to determine the virtual anchor point, due to
 * the fact that the event is missed or skipped.
 *
 * @param[in] evt       The event for which the fine time is corrected
 *
 ****************************************************************************************
 */
/*
static void lld_evt_slave_finetime_correction(struct lld_evt_tag *evt)
{
    // Get Windows Size value from the control structure
    uint16_t cs_winsize  = ble_rxwincntl_get(evt->conhdl);

    if (cs_winsize & BLE_RXWIDE_BIT)
    {
        // Remove half windows size (He hope to catch sync at middle of window)
			#if USE_SOLT_WINSIZE
        cs_winsize = (((cs_winsize & ~BLE_RXWIDE_BIT) >> 1) & BLE_RXWINSZ_MASK) * ((uint32_t)SLOT_SIZE);
			#else
				cs_winsize  = (evt->evt.conn.win_size_backup >> 1 );
			#endif
				
    }
    else
    {
        // Remove half windows size (He hope to catch sync at middle of window)
        cs_winsize = (cs_winsize >> 1);
    }
    // Add the RX in offset (33 us) and the 1/2 RX sync window
    evt->anchor_point.finetime_cnt += evt->evt.conn.rx_win_off_dft + cs_winsize;
}
*/
/**
 ****************************************************************************************
 * @brief Computes the window size formatted for the RXWINCNTL field of the control
 * structure
 * Once the value is computed, the function puts the new value into the control structure
 *
 * @param[in] elt       The element for which the window size has changed
 *
 ****************************************************************************************
 */
static void lld_evt_winsize_change(struct lld_evt_tag *evt, bool instant)
{
    /*
     * Get RX windows size in us (drift included)
     * New window size is winsize + drift before and after calculated window (=> x2)
     */
    uint32_t winsize = (evt->evt.conn.sca_drift << 1) ;
		
    // RX Windows size value put in the control structure
    uint16_t winsize_cs;
	

    if( evt->evt.conn.sync_win_size & BLE_RXWIDE_BIT)
    {	        
		winsize += ((evt->evt.conn.sync_win_size & BLE_RXWINSZ_MASK)*SLOT_SIZE);
    }
    else
    {
        winsize += evt->evt.conn.sync_win_size;
    }

 
	
    // Add the rx path delay compensation
    winsize += evt->evt.conn.rx_win_pathdly_comp;

    // Sanity Check
    if (winsize < LLD_EVT_DEFAULT_RX_WIN_SIZE)
    {
        // RX Windows size cannot be lower than default size
        winsize = LLD_EVT_DEFAULT_RX_WIN_SIZE;
		//UART_PRINTF("ccc\r\n");
    }
    /*
     * Check if half window value + new drift is still less than interval/2
     * Note: Drift is put before and after rx window so it shall be multiply by 2
     */

    if ((winsize >= ((((uint32_t)evt->interval) * SLOT_SIZE * (evt->evt.conn.latency+1)) - LLD_EVT_IFS_DURATION)) && (!instant))
    {
        // Window size is bigger than interval - T_IFS, so ask for link disconnection
        LLD_EVT_FLAG_SET(evt, WAITING_EOEVT_TO_DELETE);
    }
    else
    {
        // Convert to a CS compatible value
     	// if (winsize > (BLE_RXWINSZ_MASK / 2))
		if (winsize > (32))
        {
        	
            // Compute the value in slot count (round number of slots)
            winsize_cs = BLE_RXWIDE_BIT | (((winsize + (SLOT_SIZE - 1)) / SLOT_SIZE) + 2); 
			
        }
        else
        {
            // Use the value given in us
            winsize_cs = (uint16_t)winsize;
        }

        // Write the value into the control structure
        ble_rxwincntl_set(evt->conhdl, winsize_cs);

        evt->evt.conn.sync_win_size = winsize_cs;
    }
}
/**
 ****************************************************************************************
 * @brief Compute the RX window size according to the number of missed connection events
 *
 * The function also checks if the window size becomes more than half of the connection
 * interval. In that case, the connection is terminated.
 *
 * @param evt       The event for which the RX window is increased
 *
 ****************************************************************************************
 */
static void lld_evt_rxwin_compute(struct ea_elt_tag *elt)
{

	
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
	
    // Delay since last synchronization
    uint16_t delay = (evt->evt.conn.missed_cnt  + 1) * evt->interval;

    // Recompute the current drift
    evt->evt.conn.sca_drift = lld_evt_drift_compute(delay, evt->evt.conn.mst_sca);

    evt->evt.conn.sync_win_size = ble_rxwincntl_get(evt->conhdl);

    if(( evt->evt.conn.sync_win_size & (~BLE_RXWIDE_BIT))>(evt->interval/2))
    {
        evt->evt.conn.sync_win_size &= BLE_RXWIDE_BIT;
        evt->evt.conn.sync_win_size += evt->interval/2;
    }

    // Change the value in the control structure
    lld_evt_winsize_change(evt, evt->evt.conn.wait_con_up_sync);

}
/**
 ****************************************************************************************
 * @brief Update the RX window size according to the winsize requested in the connect upd
 *
 * @param evt       The event for which the RX window is increased
 *
 ****************************************************************************************
 */
static void lld_evt_rxwin_update(struct ea_elt_tag *elt, uint16_t slot_offset)
{
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

	
    // Compute current drift
    evt->evt.conn.sca_drift = lld_evt_drift_compute(slot_offset, evt->evt.conn.mst_sca);
    // Add update window size to the drift

	//evt->evt.conn.sca_drift += (evt->evt.conn.update_size * 2 * SLOT_SIZE);
	evt->evt.conn.sca_drift += (evt->evt.conn.update_size * 1 * SLOT_SIZE);  //modified by alen 偏移量过大，导致部分手机连接超时断开的问题
	// get current sync window size from parameters
    evt->evt.conn.sync_win_size = ble_rxwincntl_get(evt->conhdl);

    // Change the value in the control structure
    lld_evt_winsize_change(evt, true);

    // Reset the value used for the update
    evt->evt.conn.update_size      = 0;
    evt->evt.conn.update_offset    = 0;
    evt->evt.conn.wait_con_up_sync = true;
}

/**
 ****************************************************************************************
 * @brief Computes the time of programming of next slave connection event and the fine
 * counter offset to be pushed in the control structure. The function then updates the
 * element fields and control structure accordingly. The computing of the time is performed
 * from the value of the next theoretical sync point and the RX window size
 *
 * @param[in] elt           Pointer to the element for which time is computed
 * @param[in] slot_offset   Number of slots between last synchronization and next event occurrence
 *
 ****************************************************************************************
 */
static void lld_evt_slave_time_compute(struct ea_elt_tag *elt, uint16_t slot_offset)
{
    // Associated BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Number of microseconds between basetime of last event and next event
    uint32_t finetimecnt = (uint32_t)evt->anchor_point.finetime_cnt + (uint32_t)((uint32_t)slot_offset * SLOT_SIZE);
    // Basetime offset
    uint32_t basetime_offset;

    // Get Windows Size value from the control structure
    uint16_t cs_winsize  = ble_rxwincntl_get(evt->conhdl);
	
    //Set the new anchor point value to avoid issue when anchor is skipped
    uint32_t anchor_basetime = CLK_ADD_2(evt->anchor_point.basetime_cnt, (finetimecnt / SLOT_SIZE));
    uint16_t anchor_finetime = (finetimecnt % SLOT_SIZE);
	

	 if (cs_winsize & BLE_RXWIDE_BIT)
    {
        // Remove half windows size (He hope to catch sync at middle of window)
        finetimecnt -= (((cs_winsize & ~BLE_RXWIDE_BIT) >> 1) & BLE_RXWINSZ_MASK) * ((uint32_t)SLOT_SIZE);
    }
    else
    {
        // Remove half windows size (He hope to catch sync at middle of window)
        finetimecnt -= (cs_winsize >> 1);
    }		


    // Remove the RX in offset
    finetimecnt -=  evt->evt.conn.rx_win_off_dft;

    // Recalculate basetime offset and finetime counter
    basetime_offset = (finetimecnt / SLOT_SIZE);
    finetimecnt     = (finetimecnt % SLOT_SIZE);

    // Compute the event time, corrected by the window size
    elt->timestamp = CLK_ADD_2(evt->anchor_point.basetime_cnt, basetime_offset);

    //Set the new anchor point value to avoid issue when anchor is skipped
    evt->anchor_point.basetime_cnt = anchor_basetime;
    evt->anchor_point.finetime_cnt = anchor_finetime;

    // update the duration depending the rx sync window
    elt->duration_min = ((cs_winsize & BLE_RXWIDE_BIT) != 0)?((cs_winsize & ~BLE_RXWIDE_BIT)):(cs_winsize/SLOT_SIZE + 1);

    // add required bandwidth
    elt->duration_min += evt->interval_elt->bandwidth_used;

    if(elt->duration_min < BW_USED_SLAVE_DFT_SLOT)
    {
        elt->duration_min = BW_USED_SLAVE_DFT_SLOT;
    }

    elt->duration_min *= SLOT_SIZE;

    // Set the fine counter offset value in the CS
    ble_fcntoffset_set(evt->conhdl, (uint16_t)finetimecnt);
}

/**
 ****************************************************************************************
 * @brief In slave mode, performs the parameter update (called at instant)
 *
 * @param[in] evt_old  Pointer to the event used before instant
 *
 * @return true if the rx window should updated or false if we keep it
 ****************************************************************************************
 */
static bool  lld_evt_slave_param_update_perform(struct ea_elt_tag *elt_old, uint16_t *slot_offset)
{
    // Get old event pointer
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);
    // Get new element pointer
    struct ea_elt_tag *elt_new = elt_old->linked_element;
    // Get new event pointer
    struct lld_evt_tag *evt_new = LLD_EVT_ENV_ADDR_GET(elt_new);
    // Get LLC environment
    struct llc_env_tag *llc_env_ptr = llc_env[evt_old->conhdl];

    // Retrieve some information from the old event
    if((evt_old->interval != evt_new->interval) ||
            (evt_old->evt.conn.latency != evt_new->evt.conn.latency)   ||
            (llc_env_ptr->sup_to != llc_env_ptr->n_sup_to))
    {
        SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_EVT_SENT, true);
    }

    evt_new->evt.conn.counter         = evt_old->evt.conn.counter;
    evt_new->anchor_point             = evt_old->anchor_point;
    evt_new->tx_prog                  = evt_old->tx_prog;
    evt_new->tx_acl_rdy               = evt_old->tx_acl_rdy;
    evt_new->evt.conn.tx_prog_pkt_cnt = evt_old->evt.conn.tx_prog_pkt_cnt;
    evt_new->tx_llcp_pdu_rdy          = evt_old->tx_llcp_pdu_rdy;
    evt_new->tx_acl_tofree            = evt_old->tx_acl_tofree;
    evt_new->evt.conn.update_offset   = evt_old->evt.conn.update_offset;
    evt_new->evt.conn.update_size     = evt_old->evt.conn.update_size;
    evt_new->evt.conn.instant         = evt_old->evt.conn.instant;
    evt_new->evt_flag                 = evt_old->evt_flag;
    evt_new->evt.conn.instant_action  = evt_old->evt.conn.instant_action;
    evt_new->evt.conn.mst_sca         = evt_old->evt.conn.mst_sca;
    evt_new->evt.conn.eff_max_tx_time = evt_old->evt.conn.eff_max_tx_time;
    evt_new->evt.conn.eff_max_tx_size = evt_old->evt.conn.eff_max_tx_size;
    evt_new->evt.conn.rx_win_off_dft = evt_old->evt.conn.rx_win_off_dft;
    evt_new->evt.conn.rx_win_pathdly_comp = evt_old->evt.conn.rx_win_pathdly_comp;
#if (BLE_2MBPS)
    evt_new->evt.conn.rx_phy          = evt_old->evt.conn.rx_phy;
    evt_new->evt.conn.tx_phy          = evt_old->evt.conn.tx_phy;
#endif
    evt_new->cs_ptr                   = evt_old->cs_ptr;
    evt_new->interval_elt             = evt_old->interval_elt;
    elt_new->duration_min             = BW_USED_SLAVE_DFT_US;
    evt_new->evt.conn.sync_win_size   = evt_old->evt.conn.update_size * (2 * SLOT_SIZE);
    evt_new->default_prio             = evt_old->default_prio;

    // Recompute the max event time based on the new connection interval
    ble_maxevtime_set(evt_new->conhdl, evt_new->interval - elt_new->start_latency);

    // Apply the offset in multiple of 625 us
    *slot_offset += (evt_old->evt.conn.update_offset << 1);

    // Update Interval element
    evt_new->interval_elt->offset_used     = elt_old->timestamp % evt_new->interval;
    evt_new->interval_elt->interval_used   = evt_new->interval;
    evt_new->interval_elt->bandwidth_used  = (elt_new->duration_min%SLOT_SIZE !=0)? 1 : 0;
    evt_new->interval_elt->bandwidth_used += elt_new->duration_min/SLOT_SIZE;

    /*
     * Copy new event structure to old event structure
     * Needs to be done as last action because old event parameters may be used by the LLC
     */
    memcpy(evt_old, evt_new, sizeof(struct lld_evt_tag));

    // Reset linked element pointer
    elt_old->linked_element = NULL;
    evt_old->evt.conn.missed_cnt = 0;
    // Reset linked element
    elt_new->linked_element = NULL;
    evt_new->interval_elt = NULL;
    #if (BLE_AUDIO)
    if(evt_old->interval <= AUDIO_MIN_INTERVAL_SLOT)
    {
        elt_old->start_latency = AUDIO_MIN_PROG_LATENCY;
    }
    #endif // (BLE_AUDIO)

    // Free the new event structure
    lld_evt_delete_elt_push(elt_new, false, false);


    return (true);
}
/**
 ****************************************************************************************
 * @brief Check if slave has to listen for the next connection event or not
 *
 * @param[in] evt The event for which the window size has changed
 *
 ****************************************************************************************
 */
static void lld_evt_slave_wakeup(struct lld_evt_tag *evt)
{
    uint16_t latency = evt->evt.conn.latency - 1;

    // Correct the latency in case the total accuracy is 1000ppm
    if ((evt->evt.conn.mst_sca == SCA_500PPM)     &&
            (lld_evt_env.sca == SCA_500PPM)  &&
            (latency > LLD_EVT_MAX_LATENCY))
    {
        /*
         * Latency is too high and may cause connection loss due to Window Widening, so
         * limit it to the max value
         */
        latency = LLD_EVT_MAX_LATENCY;
    }

    // Check if the next event is the instant
    if ((evt->evt.conn.counter == ((evt->evt.conn.instant - 1) & 0xFFFF)) &&
            (evt->evt.conn.instant_action != LLD_UTIL_NO_ACTION))
    {
        LLD_EVT_FLAG_SET(evt, WAITING_INSTANT);
    }
}
#endif //(BLE_PERIPHERAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief In master mode, performs the actions at instant
 *
 * @param[in] evt  Pointer to the event used before instant
 *
 * @return If the rx window size should be updated or not
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL)
static bool lld_evt_slave_instant(struct ea_elt_tag *elt, uint16_t *slot_offset)
{
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    bool status = false;
    // One event before the switch instant, prepare the event with new parameters
    switch (evt->evt.conn.instant_action)
    {
        case LLD_UTIL_NO_ACTION:
        {
            // Nothing to do, should not happen
        } break;

        case LLD_UTIL_PARAM_UPDATE:
        {
			//UART_PRINTF("lld_evt_slave_instant\r\n");
            // Switch to new parameters
            status = lld_evt_slave_param_update_perform(elt, slot_offset);
            llc_lsto_con_update(evt->conhdl);
        } break;

        // A channel map update has to be performed
        case LLD_UTIL_CHMAP_UPDATE:
        {
            // Warn the LLC about the channel map update
            llc_map_update_ind(evt->conhdl);
        } break;
        #if (BLE_2MBPS)
        // A phys update has to be performed
        case LLD_UTIL_PHY_UPDATE:
        {
            // Warn the LLC about the channel map update
            lld_util_phy_update_ind(elt);
            // Indicate that the sync window should be updated
            status = false;
        } break;
        #endif //(BLE_2MBPS)

        default:
        {
            ASSERT_ERR(0);
        } break;
    }
    return (status);
}


/**
 ****************************************************************************************
 * @brief In slace mode, performs the actions at instant
 *
 * @param[in] evt  Pointer to the event used before instant
 ****************************************************************************************
 */
static void lld_evt_slave_instant_evt(struct ea_elt_tag *elt)
{
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    switch (evt->evt.conn.instant_action)
    {
        // A parameter update has to be performed
        case LLD_UTIL_PARAM_UPDATE:
        {
			//UART_PRINTF("lld_evt_slave_instant_evt\r\n");
            // When the switch instant has passed, update the time of the last sync with the offset
            evt->evt.conn.update_offset = 0;

            lld_evt_deferred_elt_push(elt, LLD_DEFER_CON_UP_INSTANT, 0);
        } break;

        // A channel map update has to be performed
        case LLD_UTIL_CHMAP_UPDATE:
        {
            lld_evt_deferred_elt_push(elt, LLD_DEFER_MAP_UP_INSTANT, 0);
        } break;
        #if(BLE_2MBPS)
        // A phys update has to be performed
        case LLD_UTIL_PHY_UPDATE:
        {
            lld_evt_deferred_elt_push(elt, LLD_DEFER_PHY_UP_INSTANT, 0);
        } break;
        #endif
        default:
        {
            // Nothing is done
        } break;
    }

    // Reset waiting instant flag
    LLD_EVT_FLAG_RESET(evt, WAITING_INSTANT);

    // Clear the action
    evt->evt.conn.instant_action = LLD_UTIL_NO_ACTION;
    // Clear the instant
    evt->evt.conn.instant = 0;
}


#endif //(BLE_PERIPHERAL)
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Compute the window offset for initiating mode
 *
 * @param[in] elt           Pointer to the scan element
 *
 * @return Offset to set in the CS-WINOFFSET.
 ****************************************************************************************
 */
static uint32_t lld_evt_get_next_free_slot(void)
{
    // Get associated BLE event environment
    uint32_t free_slot = 0;
    if (!common_list_is_empty(&lld_evt_env.elt_prog))
    {
        struct ea_elt_tag *last_elt = (struct ea_elt_tag *)lld_evt_env.elt_prog.last;
        free_slot = (last_elt->timestamp + last_elt->duration_min/SLOT_SIZE) & BLE_BASETIMECNT_MASK;
    }
    else
    {
        free_slot = lld_evt_time_get();
    }

    return (free_slot);
}


#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief In master mode, performs the parameter update (called at instant)
 *
 * @param[in] evt_old  Pointer to the event used before instant
 ****************************************************************************************
 */
static void lld_evt_master_param_update_perform(struct ea_elt_tag *elt_old)
{
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);
    // Get new event pointer
    struct ea_elt_tag *elt_new = elt_old->linked_element;
    // Get associated BLE event environment
    struct lld_evt_tag *evt_new = LLD_EVT_ENV_ADDR_GET(elt_new);
    // Get LLC task instance environment
    struct llc_env_tag *llc_env_ptr = llc_env[evt_old->conhdl];

    // Retrieve some information from the old event
    if((evt_old->interval != evt_new->interval) ||
            (evt_old->evt.conn.latency != evt_new->evt.conn.latency)   ||
            (llc_env_ptr->sup_to != llc_env_ptr->n_sup_to))
    {
        SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_EVT_SENT, true);
    }

    // Retrieve some information from the old event
    evt_new->evt.conn.counter           = evt_old->evt.conn.counter;
    evt_new->evt.conn.instant           = evt_old->evt.conn.instant;
    evt_new->evt.conn.tx_prog_pkt_cnt   = evt_old->evt.conn.tx_prog_pkt_cnt;
    evt_new->tx_prog                    = evt_old->tx_prog;
    evt_new->tx_acl_rdy                 = evt_old->tx_acl_rdy;
    evt_new->tx_llcp_pdu_rdy            = evt_old->tx_llcp_pdu_rdy;
    evt_new->tx_acl_tofree              = evt_old->tx_acl_tofree;
    evt_new->conhdl                     = evt_old->conhdl;
    evt_new->evt_flag                   = evt_old->evt_flag;
    evt_new->evt.conn.instant_action    = evt_old->evt.conn.instant_action;
    evt_new->evt.conn.eff_max_tx_time   = evt_old->evt.conn.eff_max_tx_time;
    evt_new->evt.conn.eff_max_tx_size   = evt_old->evt.conn.eff_max_tx_size;
    #if (BLE_2MBPS)
    evt_new->evt.conn.rx_phy            = evt_old->evt.conn.rx_phy;
    evt_new->evt.conn.tx_phy            = evt_old->evt.conn.tx_phy;
    #endif
    evt_new->cs_ptr                     = evt_old->cs_ptr;
    evt_new->interval_elt               = evt_old->interval_elt;
    // Remove one interval period, it will be added back in the restart function
    elt_old->timestamp                  = (elt_new->timestamp - evt_new->interval) & BLE_BASETIMECNT_MASK;

    // Update Interval element
    evt_new->interval_elt->offset_used      = elt_old->timestamp % evt_new->interval;
    evt_new->interval_elt->interval_used    = evt_new->interval;
    evt_new->interval_elt->bandwidth_used   = (elt_new->duration_min%SLOT_SIZE !=0)? 1 : 0;
    evt_new->interval_elt->bandwidth_used   += elt_new->duration_min/SLOT_SIZE;
    evt_new->default_prio                   = evt_old->default_prio;

    /*
     * Copy new event structure to old event structure
     * Needs to be done as last action because old event parameters may be used by the LLC
     */
    memcpy(evt_old, evt_new, sizeof(struct lld_evt_tag));

    elt_old->linked_element = NULL;
    // Reset linked element
    elt_new->linked_element = NULL;
    evt_new->interval_elt = NULL;

    #if (BLE_AUDIO)
    if(evt_old->interval <= AUDIO_MIN_INTERVAL_SLOT)
    {
        elt_old->start_latency = AUDIO_MIN_PROG_LATENCY;
    }
    #endif // (BLE_AUDIO)
    /*
     * New values are applied directly to control structure (before instant has passed)
     * It is acceptable as the "param update" procedure always finished and these
     * parameters are just given as information from the spec
     */
    ble_minevtime_set(evt_old->conhdl, (elt_old->duration_min/SLOT_SIZE));

    lld_util_compute_ce_max(elt_old, llc_env[evt_old->conhdl]->data_len_ext_info.conn_eff_max_tx_time,
            llc_env[evt_old->conhdl]->data_len_ext_info.conn_eff_max_rx_time);

    // Free the new event structure
    lld_evt_delete_elt_push(elt_new,false, false);
}

/**
 ****************************************************************************************
 * @brief In master mode, performs the actions at instant
 *
 * @param[in] evt  Pointer to the event used before instant
 ****************************************************************************************
 */
static void lld_evt_master_instant(struct ea_elt_tag *elt)
{
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    switch (evt->evt.conn.instant_action)
    {
        // A parameter update has to be performed
        case LLD_UTIL_PARAM_UPDATE:
        {
            // Switch to new parameters
            lld_evt_master_param_update_perform(elt);
            llc_lsto_con_update(evt->conhdl);
        } break;

        // A channel map update has to be performed
        case LLD_UTIL_CHMAP_UPDATE:
        {
            // Warn the LLC about the channel map update
            llc_map_update_ind(evt->conhdl);
        } break;
        #if (BLE_2MBPS)
        // A phys update has to be performed
        case LLD_UTIL_PHY_UPDATE:
        {
            lld_util_phy_update_ind(elt);
        } break;
        #endif //(BLE_2MBPS
        default:
        {
            // Nothing is done
        } break;
    }


}
/**
 ****************************************************************************************
 * @brief In master mode, performs the actions at instant
 *
 * @param[in] evt  Pointer to the event used before instant
 ****************************************************************************************
 */
static void lld_evt_master_instant_evt(struct ea_elt_tag *elt)
{
    // Get BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    switch (evt->evt.conn.instant_action)
    {
        // A parameter update has to be performed
        case LLD_UTIL_PARAM_UPDATE:
        {
            lld_evt_deferred_elt_push(elt, LLD_DEFER_CON_UP_INSTANT, 0);
        } break;

        // A channel map update has to be performed
        case LLD_UTIL_CHMAP_UPDATE:
        {
            lld_evt_deferred_elt_push(elt, LLD_DEFER_MAP_UP_INSTANT, 0);
        } break;
        #if(BLE_2MBPS)
        // A phys update has to be performed
        case LLD_UTIL_PHY_UPDATE:
        {
            lld_evt_deferred_elt_push(elt, LLD_DEFER_PHY_UP_INSTANT, 0);
        } break;
        #endif
        default:
        {
            // Nothing is done
        } break;
    }

    // Clear the action
    evt->evt.conn.instant_action = LLD_UTIL_NO_ACTION;
    // Clear the instant
    evt->evt.conn.instant = 0;
}

#endif //(BLE_CENTRAL)



/**
 ****************************************************************************************
 * @brief Check if after an apfm IRQ the event should be restarted
 *
 * @param[in] elt           Pointer to the scan element
 *
 * @return True if element reprogrammed; else not able to continue activity
 ****************************************************************************************
 */
static bool lld_evt_continue(struct ea_elt_tag *elt)
{
    // Get associated BLE Event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Status
    bool status = false;
    switch (evt->mode)
    {
        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        case LLD_EVT_ADV_MODE:
        {
            // only valid for High duty cycle advertising
            if(llm_util_get_adv_type() == LLD_LD_ADVERTISER)
            {
                break;
            }
        }
        //no break
        #endif //(BLE_PERIPHERAL || BLE_BROADCASTER)
        #if (BLE_CENTRAL || BLE_OBSERVER)
        case LLD_EVT_SCAN_MODE:
        #endif //(BLE_CENTRAL || BLE_OBSERVER)
        {
            // Check if element(s) is(are) programmed
            elt->timestamp = lld_evt_get_next_free_slot();

            // Start event as soon as possible with a limit of event duration
            EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_LIMIT, EA_NO_PARITY, 0, LLD_SCAN_AUTO_RESCHEDULE_ATTEMPT, rwip_priority[RWIP_PRIO_SCAN_IDX].increment);
            elt->asap_limit = evt->evt.non_conn.end_ts;

            // try to insert the new element
            if(ea_elt_insert(elt) == EA_ERROR_OK)
            {
                lld_evt_elt_wait_insert(elt);
                status = true;
            }
        } break;
        default: /* Nothing to do */ break;
    }
    return (status);
}

static void lld_evt_priority_reset(struct ea_elt_tag *elt)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    elt->current_prio = evt->default_prio;
}

/**
 ****************************************************************************************
 * @brief Reset function, delet all pending element and all memory allocated
 ****************************************************************************************
 */
static void lld_evt_reset(void)
{
    struct ea_elt_tag *elt_to_delete = NULL;
    struct lld_evt_tag *evt = NULL;
    struct lld_evt_wait_tag *scan_wait_list = NULL;
    GLOBAL_INT_DIS();
    /**
     * Deletion of one or several events
     */
    while ((!common_list_is_empty(&lld_evt_env.elt_prog)) || (!common_list_is_empty(&lld_evt_env.elt_wait)))
    {
        // Ge t element in the programmed queue
        elt_to_delete = (struct ea_elt_tag *)common_list_pop_front(&lld_evt_env.elt_prog);

        //No element in programemd queue
        if(!elt_to_delete)
        {
            //Check the pending queue
            scan_wait_list = (struct lld_evt_wait_tag *)common_list_pop_front(&lld_evt_env.elt_wait);
            if(scan_wait_list)
            {
                elt_to_delete = scan_wait_list->elt_ptr;
                kernel_free(scan_wait_list);
            }
        }

        //If no more element exit
        if(!elt_to_delete)
        {
            break;
        }

        evt = LLD_EVT_ENV_ADDR_GET(elt_to_delete);

        /**
         * Flush pending pdu attached to this event
         */
        lld_pdu_tx_flush(evt);
        /**
         * Remove and free interval and element attached to this event
         */
        if(evt->interval_elt != NULL)
        {
            ea_interval_remove(evt->interval_elt);
            kernel_free(evt->interval_elt);
        }
        /**
         *  If a linked element exists, free it
         */
        if (elt_to_delete->linked_element != NULL)
        {
            kernel_free(elt_to_delete->linked_element);
        }
        /**
         * Free element attached to the event
         */
        if (elt_to_delete != NULL)
        {
            kernel_free(elt_to_delete);
        }
    };
    GLOBAL_INT_RES();
}
/*
 ****************************************************************************************
 *
 * PUBLIC FUNCTION DEFINITIONS
 *
 ****************************************************************************************
 */

void lld_evt_delete_elt_push(struct ea_elt_tag *elt, bool flush, bool send_indication)
{
    GLOBAL_INT_DIS();
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Allocate a deferred element structure
    if((!evt->delete_ongoing) || (elt == NULL))
    {
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

        struct lld_evt_delete_tag *deleted_elt
            = (struct lld_evt_delete_tag *)kernel_malloc(sizeof(struct lld_evt_delete_tag), KERNEL_MEM_NON_RETENTION);

        deleted_elt->elt_ptr    = elt;
        deleted_elt->flush      = flush;
        deleted_elt->send_ind   = send_indication;

        if(elt)
        {
            evt->delete_ongoing = true;
        }

        // Push the element at the end of the deferred list
        common_list_push_back(&lld_evt_env.elt_to_be_deleted, &deleted_elt->hdr);
    }
    GLOBAL_INT_RES();

    #if (DEEP_SLEEP)
    // Set prevent bit
    rwip_prevent_sleep_set(RW_DELETE_ELT_ONGOING);
		//rom_env.rwip_prevent_sleep_set(RW_DELETE_ELT_ONGOING);
    #endif // (DEEP_SLEEP)

    // Set kernel event
    kernel_event_set(KERNEL_EVENT_BLE_EVT_DELETE);
}

void lld_evt_channel_next(uint16_t conhdl, int16_t nb_inc)
{
    // Get hopping information from the CS
    uint16_t hopcntl = ble_hopcntl_get(conhdl);
    // Get last channel
    uint16_t last_ch  = hopcntl & BLE_CH_IDX_MASK;
    // Get hopping value
    uint16_t hop     = (hopcntl & BLE_HOP_INT_MASK) >> BLE_HOP_INT_LSB;
    // Perform requested number of hopping
    int16_t next_ch  = (last_ch + (nb_inc * hop)) % 37;

    if (next_ch < 0)
    {
        next_ch += 37;
    }

    // Set tx power index for this channel    
    rwnx_cal_set_txpwr_by_channel(next_ch);

    // Set it in the control structure
    ble_ch_idx_setf(conhdl, next_ch);
}

void lld_evt_init(bool reset)
{
    // Initialize the local SCA
    lld_evt_local_sca_init();
    lld_evt_env.renew                    = false;
    lld_evt_env.hw_wa_sleep_compensation = 0;

    if (reset)
    {
        // Flush all lists containing elements
        lld_util_flush_list(&lld_evt_env.elt_deferred);
        lld_util_flush_list(&lld_evt_env.rx_pkt_deferred);
        lld_evt_reset();
        //Clear delete queue if not empty
        while(!common_list_is_empty(&lld_evt_env.elt_to_be_deleted))
        {
            struct common_list_hdr *elt_to_free = common_list_pop_front(&lld_evt_env.elt_to_be_deleted);
            kernel_free(elt_to_free);
        }
        #if (DEEP_SLEEP)
        // Set prevent bit
        rwip_prevent_sleep_clear(RW_DELETE_ELT_ONGOING);
			//	rom_env.rwip_prevent_sleep_clear(RW_DELETE_ELT_ONGOING);
        #endif //  (DEEP_SLEEP)
    }

    common_list_init(&lld_evt_env.elt_prog);
    common_list_init(&lld_evt_env.elt_deferred);
    common_list_init(&lld_evt_env.rx_pkt_deferred);
    common_list_init(&lld_evt_env.elt_to_be_deleted);

    // Register BLE end event kernel event
    kernel_event_callback_set(KERNEL_EVENT_BLE_EVT_DEFER,   &lld_evt_deffered_elt_handler);
    // Register BLE end event kernel event
    kernel_event_callback_set(KERNEL_EVENT_BLE_EVT_DELETE,   &lld_evt_delete_elt_handler);
}

void lld_evt_init_evt(struct lld_evt_tag *evt)
{
    // Initialize Max power
    evt->tx_pwr = rwip_rf.txpwr_max;
    // Reset the RX/TX counter
    evt->rx_cnt = 0;
    //Initialized delete on going
    evt->delete_ongoing = false;

    GLOBAL_INT_DIS();
    // Initialize TX ACL lists (Prog and Ready)
    common_list_init(&evt->tx_acl_rdy);
    common_list_init(&evt->tx_acl_tofree);
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    // Initialize TX LLCP lists (Prog and Ready)
    common_list_init(&evt->tx_llcp_pdu_rdy);
    #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
    common_list_init(&evt->tx_prog);
    GLOBAL_INT_RES();
}


bool lld_evt_restart(struct ea_elt_tag *elt, bool rejected)
{
    // Get associated BLE Event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    bool can_be_restarted = true;

    DBG_SWDIAG(EVT, BLE_RESTART, 1);

    switch (evt->mode)
    {
        // Cases where event restart has to be performed
        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        case LLD_EVT_ADV_MODE:
        {
            if(llm_util_get_adv_type() == LLD_HD_ADVERTISER)
            {
                // Check if element(s) is(are) programmed
                elt->timestamp = lld_evt_get_next_free_slot();
                elt->asap_limit = evt->evt.non_conn.end_ts;
                EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_LIMIT, EA_NO_PARITY, 0, LLD_ADV_HDC_AUTO_RESCHEDULE_ATTEMPT, rwip_priority[RWIP_PRIO_ADV_HDC_IDX].increment);

                if (CLK_DIFF(elt->timestamp, elt->asap_limit) < 0)
                {
                    // event cannot be restarted
                    can_be_restarted = false;
                }
            }
            else
            {
                // Advertising delay is random between 0 and 10ms, and must have an even value
                uint16_t delay = (uint16_t)COMMON_ALIGN2_HI(common_rand_byte() & 0x0F);

                // Add the interval to the time to get the next target time
                elt->timestamp = (elt->timestamp + evt->interval + delay) & BLE_BASETIMECNT_MASK;
                EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_NO_LIMIT, EA_NO_PARITY, 0, LLD_ADV_LDC_AUTO_RESCHEDULE_ATTEMPT, rwip_priority[RWIP_PRIO_ADV_IDX].increment);
            }
        } break;
        #endif //(BLE_PERIPHERAL || BLE_BROADCASTER)

        #if (BLE_CENTRAL || BLE_OBSERVER)
        case LLD_EVT_SCAN_MODE:
        {
            // If Initiating
            // and if we got a synchronization during the event
            // and if connection request has been sent
            if(evt->evt.non_conn.initiate && ble_evtrxok_getf(evt->conhdl) && evt->evt.non_conn.connect_req_sent)
            {
                // Get the RX time from the control structure
                evt->anchor_point.basetime_cnt = (uint32_t)ble_btcntsync0_get(LLD_ADV_HDL)
                                                           | (((uint32_t)ble_btcntsync1_get(LLD_ADV_HDL)) << 16);
                evt->anchor_point.finetime_cnt = LLD_EVT_FINECNT_MAX - ble_fcntrxsync_getf(LLD_ADV_HDL);

                // event shall not be restarted
                can_be_restarted = false;
                break;
            }

            // calculate new anchor
            evt->evt.non_conn.anchor = (evt->evt.non_conn.anchor + evt->interval) & BLE_BASETIMECNT_MASK;
            // save anchor into timestamp
            elt->timestamp = evt->evt.non_conn.anchor;
            // Retrieve end of the event
            evt->evt.non_conn.end_ts = (evt->evt.non_conn.anchor + (evt->evt.non_conn.window/SLOT_SIZE)) & BLE_BASETIMECNT_MASK;

            EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_LIMIT, EA_NO_PARITY, 0, LLD_SCAN_AUTO_RESCHEDULE_ATTEMPT, rwip_priority[RWIP_PRIO_SCAN_IDX].increment);
            elt->asap_limit = evt->evt.non_conn.end_ts;

        } break;
        #endif //(BLE_CENTRAL || BLE_OBSERVER)

        #if (BLE_CENTRAL)
        case LLD_EVT_MST_MODE:
        {
            // Compute next anchor point using new parameters one event before the switch instant
            if ((evt->evt.conn.counter == ((evt->evt.conn.instant - 1) & 0xFFFF)) && (evt->evt.conn.instant_action != LLD_UTIL_NO_ACTION))
            {
                lld_evt_master_instant(elt);
            }
            // Send the event to the host only when the instant is reached
            else if (evt->evt.conn.counter == (evt->evt.conn.instant))
            {
                lld_evt_master_instant_evt(elt);
            }

            // If the element has not been inserted (Collision in the ET)
            if (rejected)
            {
                // Compute the hopping
                lld_evt_channel_next(evt->conhdl, 1);
            }

            #if(BLE_AUDIO)
            uint8_t voice_ch;
            if(audio_get_voice_channel(evt->conhdl, &voice_ch) != COMMON_ERROR_UNKNOWN_CONNECTION_ID)
            {
                if(audio_channel_enable(voice_ch))
                {
                    audio_evt_restart(voice_ch, elt, rejected);
                }
            }
            #endif //#if(BLE_AUDIO)

            elt->timestamp = (elt->timestamp + evt->interval) & BLE_BASETIMECNT_MASK;

            evt->evt.conn.missed_cnt = 0;

            // Increment the event counter
            evt->evt.conn.counter++;

            // check if waiting for first sync
            if(LLD_EVT_FLAG_GET(evt, WAITING_SYNC))
            {
                // 6 attempts without sync reached
                if(evt->evt.conn.counter >= 6)
                {
                    // connection establishment considered failed
                    LLD_EVT_FLAG_SET(evt, WAITING_EOEVT_TO_DELETE);
                }
            }
            else
            {
                // Change the event Arbiter priority if not rejected by EA
                if(!rejected)
                {
                    lld_util_priority_set(elt, (LLD_EVT_FLAG_GET(evt, WAITING_TXPROG) || LLD_EVT_FLAG_GET(evt, WAITING_ACK))
                            ? RWIP_PRIO_LE_CON_ACT_IDX : RWIP_PRIO_LE_CON_IDLE_IDX);
                }
            }
        } break;
        #endif //(BLE_CENTRAL)

        #if (BLE_PERIPHERAL)
        case LLD_EVT_SLV_MODE:
        {
            /*
             * Number of slots between last detected synchronization and next event occurrence
             *     - A 16-bit value allows to miss the sync during 65535 slots -> 40s (Max LSTO is 32s)
             */
            uint16_t slot_offset = 0;
            /*
             * Number of missed events
             *     - This value will be added to the interval event counter value
             */
            uint16_t evt_cnt_inc = 1;
            /*
             * Number of channel hopping that have to be performed by SW
             *     - A channel hopping is performed by the HW when the event start
             */
            uint16_t chnl_hop_nb = 0;

            // Boolean to decide if the rx sync window should be updated
            bool rx_sync_win_update = false;

            // reset the latency active flag before trying to reschedule
            if(!rejected)
            {
                LLD_EVT_FLAG_RESET(evt, LATENCY_ACTIVE);
            }
            evt->evt.conn.missed_cnt = 0;
            // Check if we got a synchronization during the event or if it is rejected
            if ((!ble_evtrxok_getf(evt->conhdl)) || (rejected))
            {
                if(rejected)
                {
                    // An hopping has to be done
                    chnl_hop_nb++;
                }
            }
            else
            {
                bool tx_lists_not_empty;
                // Set the  Sync Window default size
                ble_rxwincntl_set(evt->conhdl,LLD_EVT_DEFAULT_RX_WIN_SIZE);

                GLOBAL_INT_DIS();
                tx_lists_not_empty = (!common_list_is_empty(&evt->tx_acl_rdy)) || (!common_list_is_empty(&evt->tx_llcp_pdu_rdy))
                        || (!common_list_is_empty(&evt->tx_prog));
                GLOBAL_INT_RES();



                // Check if we have data to transmit or if a procedure is ongoing
                if (tx_lists_not_empty
                        || LLD_EVT_FLAG_GET(evt, WAITING_ACK)
                        || LLD_EVT_FLAG_GET(evt, WAITING_INSTANT)
                        || (evt->evt.conn.last_md_rx))
                {
                    // In such case we don't use slave latency
                    evt->evt.conn.missed_cnt = 0;
                }
                else
                {
                    uint16_t latency = evt->evt.conn.latency - 1;

                    // Correct the latency in case the total accuracy is 1000ppm
                    if ((evt->evt.conn.mst_sca == SCA_500PPM) && (lld_evt_env.sca == SCA_500PPM) &&
                            (latency > LLD_EVT_MAX_LATENCY))
                    {
                        // Latency is too high and may cause connection loss due to Window Widening, so
                        // limit it to the max value
                        latency = LLD_EVT_MAX_LATENCY;
                    }

                    if ((evt->evt.conn.instant_action != LLD_UTIL_NO_ACTION) && (((evt->evt.conn.instant - 1) & 0xFFFF) > evt->evt.conn.counter))
                    {
                        latency = common_min(evt->evt.conn.instant - evt->evt.conn.counter - 2, latency);
                    }

                    // mark that latency is active
                    if(latency>1)
                    {
                        LLD_EVT_FLAG_SET(evt, LATENCY_ACTIVE);
                    }

                    evt->evt.conn.missed_cnt = latency;
                    evt_cnt_inc += latency;
                    chnl_hop_nb += latency;
                }

                // Get the RX time from the control structure
                evt->anchor_point.basetime_cnt = (uint32_t)ble_btcntsync0_get(evt->conhdl) |
                        (((uint32_t)ble_btcntsync1_get(evt->conhdl)) << 16);
                evt->anchor_point.finetime_cnt = LLD_EVT_FINECNT_MAX - ble_fcntrxsync_getf(evt->conhdl);
                evt->evt.conn.wait_con_up_sync = false;
            }

            // Check if the event has to be reprogrammed
            if (LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
            {
                break;
            }

            #if(BLE_AUDIO)
            uint8_t voice_ch;
            if(audio_get_voice_channel(evt->conhdl, &voice_ch) != COMMON_ERROR_UNKNOWN_CONNECTION_ID)
            {
                if(audio_channel_enable(voice_ch))
                {
                    audio_evt_restart(voice_ch, elt, rejected);
                }
            }
            #endif //#if(BLE_AUDIO)
            // Compute slot offset
            slot_offset += evt->interval * (evt->evt.conn.missed_cnt + 1);

            /**
             * Procedures with instant management
             */
            if (LLD_EVT_FLAG_GET(evt, WAITING_INSTANT))
            {
            	//UART_PRINTF("111111\r\n");
                /**
                 * (Instant T0-1) management
                 */
                if (evt->evt.conn.counter == ((evt->evt.conn.instant - 1) & 0xFFFF))
                {
                	//UART_PRINTF("0000000\r\n");
                    rx_sync_win_update = lld_evt_slave_instant(elt, &slot_offset);
                }
                /**
                 * (Instant T0)  management
                 */
                else if (evt->evt.conn.counter == evt->evt.conn.instant)
                {
                    ASSERT_ERR(evt->evt.conn.counter == evt->evt.conn.instant);
                    lld_evt_slave_instant_evt(elt);
                }
                else
                {
                    ASSERT_WARN(0, evt->evt.conn.counter, evt->evt.conn.instant);
                    LLD_EVT_FLAG_RESET(evt, WAITING_INSTANT);
                }
            }

            /**
             * Sync window should be updated accordingly the procedure done at (T0-1)
             */
            if(rx_sync_win_update)
            {
                lld_evt_rxwin_update(elt, slot_offset);
            }
            else
            {
                // Recompute the sync window
                lld_evt_rxwin_compute(elt);
            }

            // Compute the time of the next event according to new Anchor point
            lld_evt_slave_time_compute(elt, slot_offset);

            // Increment the event counter
            evt->evt.conn.counter += evt_cnt_inc;

            evt->evt.conn.missed_cnt = 0;

            // Perform needed channel hopping
            if (chnl_hop_nb)
            {
                lld_evt_channel_next(evt->conhdl, chnl_hop_nb);
            }

            // Check if the event has to be programmed or not
            lld_evt_slave_wakeup(evt);

            // check if waiting for first sync
            if(LLD_EVT_FLAG_GET(evt, WAITING_SYNC))
            {
                // 6 attempts without sync reached
                if(evt->evt.conn.counter >= 6)
                {
                    // connection establishment considered failed
                    LLD_EVT_FLAG_SET(evt, WAITING_EOEVT_TO_DELETE);
                }
            }
            else
            {
                // Change the event Arbiter priority if not rejected by EA
                if(!rejected)
                {
                    lld_util_priority_set(elt, (LLD_EVT_FLAG_GET(evt, WAITING_TXPROG) || LLD_EVT_FLAG_GET(evt, WAITING_ACK))
                            ? RWIP_PRIO_LE_CON_ACT_IDX : RWIP_PRIO_LE_CON_IDLE_IDX);
                }
            }
        } break;
        #endif //(BLE_PERIPHERAL)
        #if (BLE_TEST_MODE_SUPPORT)
        case LLD_EVT_TEST_MODE:
        {
            // Add the interval to the time to get the next target time
            elt->timestamp = (elt->timestamp  + TEST_MODE_MARGIN) & BLE_BASETIMECNT_MASK;
			//UART_PRINTF("R start t = %x\r\n",elt->timestamp);
        } break;
        #endif //(BLE_TEST_MODE_SUPPORT)
        default:
        {
            ASSERT_ERR(0);
        } break;
    }

    DBG_SWDIAG(EVT, BLE_RESTART, 0);

    return can_be_restarted && !LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE);
}

void lld_evt_elt_insert(struct ea_elt_tag *elt, bool inc_prio)
{
//		UART_PRINTF("elt_insert");
    bool evt_restarted = true;

    // Check if the element should be stopped and freed
    while (evt_restarted && (ea_elt_insert(elt) == EA_ERROR_REJECTED))
    {
        // Increment element priority
        if (inc_prio && (elt->current_prio < RWIP_PRIO_MAX))
        {
            elt->current_prio++;
        }
        // try to restart the event
        evt_restarted = lld_evt_restart(elt, true);
				
			
    };
	//	UART_PRINTF("evt_restarted = %d\r\n",evt_restarted);
    // insert only if event can be restarted
    if(evt_restarted)
    {
        GLOBAL_INT_DIS();
        lld_evt_elt_wait_insert(elt);
        GLOBAL_INT_RES();
			 
    }
    else
    {
        // Push the element in the Tx / Rx pending list
        lld_evt_deferred_elt_push(elt, LLD_DEFER_END, 0);
        // Defer the TX and RX handling
        kernel_event_set(KERNEL_EVENT_BLE_EVT_DEFER);
				
    }
}



bool lld_evt_elt_delete(struct ea_elt_tag *elt, bool flush_data, bool send_indication)
{
    bool elt_deleted = false;
    // Get the associated BLE event
    bool found = false;
    ASSERT_INFO(elt != NULL, elt,send_indication);
    GLOBAL_INT_DIS();
    //Check if the element is on going
    found = common_list_find(&lld_evt_env.elt_prog,&elt->hdr);

    // If the element is ongoing
    if(found)
    {
        // Set the delete status flag
        LLD_EVT_FLAG_SET(LLD_EVT_ENV_ADDR_GET(elt), WAITING_EOEVT_TO_DELETE);

        #if (DEEP_SLEEP)
        //Prevent sleep
			  rwip_prevent_sleep_set(RW_DELETE_ELT_ONGOING);
       // rom_env.rwip_prevent_sleep_set(RW_DELETE_ELT_ONGOING);
        #endif // #if (DEEP_SLEEP)
    }
    else
    {
        struct lld_evt_wait_tag *elt_wait_to_free = NULL;
        struct ea_elt_tag *elt_to_delete = NULL;
        struct lld_evt_tag *evt = NULL;
        do
        {
            // Get the element which point on the event in the pending mirror queue
            /**
             * Deletion of one or several events management
             */
            elt_wait_to_free = lld_evt_elt_wait_get(elt);

            // If the input elt is set to NULL the Flush all elements
            if(elt == NULL)
            {
                if(elt_wait_to_free == NULL)
                {
                    break; // Initialization phase (1st time)
                }
                elt_to_delete = elt_wait_to_free->elt_ptr;
            }
            else // Flush only one element
            {
                if (elt_deleted)
                {
                    break; // Deleted in the 1st loop
                }
                elt_to_delete = elt;
            }
            evt = LLD_EVT_ENV_ADDR_GET(elt_to_delete);

            if(send_indication)
            {
                #if (BLE_CENTRAL || BLE_PERIPHERAL)
                //Send indication
                if(evt->conhdl <BLE_CONNECTION_MAX)
                {
                    kernel_msg_send_basic(LLD_STOP_IND, KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), TASK_LLD);
                }
                else
                #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
                {
                    kernel_msg_send_basic(LLD_STOP_IND, TASK_LLM, TASK_LLD);
                }
            }

            // If Not reset phase remove it from ea queues
            ea_elt_remove(elt_to_delete);

            /**
             * Extract element from the programming list if found
             */
            common_list_extract(&lld_evt_env.elt_prog, &elt_to_delete->hdr, 0);

            /**
             * Flush pending pdu attached to this event
             */
            if(flush_data)
            {
                // Clean all data lists
                lld_pdu_tx_flush(evt);
            }

            /**
             * Remove and free interval and element attached to this event
             */
            if(evt->interval_elt != NULL)
            {
                ea_interval_remove(evt->interval_elt);
                kernel_free(evt->interval_elt);
            }
            // If a linked element exists, free it
            if (elt_to_delete->linked_element != NULL)
            {
                kernel_free(elt_to_delete->linked_element);
            }
            /**
             * Free element attached to the event
             */
            if (elt_to_delete != NULL)
            {
               kernel_free(elt_to_delete);
            }
            /**
            * Free mirror element attached to the event
            */
            if(elt_wait_to_free != NULL)
            {
               // Free the element
               kernel_free(elt_wait_to_free);
            }

           // Element has been deleted.
           elt_deleted = true;
        }while (elt_wait_to_free != NULL);

    }
    GLOBAL_INT_RES();

    return (elt_deleted);
}

uint16_t lld_evt_drift_compute(uint16_t delay, uint8_t master_sca)
{
    // Compute the total accuracy in ppm
    uint32_t accuracy = ((uint32_t) common_sca2ppm[lld_evt_sca_get()])
                                                            + ((uint32_t) common_sca2ppm[master_sca]);

    // Compute the drift from the interval and the accuracy and add max max expected drift
    // (41 / 2^16) = 0.0006256103515625 means approximate slot duration in seconds
    // delay is in slot
    // accuracy is in ppm
    // so result of this function is in us
    return (((((uint32_t)delay) * accuracy * 41) >> 16) + 1) + LLD_EVT_MAX_JITTER;
}

#if (BLE_PERIPHERAL)
void lld_evt_schedule_next(struct ea_elt_tag *elt)
{
    // Get event BLE specific environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Get current time
    uint32_t curr_time = (ea_time_get_halfslot_rounded() + elt->start_latency) & BLE_BASETIMECNT_MASK;
    // Number of event that can be reprogram
    int32_t nb_reprog_evt;
    // drift to remove on sync window
    uint32_t drift;
    // sync window
    uint32_t winsize;
    // Slot offset between last synchronization and next occurence of the event
    uint16_t slot_offset;
    // number of slot to reprogram.
    uint32_t reprog_delay;
    // boolean used for element search in a list
    bool found;

    do
    {
        // Check if the event is a slave one
        if (evt->mode != LLD_EVT_SLV_MODE)
            break;

        // Check if slave latency is enabled aligned
        if (!LLD_EVT_FLAG_GET(evt, LATENCY_ACTIVE))
            break;

        // Check if next programmed occurrence of the event is not too close in the future
        if (lld_evt_time_cmp(elt->timestamp, CLK_ADD_2(curr_time, evt->interval)))
            break;

        // Compute the number of event remaining till next anchor time
        // the anchor point is in the future, check if it's possible to schedule earlier
        nb_reprog_evt  = CLK_SUB(evt->anchor_point.basetime_cnt, curr_time);
        nb_reprog_evt /= evt->interval;
        

        // Check if we need to stop latency
        if (nb_reprog_evt < 1)
            break;


        // Check if the event is not already programmed to exchange memory before trying to remove it from wait queue
        GLOBAL_INT_DIS();

        // check if an LLCP has to be sent
        if(!common_list_is_empty(&(evt->tx_llcp_pdu_rdy)))
        {
            // Signal that data has to be transmitted
            LLD_EVT_FLAG_SET(evt, WAITING_TXPROG);
        }

        found = common_list_find(&lld_evt_env.elt_prog, &elt->hdr);
        if(!found)
        {
            // remove element from wait queue
            struct lld_evt_wait_tag *elt_to_free = lld_evt_elt_wait_get(elt);
            // Clean the local element
            if(elt_to_free)
            {
                kernel_free(elt_to_free);
            }
            ea_elt_remove(elt);
        }
        GLOBAL_INT_RES();
        if (found)
            break;

        
        // Compute number of event that are not skipped anymore
        reprog_delay = (nb_reprog_evt)*evt->interval;
        // retrieve an anchor point just before current time
        evt->anchor_point.basetime_cnt = CLK_SUB(evt->anchor_point.basetime_cnt, reprog_delay + evt->interval);
        // recompute event counter and channel index
        evt->evt.conn.counter   -= nb_reprog_evt+1;
        lld_evt_channel_next(evt->conhdl, (-1)*nb_reprog_evt);
        // mark latency stopped
        LLD_EVT_FLAG_RESET(evt, LATENCY_ACTIVE);

        // recalculate the sync window
        drift = lld_evt_drift_compute(nb_reprog_evt, evt->evt.conn.mst_sca);

        // retrieve current window size
        winsize =  ble_rxwincntl_get(evt->conhdl);

        // if it's a winsize in slots:
        if(winsize & BLE_RXWIDE_BIT)
        {
            winsize = (winsize& BLE_RXWINSZ_MASK) * SLOT_SIZE;
        }

        // remove the drift
        winsize -= (drift << 1);

        // Sanity Check
        if (winsize < LLD_EVT_DEFAULT_RX_WIN_SIZE)
        {
            // RX Windows size cannot be lower than default size
            winsize = LLD_EVT_DEFAULT_RX_WIN_SIZE;
        }

        // Convert to a CS compatible value
        if (winsize > 32)
        {
            // Compute the value in slot count (round number of slots)
            winsize = BLE_RXWIDE_BIT | (((winsize + (SLOT_SIZE - 1)) / SLOT_SIZE));
        }

        // Write the value into the control structure
        ble_rxwincntl_set(evt->conhdl, (uint16_t)winsize);
				
        // keep latest CS value
        evt->evt.conn.sync_win_size = winsize;

        /*------------------------------------------------------------------------------------
         * Compute anchor point of next occurrence
         *  -> update offset has to be taken in account in case of update procedure
         *------------------------------------------------------------------------------------*/
        slot_offset = evt->interval;

        while (1)
        {
            // Compute the window size according to the number of missed events
            lld_evt_rxwin_compute(elt);

            // Compute the next event programming time
            lld_evt_slave_time_compute(elt, slot_offset);


            // Check if this time is far enough in the future
            if (lld_evt_time_cmp(curr_time, elt->timestamp))
				break;
               

            // Next anchor point is too close in the future due to window widening, so
            // schedule the event 1 interval later
            evt->evt.conn.missed_cnt++;

            // Compute the window size according to the number of missed events
            //lld_evt_rxwin_compute(elt);
        }

        /*------------------------------------------------------------------------------------
         * Re-compute the event counter and the channel index
         *------------------------------------------------------------------------------------*/
        evt->evt.conn.counter   += evt->evt.conn.missed_cnt+1;
        lld_evt_channel_next(evt->conhdl, evt->evt.conn.missed_cnt);

        // mark that wait instant is not expected
        LLD_EVT_FLAG_RESET(evt, WAITING_INSTANT);

        // ensure that instant will be well processed if we are closed to it
        if ((evt->evt.conn.counter == ((evt->evt.conn.instant - 1) & 0xFFFF)) &&
            (evt->evt.conn.instant_action != LLD_UTIL_NO_ACTION))
        {
            LLD_EVT_FLAG_SET(evt, WAITING_INSTANT);
        }

        // Insert back the event to the list
        lld_evt_elt_insert(elt, true);
    } while (0);
}
#endif //(BLE_OBSERVER || BLE_CENTRAL)



void lld_evt_schedule(struct ea_elt_tag *elt)
{
	//  UART_PRINTF("lld_evt_schedule t = %x\r\n",ea_time_get_halfslot_rounded());	
    DBG_SWDIAG(EVT, BLE_SCHEDULE, 1);
    if(elt != NULL)
    {
        // Get associated BLE event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
        // Get index of the event in the exchange table
        int8_t em_idx = elt->timestamp & 0x0F;
        // Get the element in the local wait queue
        struct lld_evt_wait_tag *elt_to_free = lld_evt_elt_wait_get(elt);
        // Clean the local element
        if(elt_to_free)
        {
            kernel_free(elt_to_free);
        }

        #if (BLE_BROADCASTER || BLE_PERIPHERAL)
        if (evt->mode == LLD_EVT_ADV_MODE)
        {
            if (LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
            {
                lld_evt_deferred_elt_push(elt, LLD_DEFER_END, 0);
                return;
            }

            // Check if advertising data need to be updated
            llm_util_adv_data_update();

            // for direct advertising, calculate event duration
            if(llm_util_get_adv_type() == LLD_HD_ADVERTISER)
            {
                //Compute the new window size
                int16_t evt_duration_new  = (evt->evt.non_conn.end_ts - elt->timestamp) & BLE_BASETIMECNT_MASK;

                //set the new maximum event time. ensure that new duration is not zero
                ble_maxevtime_set(LLD_ADV_HDL, (evt_duration_new > 0) ? evt_duration_new : 1);
            }
        }
        #endif//(BLE_BROADCASTER || BLE_PERIPHERAL)
        #if (BLE_OBSERVER || BLE_CENTRAL)
        if (evt->mode == LLD_EVT_SCAN_MODE)
        {
            if (LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
            {
                lld_evt_deferred_elt_push(elt, LLD_DEFER_END, 0);
                return;
            }
            //Compute the new event size
            int16_t evt_duration_new  = (evt->evt.non_conn.end_ts - elt->timestamp) & BLE_BASETIMECNT_MASK;

            // ensure that we have a minimal duration of 1 slot, not a negative value
            if(evt_duration_new <= 0)
            {
                evt_duration_new = 1;
            }

            //set the new maximum event time. ensure that new duration is not zero
            ble_maxevtime_set(LLD_ADV_HDL, evt_duration_new);
            ble_rxwincntl_pack(LLD_ADV_HDL, 1, evt_duration_new);

            // If initiating
            if(elt->linked_element != NULL)
            {
                // Compute the new window offset for the next scan event
                ble_winoffset_set(LLD_ADV_HDL, (lld_evt_compute_winoffset(elt)>>1) - 1);
            }
        }
        #endif//(BLE_BROADCASTER || BLE_PERIPHERAL)
        #if (BLE_PERIPHERAL || BLE_CENTRAL)
        //Clear ET field
        em_common_extab0_set(em_idx, 0);

        if ((evt->mode == LLD_EVT_SLV_MODE) || (evt->mode == LLD_EVT_MST_MODE))
        {
            // Program the new RX max time and size in the CS
            lld_util_dle_set_cs_fields(evt->conhdl);
            #if(BLE_AUDIO)
            {
                uint8_t voice_ch;
                if(audio_get_voice_channel(evt->conhdl, &voice_ch) != COMMON_ERROR_UNKNOWN_CONNECTION_ID)
                {
                    if(audio_channel_enable(voice_ch))
                    {
                        audio_evt_schedule(voice_ch, elt);
                    }
                }
            }
            #endif //#if(BLE_AUDIO)
        }
        #endif
        
        // Set the event counter in the control structure, used for encryption AES-CTR + useful for debug
        ble_evtcnt_set(evt->conhdl, evt->evt.conn.counter);

        // Set the status to ready
        em_common_extab0_status_setf(em_idx, EM_ET_STATUS_READY);
        em_common_extab0_mode_setf(em_idx, EM_ET_MODE_BLE);
        // Program the CS ptr in the Exchange Table
        em_common_extab1_set(em_idx, evt->cs_ptr);

        // Program the TX data buffers in the control structure
        lld_pdu_tx_prog(evt);

        // Push the element in the programmed environment queue
        common_list_push_back(&lld_evt_env.elt_prog,&elt->hdr);
				
				

        // check if Resolvable private address should be renewed
        if((evt->mode == LLD_EVT_SCAN_MODE) || (evt->mode == LLD_EVT_ADV_MODE))
        {
            if(lld_evt_env.renew)
            {
                // force RPA renew
                lld_util_ral_force_rpa_renew();
                // clear the renew bit field
                lld_evt_env.renew = false;
            }
        }
    }
    DBG_SWDIAG(EVT, BLE_SCHEDULE, 0);
}

void lld_evt_prevent_stop(struct ea_elt_tag *elt)
{
//    ASSERT_ERR(0);
	//UART_PRINTF("lld_evt_prevent_stop \r\n");
}

#if (BLE_CENTRAL || BLE_OBSERVER)


struct ea_elt_tag *lld_evt_scan_create(uint16_t handle, uint16_t latency)
{
    // Allocate event structure
    struct ea_elt_tag *elt  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Sanity check: There should always be a free event
    ASSERT_ERR(elt != NULL);

    // Initialize BLE event environment
    lld_evt_init_evt(evt);

    evt->conhdl       = handle;
    evt->evt.conn.latency      = latency + 1;
    evt->mode         = LLD_EVT_SCAN_MODE;
    evt->cs_ptr       = REG_BLE_EM_CS_ADDR_GET(handle);

    //Set the SCANNING priority
    lld_util_priority_set(elt, RWIP_PRIO_SCAN_IDX);
    elt->ea_cb_start   = lld_evt_schedule;
    elt->ea_cb_cancel  = lld_evt_canceled;
    elt->ea_cb_stop    = lld_evt_prevent_stop;
    elt->start_latency = RWBLE_PROG_LATENCY_DFT;
    elt->duration_min  = BW_USED_SCAN_DFT_US;
    EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_NO_LIMIT, EA_NO_PARITY, 0, LLD_SCAN_AUTO_RESCHEDULE_ATTEMPT, rwip_priority[RWIP_PRIO_SCAN_IDX].increment);
    elt->stop_latency1 = 0;
    elt->stop_latency2 = 0;
    // Schedule event as soon as possible
    elt->timestamp = (ea_time_get_halfslot_rounded() + 2*LLD_EVT_START_MARGIN) & BLE_BASETIMECNT_MASK ;


    // Return pointer to the created event
    return (elt);
}

#endif //(BLE_CENTRAL || BLE_OBSERVER)

#if (BLE_CENTRAL)
struct ea_elt_tag* lld_evt_move_to_master(struct ea_elt_tag *elt_scan, uint16_t conhdl, struct llc_create_con_req_ind const *pdu_tx, uint8_t rx_hdl)
{
    // Allocate element for the connection
    struct ea_elt_tag *elt_connect = elt_scan->linked_element;
    // Get associated BLE event environment
    struct lld_evt_tag *evt_connect = LLD_EVT_ENV_ADDR_GET(elt_connect);
    // Get BLE scan event environment
    struct lld_evt_tag *evt_scan      = LLD_EVT_ENV_ADDR_GET(elt_scan);
    // Allocate a new interval element
    struct ea_interval_tag *new_intv = ea_interval_create();
    // Current time value (basetime)
    uint32_t current_time = 0;
    // End of connect request packet time
    uint32_t end_cnx_req_packet_time = 0;
    // First connection anchor point
    uint32_t first_anchor_point = 0;
    // Transmit window offset
    uint16_t txwinoff = ble_txwinoffset_get(evt_scan->conhdl) << 1;
    // connection interval in slots
    uint16_t con_intv   =  evt_connect->interval;
    // offset of connection in scheduling in slots
    uint16_t con_offset =  (elt_connect->timestamp % con_intv);
    // used to compute time difference
    int32_t  diff_time;
    // Set the linked element pointer to NULL to avoid free
    elt_scan->linked_element = NULL;
    // Stop and clear the scanning element no more useful
    lld_scan_stop(elt_scan);


    // Get the current time
    current_time = ea_time_get_halfslot_rounded();

    // Calculate the end time of the connection request packet
    // Rx sync time + payload header (16 bits) + length + crc (24bits) + IFS (150us) + CONNECT_REQ (352us)
    end_cnx_req_packet_time = evt_scan->anchor_point.finetime_cnt + PAYLOAD_HEADER_LENGTH_US + (ble_rxadvlen_getf(rx_hdl) * BYTE_LENGTH_US)
            + CRC_LENGTH_US + LLD_EVT_IFS_DURATION + CONNECT_REQ_LENGTH_US;

    // Convert in slot the time when end of connect response has been received
    end_cnx_req_packet_time = CLK_ADD_2(evt_scan->anchor_point.basetime_cnt, ((end_cnx_req_packet_time + (SLOT_SIZE-1)) / SLOT_SIZE)-1);

    /*
     * Calculate the first anchor point
     */
    // compute the connection timestamp according to connection request TX time
    first_anchor_point = CLK_ADD_2(end_cnx_req_packet_time,
                                   // remove offset before computing the modulo
                                   ((con_intv) - (CLK_SUB(end_cnx_req_packet_time, con_offset) % (con_intv))));

    // check if TX win offset is similar to computed one, except if tx win offset counter has been reloaded,
    // difference between those counter should be 1.25 ms
    if(CLK_DIFF(end_cnx_req_packet_time, first_anchor_point) < txwinoff)
    {
        // if not use next connection interval
        first_anchor_point = CLK_ADD_2(first_anchor_point, con_intv);
    }

    // compute difference between current time and next anchor point
    diff_time = CLK_DIFF((int32_t)first_anchor_point, (int32_t)current_time);

    // first anchor is in the pasted
    if (diff_time >= 0)
    {
        // compute number of missed event
        uint16_t nb_missed = (diff_time/ con_intv) +1;

        // update anchor point time according to number of missed event
        first_anchor_point = CLK_ADD_2(first_anchor_point, con_intv * nb_missed);
        // compute channel and event counter according to number of missed event
        lld_evt_channel_next(conhdl, nb_missed);
        evt_connect->evt.conn.counter += nb_missed;
    }

    // set the first connection timestamp
    elt_connect->timestamp   = first_anchor_point;

    //Set the interval element value
    new_intv->interval_used  = evt_connect->interval;
    new_intv->bandwidth_used = elt_connect->duration_min/SLOT_SIZE;
    new_intv->conhdl_used    = evt_connect->conhdl;
    new_intv->role_used      = MASTER_ROLE;
    new_intv->offset_used    = con_offset;
    new_intv->odd_offset     = (con_offset  & 0x1) ? true : false;
    new_intv->linkid         = REG_BLE_EM_CS_ADDR_GET(evt_connect->conhdl);

    //Insert the interval element in the list
    ea_interval_insert(new_intv);

    //Save the pointer of the interval in the event environment
    evt_connect->interval_elt = new_intv;
    //Set the MASTER CONNECTED priority
    lld_util_priority_set(elt_connect, RWIP_PRIO_LE_ESTAB_IDX);

    LLD_EVT_FLAG_SET(evt_connect, WAITING_SYNC);


    #if (BLE_AUDIO)
    if(evt_connect->interval <= AUDIO_MIN_INTERVAL_SLOT)
    {
        elt_connect->start_latency = AUDIO_MIN_PROG_LATENCY;
    }
    #endif // (BLE_AUDIO)
    #if (BLE_2MBPS)
    // Init default PHY
    evt_connect->evt.conn.tx_phy = PHYS_1MBPS_PREF;
    evt_connect->evt.conn.rx_phy = PHYS_1MBPS_PREF;
    #endif

    // Get the new element
    return (elt_connect);
}


struct ea_elt_tag *lld_evt_update_create(struct ea_elt_tag *elt_old,
        uint16_t ce_len,
        uint16_t mininterval,
        uint16_t maxinterval,
        uint16_t latency,
        uint8_t  pref_period,
        struct lld_evt_update_tag *upd_par)
{
    // Allocate temporary event structure
    struct ea_elt_tag *elt_new  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated BLE environment
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);


    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param;

    if(elt_new != NULL)
    {
        // Get associated BLE environment
        struct lld_evt_tag *evt_new = LLD_EVT_ENV_ADDR_GET(elt_new);
        // Fill in event properties
        memcpy(elt_new,elt_old,sizeof(struct ea_elt_tag));

        // Initialize BLE event environment
        lld_evt_init_evt(evt_new);

        evt_new->evt.conn.latency    = latency + 1;
        evt_new->mode       = LLD_EVT_MST_MODE;

        input_param.interval_min = mininterval;
        input_param.interval_max = maxinterval;
        input_param.duration_min = input_param.duration_max = ce_len;
        input_param.pref_period  = pref_period;
        input_param.action       = EA_PARAM_REQ_GET;
        input_param.conhdl       = evt_old->conhdl;
        input_param.role         = MASTER_ROLE;
        input_param.linkid       = REG_BLE_EM_CS_ADDR_GET(evt_old->conhdl);
        ea_interval_duration_req(&input_param, &set_param);
        
        input_param.offset       = 0;
        input_param.odd_offset   = 0;

        if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
        {
            evt_new->interval = set_param.interval;
            lld_util_connection_param_set(elt_new, &set_param);
        }
        else
        {
            lld_evt_delete_elt_push(elt_new, true, false);
            elt_new = NULL;
        }

        // Compute the parameters of the connection update
        lld_evt_update_param_compute(elt_old, elt_new, upd_par);

        // Store the new event pointer into the old one, in order to program it at instant
        elt_old->linked_element = elt_new;
        evt_old->evt.conn.instant_action = LLD_UTIL_PARAM_UPDATE;
    }
    // Return pointer to the created event
    return (elt_new);
}

#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
struct ea_elt_tag* lld_evt_move_to_slave(struct llc_create_con_req_ind *con_par,
        struct llm_pdu_con_req_rx *con_req_pdu,
        struct ea_elt_tag *elt_adv,
        uint16_t conhdl)
{
	
	//UART_PRINTF("%s \r\n", __func__);
    // Allocate event structure
    struct ea_elt_tag *elt_connect  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated event
    struct lld_evt_tag *evt_connect = LLD_EVT_ENV_ADDR_GET(elt_connect);
    // Allocate a new interval element
    struct ea_interval_tag *new_intv = ea_interval_create();

    uint16_t winoff_pdu = common_btohs(con_req_pdu->winoffset);

    // Get the time of the CONNECT_REQ reception
    uint32_t basetimecnt = (uint32_t)ble_btcntsync0_get(LLD_ADV_HDL) |
            (((uint32_t)ble_btcntsync1_get(LLD_ADV_HDL)) << 16);
    uint16_t finetimecnt = LLD_EVT_FINECNT_MAX - ble_fcntrxsync_getf(LLD_ADV_HDL);
    uint16_t slot_offset = 0;

    uint8_t mst_sca     = (con_req_pdu->hop_sca >> 5);
    uint8_t winsize_pdu = con_req_pdu->winsize;

    //Get the current filter policy
    con_par->filter_policy = ble_filter_policy_getf(LLD_ADV_HDL);

    if(elt_connect != NULL)
    {
		//UART_PRINTF("elt_connect != NULL\r\n");
        // Initialize BLE event environment
        lld_evt_init_evt(evt_connect);

        // Delete element used for advertising
        lld_evt_delete_elt_push(elt_adv,true, false);

        evt_connect->conhdl                    = conhdl;
        evt_connect->interval                  = con_par->con_int << 1;
        evt_connect->evt.conn.latency          = con_par->con_lat + 1;
        evt_connect->mode                      = LLD_EVT_SLV_MODE;
        evt_connect->evt.conn.mst_sca          = mst_sca;
        evt_connect->evt.conn.duration_dft     = BW_USED_SLAVE_DFT_SLOT;
        evt_connect->evt.conn.wait_con_up_sync = false;

        elt_connect->start_latency  = elt_adv->start_latency;
        elt_connect->duration_min   = BW_USED_SLAVE_DFT_US;

        // Check if window offset (multiple of 1.25ms) is too small (elt->start_latency is in multiple of 625)
        if ((uint32_t)(winoff_pdu << 1) <= elt_connect->start_latency)
        {
            // Move the offset by one interval
            winoff_pdu += con_par->con_int;
            // Update the channel index
            lld_evt_channel_next(evt_connect->conhdl, 1);

            // Update the event counter
            evt_connect->evt.conn.counter = 1;

			//UART_PRINTF("winoff_pdu < elt_connect->start_latency\r\n");
        }
		//UART_PRINTF("winoff_pdu > elt_connect->start_latency\r\n");
        // Compute now the time of the window center (current + 1.25ms + WinOffset + WinSize/2)
        slot_offset = (LLD_EVT_FRAME_DURATION / SLOT_SIZE)
                                           + (winoff_pdu << 1) + ((winsize_pdu << 1) >> 1);
        /*
         * Compute the possible drift to the connection window:
         *     The first anchor point must not be earlier than 1.25ms + transmitWindowOffset,
         *     and no later than 1.25ms + transmitWindowOffset + transmitWindowSize after the
         *     end of the of the CONNECT_REQ packet.
         *     Delay provided in number of slots.
         */
        evt_connect->evt.conn.sca_drift = lld_evt_drift_compute(slot_offset, mst_sca);

        /*
         * Set the window size according to the value received in the connect_req
         * win size = transmit window - default rx_win offset used to receive sync word
         */
        evt_connect->evt.conn.rx_win_off_dft = LLD_EVT_RX_WIN_DFT_OFF_1MBPS;
        evt_connect->evt.conn.rx_win_pathdly_comp = LLD_EVT_RX_WIN_PATHDLY_COMP_1MBPS;
        evt_connect->evt.conn.sync_win_size
                = (((uint32_t)winsize_pdu) * LLD_EVT_FRAME_DURATION) - evt_connect->evt.conn.rx_win_off_dft;
		//evt_connect->evt.conn.win_size_backup   = evt_connect->evt.conn.sync_win_size;
        #if (BLE_2MBPS)
        // Init default PHY
        evt_connect->evt.conn.tx_phy = PHYS_1MBPS_PREF;
        evt_connect->evt.conn.rx_phy = PHYS_1MBPS_PREF;
        #endif

        // Set the value of the window size in the control structure
        lld_evt_winsize_change(evt_connect, false);

        /*-----------------------------------------------------------------------------------
         * Compute the time of first occurrence of the event
         *----------------------------------------------------------------------------------*/

        // Add the duration of the CONNECT_REQ
        finetimecnt += LLD_EVT_CONNECT_REQ_DURATION;

        if (finetimecnt >= SLOT_SIZE)
        {
            // Fine counter has wrapped
            finetimecnt -= SLOT_SIZE;
            // Increment Base time counter
            basetimecnt++;
        }

        // Get the RX time from the control structure
        evt_connect->anchor_point.basetime_cnt = basetimecnt;
        evt_connect->anchor_point.finetime_cnt = finetimecnt;

        /*
         * Trick to avoid losing the 2nd anchor point in case where the 1st is lost,
         * we are saving the virtual last anchor point value.
         */

        evt_connect->cs_ptr         = REG_BLE_EM_CS_ADDR_GET(evt_connect->conhdl);
        elt_connect->ea_cb_start    = lld_evt_schedule;
        elt_connect->ea_cb_cancel   = lld_evt_canceled;
        elt_connect->ea_cb_stop     = lld_evt_prevent_stop;
        
        EA_ASAP_STG_SET(elt_connect, EA_FLAG_ASAP_NO_LIMIT, EA_NO_PARITY, 0, 0, 0);
        //Set the Slave  CONNECTED priority
        lld_util_priority_set(elt_connect, RWIP_PRIO_LE_ESTAB_IDX);

        LLD_EVT_FLAG_SET(evt_connect, WAITING_SYNC);


        // Input parameters
        struct ea_param_input input_param;
        struct ea_param_output set_param = {0,0,0};

        // Compute the scheduling parameters
        input_param.interval_min = evt_connect->interval;
        input_param.interval_max = evt_connect->interval;

        // Slave default BW used
        input_param.duration_min = BW_USED_SLAVE_DFT_SLOT;
        input_param.duration_max = BW_USED_SLAVE_DFT_SLOT;
        input_param.pref_period  = 0;
        input_param.action       = EA_PARAM_REQ_CHECK;
        input_param.conhdl       = evt_connect->conhdl;
        input_param.role         = SLAVE_ROLE;
        input_param.linkid       = REG_BLE_EM_CS_ADDR_GET(evt_connect->conhdl);
        ea_interval_duration_req(&input_param, &set_param);
        
        input_param.offset       = elt_connect->timestamp % evt_connect->interval;
        input_param.odd_offset   = (input_param.offset  & 0x1) ? true : false;

        if(ea_offset_req(&input_param, &set_param) != EA_ERROR_OK)
        {
            input_param.action = EA_PARAM_REQ_GET;

            if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
            {
                // Allocate a LLC_CON_UPD_REQ_IND message
                struct llc_con_upd_req_ind *msg = KERNEL_MSG_ALLOC(LLC_CON_UPD_REQ_IND, KERNEL_BUILD_ID(TASK_LLC, conhdl), TASK_LLD, llc_con_upd_req_ind);

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
        new_intv->offset_used = elt_connect->timestamp % evt_connect->interval;
        new_intv->interval_used = evt_connect->interval;
        // Slave default BW used
        new_intv->bandwidth_used = BW_USED_SLAVE_DFT_SLOT;
        new_intv->conhdl_used = evt_connect->conhdl;
        new_intv->role_used = SLAVE_ROLE;
        new_intv->odd_offset = (new_intv->offset_used  & 0x1) ? true : false;
        new_intv->linkid = REG_BLE_EM_CS_ADDR_GET(evt_connect->conhdl);

        ea_interval_insert(new_intv);
        //Save the pointer of the interval in the event environment
        evt_connect->interval_elt = new_intv;

        // Compute the time of the event according to the parameters
        lld_evt_slave_time_compute(elt_connect, slot_offset);


        #if (BLE_AUDIO)
        if(evt_connect->interval <= AUDIO_MIN_INTERVAL_SLOT)
        {
            elt_connect->start_latency = AUDIO_MIN_PROG_LATENCY;
        }
        #endif // (BLE_AUDIO)

        // Indicate that we are waiting for the initial synchronization with the master
        LLD_EVT_FLAG_SET(evt_connect, WAITING_ACK);
    }
    else
    {
    	//UART_PRINTF("elt_connect == NULL\r\n");
        lld_evt_delete_elt_push(elt_adv,true, false);
    }

    return (elt_connect);
}

void lld_evt_slave_update(struct llcp_con_upd_ind const *param_pdu,
        struct ea_elt_tag *elt_old)
{
	//UART_PRINTF("a\r\n");
    // Allocate a new temporary event for the slave connection
    struct ea_elt_tag *elt_new = NULL;
    // Get BLE event environments
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);

    /**
     * (Instant T0) management
     */
    if(evt_old->evt.conn.counter == param_pdu->instant)
    {
    	//UART_PRINTF("b\r\n");
        // Get LLC environment
        struct llc_env_tag *llc_env_ptr = llc_env[evt_old->conhdl];

        // Retrieve some information from the old event
        if((evt_old->interval != (param_pdu->interv * 2))
                || (evt_old->evt.conn.latency != (param_pdu->latency + 1))
                || (llc_env_ptr->sup_to != llc_env_ptr->n_sup_to))
        {
            SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_EVT_SENT, true);
			//UART_PRINTF("c\r\n");
        }

        // apply new interval and latency, ignore the rest
        evt_old->interval                 = param_pdu->interv * 2;
        evt_old->evt.conn.latency         = param_pdu->latency + 1;

        evt_old->evt.conn.instant           = common_btohs(param_pdu->instant);
        evt_old->evt.conn.instant_action    = LLD_UTIL_PARAM_UPDATE;

        LLD_EVT_FLAG_SET(evt_old, WAITING_ACK);
        // Reset waiting instant flag
        LLD_EVT_FLAG_SET(evt_old, WAITING_INSTANT);
    }
    else
    {
         elt_new = ea_elt_create(sizeof(struct lld_evt_tag));
		 //UART_PRINTF("d\r\n");
    }

    if(elt_new != NULL)
    {
    	//UART_PRINTF("e\r\n");
        // Get BLE event environments
        struct lld_evt_tag *evt_new     = LLD_EVT_ENV_ADDR_GET(elt_new);

        // Fill in event properties
        memcpy(elt_new,elt_old,sizeof(struct ea_elt_tag));

        // Initialize BLE event environment
        lld_evt_init_evt(evt_new);

        // Fill in event properties
        evt_new->conhdl                   = evt_old->conhdl;
        evt_new->interval                 = param_pdu->interv * 2;
        evt_new->evt.conn.latency         = param_pdu->latency + 1;
        evt_new->mode                     = LLD_EVT_SLV_MODE;
        evt_new->evt.conn.mst_sca         = evt_old->evt.conn.mst_sca;
        evt_new->evt.conn.duration_dft    = evt_old->evt.conn.duration_dft;

        LLD_EVT_FLAG_SET(evt_new, WAITING_ACK);

        elt_new->duration_min = 0xFFFF;

        // Compute the possible drift to the update window
        evt_new->evt.conn.sca_drift = lld_evt_drift_compute((common_btohs(param_pdu->win_off) +
                param_pdu->win_size) * 2 + evt_old->interval, evt_new->evt.conn.mst_sca);

        // Set the half connect window size according to the value received in the connect_req
        evt_new->evt.conn.sync_win_size = ((uint32_t)param_pdu->win_size * LLD_EVT_FRAME_DURATION) - evt_old->evt.conn.rx_win_off_dft;
		//evt_new->evt.conn.win_size_backup   = evt_new->evt.conn.sync_win_size;
        // Store the new event in the old one while waiting for instant
        elt_old->linked_element             = elt_new;
        evt_old->evt.conn.instant           = common_btohs(param_pdu->instant);
        evt_old->evt.conn.instant_action    = LLD_UTIL_PARAM_UPDATE;
        evt_old->evt.conn.update_offset     = common_btohs(param_pdu->win_off);
        evt_old->evt.conn.update_size       = param_pdu->win_size;

        LLD_EVT_FLAG_SET(evt_old, WAITING_ACK);

        /**
         * (Instant T0-1) management
         */
        if (evt_old->evt.conn.counter == ((param_pdu->instant - 1) & 0xFFFF))
        {
            // Reset waiting instant flag
            LLD_EVT_FLAG_SET(evt_old, WAITING_INSTANT);
			// UART_PRINTF("f\r\n");
        }
    }
}
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

struct ea_elt_tag *lld_evt_adv_create(uint16_t handle, uint16_t mininterval, uint16_t maxinterval, bool restart_pol)
{
	
	//UART_PRINTF("lld_evt_adv_create\r\n");
    // Allocate event structure
    struct ea_elt_tag *elt  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Sanity check: There should always be a free event
    ASSERT_ERR(elt != NULL);

    // Initialize BLE event environment
    lld_evt_init_evt(evt);

    evt->conhdl       = handle;
    evt->interval     = maxinterval;
    evt->mode         = LLD_EVT_ADV_MODE;
    evt->rx_cnt       = 0;
    // The duration of an ADV event if the 3 channel are used is ~ 6 slots
    // (adv_int)1500us + (adv_int)1500us + (adv_dur_max) 376 us) )
    elt->duration_min       = BW_USED_ADV_DFT_US;
    evt->cs_ptr             = REG_BLE_EM_CS_ADDR_GET(handle);

    //Set the ADVERTISING priority
    lld_util_priority_set(elt, RWIP_PRIO_ADV_IDX);


    elt->start_latency         = RWBLE_PROG_LATENCY_DFT;
    elt->stop_latency1         = 0;
    elt->stop_latency2         = 0;
    elt->ea_cb_start           = lld_evt_schedule;
    elt->ea_cb_cancel          = lld_evt_canceled;
    elt->ea_cb_stop            = lld_evt_prevent_stop;
    EA_ASAP_STG_SET(elt, EA_FLAG_ASAP_NO_LIMIT, EA_NO_PARITY, 0, 0, 0);

    if (!restart_pol)
    {
        LLD_EVT_FLAG_SET(evt, NO_RESTART);
    }
    // Schedule event as soon as possible
    // Compute the start instant by taking a delay to be able to program it
    elt->timestamp = (ea_time_get_halfslot_rounded() + 20*LLD_EVT_START_MARGIN) & BLE_BASETIMECNT_MASK ;
	//UART_PRINTF("elt->timestamp  = 0x%x\r\n",elt->timestamp);
		
    // Return pointer to the created event
    return (elt);
}

void lld_evt_deffered_elt_handler(void)
{
	
    // Deferred element
    struct ea_elt_tag *elt;
    // Indicate on which interrupt treatment has been postponed (RX ISR or END OF EVENT ISR)
    uint8_t type;
    // Number of RX descriptors consumed
    uint8_t rx_desc_cnt;
    //Status of the scan element dedicated for the initiating
    bool elt_deleted = false;
    // Clear kernel event
    kernel_event_clear(KERNEL_EVENT_BLE_EVT_DEFER);

    // First deferred element
    elt = lld_evt_deferred_elt_pop(&type, &rx_desc_cnt);

    while (elt)
    {
        // Get associated BLE event
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        if(type == LLD_DEFER_CON_UP_INSTANT)
        {
            // Warn the LLC about the connection parameter update
            llc_con_update_ind(evt->conhdl, elt);

            llc_con_update_finished(evt->conhdl);
        }
        else if(type == LLD_DEFER_MAP_UP_INSTANT)
        {
            llc_map_update_finished(evt->conhdl);

        }
        #if (BLE_2MBPS)
        else if(type == LLD_DEFER_PHY_UP_INSTANT)
        {
            struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), llc_phy_upd_req_ind);
            phy_update->operation   = LLC_PHY_UPD_TERMINATE;
            phy_update->rx_phys     =  evt->evt.conn.rx_phy;
            phy_update->tx_phys     = evt->evt.conn.tx_phy;
            phy_update->status      = COMMON_ERROR_NO_ERROR;
            kernel_msg_send(phy_update);
        }
        #endif // (BLE_2MBPS)
        else
        #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
        if ((type == LLD_DEFER_END) && LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
        {
            lld_evt_delete_elt_push(elt, true, true);
        }
        else if(type == LLD_DEFER_TEST_END)
        {
            GLOBAL_INT_DIS();
            // Free the buffer here to avoid sending a number of completed packet later (in lld_evt_elt_delete function)
            if(!common_list_is_empty(&evt->tx_prog))
            {
                em_buf_tx_free((struct em_desc_node *)common_list_pop_front(&evt->tx_prog));
            }
            if(!common_list_is_empty(&evt->tx_acl_rdy))
            {
                em_buf_tx_free((struct em_desc_node *)common_list_pop_front(&evt->tx_acl_rdy));
            }
            GLOBAL_INT_RES();

            // not flush data, already done above
            lld_evt_delete_elt_push(elt,false, false);
        }

        // Check received and transmitted data
        elt_deleted = lld_pdu_check(evt);

        if (((type == LLD_DEFER_END) || (type == LLD_DEFER_TEST_END)) && (!elt_deleted))
        {
            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            if(evt->conhdl != LLD_ADV_HDL)
            {
                llc_end_evt_defer(evt->conhdl);
            }
            else
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
            #if (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)
            {
                llm_end_evt_defer();
            }
            #endif // (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)
        }

            // Get next element
        elt = lld_evt_deferred_elt_pop(&type, &rx_desc_cnt);
    }
}

void lld_evt_end(struct ea_elt_tag *elt)
{
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    #if(HW_AUDIO)
    // Read total number of RX descriptor burnt during the event
    uint8_t rx_desc_cnt = ble_aclrxdesccnt_getf(evt->conhdl);
    rx_desc_cnt += ble_isorxpktcntl_getf(evt->conhdl);
    #else
    // Read total number of RX descriptor burnt during the event
    uint8_t rx_desc_cnt = ble_aclrxdesccnt_getf(evt->conhdl);//ble_rxdesccnt_getf(evt->conhdl);
    #endif
    // Read number of tx descriptor used
    #if(HW_AUDIO)
    uint8_t tx_desc_cnt = ble_acltxdesccnt_getf(evt->conhdl);
    #else  // !(HW_AUDIO)
    uint8_t tx_desc_cnt = ble_acltxdesccnt_getf(evt->conhdl); //ble_txdesccnt_getf(evt->conhdl);
    #endif // (HW_AUDIO)
    // Defer element status
    bool defer_elt = true;
    // Defer element status
    bool try_to_insert_elt = true;
	  
    // Check if a conflict has been detected and the event has not been started
    if(ble_conflict_getf(evt->conhdl) && (!rx_desc_cnt))
    {
        defer_elt = false;
	} 
    else
	{
        // Check if the End of Event comes from APFM (smooth or immediate) abort or a normal End of Evt
        // ignore event that shall be deleted
        if (!LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
        {
            if(LLD_EVT_FLAG_GET(evt, APFM))
            {
                // Reset the flag
                LLD_EVT_FLAG_RESET(evt, APFM);
            }
            else
            {
                // If no APFM issue, reset the priority with the default one
                lld_evt_priority_reset(elt);
            }
			//UART_PRINTF("rx_desc_cnt = %x\r\n",rx_desc_cnt);
            if (!rx_desc_cnt)
            {
								
                // Check if the event can be restarted and no RX pending
                if(lld_evt_continue(elt))
                {
                    defer_elt = false;
                    try_to_insert_elt = false;
                }
            }
        }
    }

    // Check if the tx/rx deferring is authorized
    if(defer_elt)
    {
        lld_pdu_rx_handler(evt, rx_desc_cnt - evt->rx_cnt);

        if(evt->mode != LLD_EVT_TEST_MODE)
        {
            ASSERT_INFO(rx_desc_cnt >= evt->rx_cnt, rx_desc_cnt, evt->rx_cnt);
            // Push the element in the Tx / Rx pending list
            lld_evt_deferred_elt_push(elt, LLD_DEFER_END, (rx_desc_cnt - evt->rx_cnt));
            // Check the MD bit value to restart the event with or without latency
            lld_evt_check_md_bit(evt,rx_desc_cnt);
        }
        else
        {
            // Push the element in the Tx / Rx pending list
            lld_evt_deferred_elt_push(elt, LLD_DEFER_TEST_END, (LLD_RX_IRQ_THRES > 0) ? (rx_desc_cnt % LLD_RX_IRQ_THRES) : 0);
        }

        if((evt->mode == LLD_EVT_SCAN_MODE) && (evt->evt.non_conn.initiate) && (tx_desc_cnt != 0))
        {
            evt->evt.non_conn.connect_req_sent = true;
        }

        // Reset RX counter stored in the event environment
        evt->rx_cnt = 0;

        // Defer the TX and RX handling
        kernel_event_set(KERNEL_EVENT_BLE_EVT_DEFER);
    }

    if (!LLD_EVT_FLAG_GET(evt, NO_RESTART) &&
            !LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE) && (try_to_insert_elt))
    {
        // Restart the event
        bool evt_restarted = lld_evt_restart(elt, false);

        if (evt_restarted)
        {
            lld_evt_elt_insert(elt, false);
        }
    }
    // if element deferred, do not delete it immediately.
    else if(LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE) && (!defer_elt))
    {
        lld_evt_delete_elt_push(elt, true, true);
    }
}

void lld_evt_rx(struct ea_elt_tag *elt)
{
    // Linked event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Read RX threshold set in the CS
    uint8_t rx_thr = ble_rxthr_getf(evt->conhdl);

    lld_pdu_rx_handler(evt, rx_thr);

    // Update number of rx packet in the event
    evt->rx_cnt += rx_thr;

    // Push the element in the Tx / Rx pending list
    lld_evt_deferred_elt_push(elt, LLD_DEFER_RX, rx_thr);
    // Defer the RX handling
    kernel_event_set(KERNEL_EVENT_BLE_EVT_DEFER);
}

void lld_evt_canceled(struct ea_elt_tag *elt)
{
	//UART_PRINTF("lld_evt_canceled ");
    struct lld_evt_wait_tag *elt_to_free = lld_evt_elt_wait_get(elt);
    if(elt_to_free)
    {
        kernel_free(elt_to_free);
    }
    // Increment element priority
    if (elt->current_prio < RWIP_PRIO_MAX)
    {
        elt->current_prio++;
    }

    // Restart the event
    if(lld_evt_restart(elt, true))
    {
        // Check if the element should be stopped and freed
        lld_evt_elt_insert(elt, true);
		//UART_PRINTF("aaaaaa \r\n");
    }
    else
    {
        // Push the element in the Tx / Rx pending list
        lld_evt_deferred_elt_push(elt, LLD_DEFER_END, 0);
        // Defer the TX and RX handling
        kernel_event_set(KERNEL_EVENT_BLE_EVT_DEFER);
    }
}

#if (!BT_DUAL_MODE)
void lld_evt_timer_isr(void)
{
    // Set kernel event for deferred handling
    kernel_event_set(KERNEL_EVENT_KERNEL_TIMER);
}
#endif //(!BT_DUAL_MODE)

void lld_evt_end_isr(bool apfm)
{
    // Pop the element in the programmed environment queue
    struct ea_elt_tag *elt = (struct ea_elt_tag *)common_list_pop_front(&lld_evt_env.elt_prog);
    // Get the associated BLE event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Pop the element from the EA
    ea_elt_remove(elt);

    if (apfm)
    {
        // Report the APFM error in element internal status
        LLD_EVT_FLAG_SET(evt, APFM);
    }
    else
    {
        // Clear ASAP flag if needed
        if(EA_ASAP_STG_TYPE_GET(elt) != EA_FLAG_NO_ASAP)
        {
            elt->asap_settings = 0;
        }
    }

    // Call back the end of event function
    lld_evt_end(elt);
}

void lld_evt_rx_isr(void)
{
    // Get the current event programmed
    struct ea_elt_tag *elt = (struct ea_elt_tag *)common_list_pick(&lld_evt_env.elt_prog);
    if(elt)
    {
        lld_evt_rx(elt);
    }
}

/// @} LLDEVT

