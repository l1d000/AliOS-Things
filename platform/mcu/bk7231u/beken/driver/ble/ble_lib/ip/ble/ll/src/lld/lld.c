/**
 ****************************************************************************************
 *
 * @file lld.c
 *
 * @brief Definition of the functions used by the logical link driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "ea.h"
#include "llm.h"
#include "llm_util.h"
#include "llc_llcp.h"
#include "llcontrl.h"
#include "lld.h"

#include "common_endian.h"
#include "common_utils.h"
#include "kernel_event.h"
#include "lld_evt.h"
#include "lld_pdu.h"
#include "lld_util.h"
#include "reg_blecore.h"
#include "reg_ble_em_wpb.h"
#include "reg_ble_em_wpv.h"
#include "reg_ble_em_ral.h"
#include "reg_ble_em_cs.h"
#include "reg_common_em_et.h"
#include "rwip.h"
#include "rwble.h"
#if (SECURE_CONNECTIONS)
#include "ecc_p256.h"
#endif // (SECURE_CONNECTIONS)

#if (DEEP_SLEEP)
#if (!BT_DUAL_MODE)
#include "lld_sleep.h"
#include "RomCallFlash.h"
#endif //(!BT_DUAL_MODE)
#endif //(DEEP_SLEEP)

#if (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
#include "lld_wlcoex.h"         // WLAN/MWS coexistence definitions
#endif //(RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)

#if (NVDS_SUPPORT)
#include "nvds.h"               // NVDS definitions
#endif //(NVDS_SUPPORT)
#if(BLE_AUDIO)
#include "audio.h"
#endif
#include "dbg.h"
#include "bk7011_cal_pub.h"

/*
 * DEFINES
 ****************************************************************************************
 */


// Define in slot number the High Duty Cycle directed advertising duration 1.28s = 2048 slots(625us)
#define ADV_HDC_DURATION    (2048)
// Define in slot number the Low Duty Cycle event duration 1.5ms * 3 = 8 slots (625us)
#define ADV_LDC_DURATION    (8)

/**
 * Margin to match with the requierment in the spec v4.1 :
 * "The ReferenceConnEventCount field shall be set to indicate that at least
 * one of the Offset0 to Offset5 fields is valid. If the ReferenceConnEvent-
 * Count field is set, then it shall always be set to the connEventCounter of
 * a connection event that is less than 32767 connection events in the
 * future from the first transmission of the PDU."

 */
#define REF_COUNTER_MARGIN  (2)
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// LLD task descriptor. LLD is a dummy task that is defined just for its message API
static const struct kernel_task_desc TASK_DESC_LLD = {NULL, NULL, NULL, 1, 1};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BT_DUAL_MODE)
/**
 ****************************************************************************************
 * @brief Initialization of the DM priority
 *
 * @param[in] step    Increment of priority
 * @param[in] init    Default priority
 *
 ****************************************************************************************
 */
__INLINE void lld_prio_init(int idx, uint8_t step, uint8_t init, uint8_t current)
{
    ble_dmpriocntl_pack(idx, step, init, 0, current);
}
#endif //(BT_DUAL_MODE)

void lld_init(bool reset)
{
    int i;
    struct bd_addr bd_addr;

    #if (NVDS_SUPPORT)
    //uint8_t length = NVDS_LEN_BD_ADDRESS;

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (SECURE_CONNECTIONS)
    uint8_t p256_sk[32];
    uint8_t sk_len = NVDS_LEN_LE_PRIVATE_KEY_P256;
    #endif // (SECURE_CONNECTIONS)
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
    #endif //(NVDS_SUPPORT)

    #if (RW_DEBUG)
    {
        // Debug only, trash all EM with invalid data.
        uint8_t *ptr =  (uint8_t *)(EM_BASE_ADDR + EM_BLE_OFFSET);
        memset(ptr,0xFF,EM_BLE_END - EM_BLE_OFFSET);
    }
    #endif // (RW_DEBUG)
    
    if(! reset)
    {
        // Create LLD task. LLD is a dummy task that is defined just for its message API
        kernel_task_create(TASK_LLD, &TASK_DESC_LLD);
    }

    // Set default window size
    ble_rxwinszdef_setf(LLD_EVT_DEFAULT_RX_WIN_SIZE);

    // Reduce prefetch time to 150us
    ble_prefetch_time_setf(LLD_EVT_PREFETCH_TIME_US);

    // Set the prefetch abort time
    ble_prefetchabort_time_setf(LLD_EVT_ABORT_CNT_DURATION_US);
#if 0	
		ble_radiopwrupdn0_set(0x0770006E); //0x077A0078  0x07750078
		ble_radiopwrupdn1_set(0x077A0078);
#else

		ble_radiopwrupdn0_set(0x077A0078); //0x077A0078  0x07750078
		ble_radiopwrupdn1_set(0x077A0078);
#endif
    // All interrupts enabled except CSNT and SLP
    ble_intcntl_set(BLE_RXINTMSK_BIT | BLE_EVENTINTMSK_BIT | BLE_EVENTAPFAINTMSK_BIT |
            BLE_CRYPTINTMSK_BIT /*| BLE_ERRORINTMSK_BIT */| BLE_SWINTMSK_BIT);

    ble_sn_dsb_setf(0);
    ble_nesn_dsb_setf(0);

    // Initialize BD address
    #if 0//(NVDS_SUPPORT)
    if (nvds_get(NVDS_TAG_BD_ADDRESS, &length, (uint8_t *)&bd_addr) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        memcpy(&bd_addr, &common_default_bdaddr, sizeof(common_default_bdaddr));
    }

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (SECURE_CONNECTIONS)
    #if (NVDS_SUPPORT)
    if (nvds_get(NVDS_TAG_LE_PRIVATE_KEY_P256, &sk_len, (uint8_t *)&p256_sk) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        // If Secret Key not in NVRAM - then generate a new Secret Key and store in NVRAM.
        ecc_gen_new_secret_key(&llm_le_env.secret_key256[0], false);
        #if (NVDS_SUPPORT)
        if (nvds_put(NVDS_TAG_LE_PRIVATE_KEY_P256, NVDS_LEN_LE_PRIVATE_KEY_P256, &llm_le_env.secret_key256[0]) != NVDS_OK)
        {
            ASSERT_ERR(0);// Could not write new secret key to NVDS. Assert added for debug.
        }
        #endif //(NVDS_SUPPORT)
    }
    #if (NVDS_SUPPORT)
    else
    {
        // Copy the Secret Key read from NVRAM to LLM.
        memcpy(&llm_le_env.secret_key256[0],&p256_sk[0], NVDS_LEN_LE_PRIVATE_KEY_P256);
    }
    #endif // (NVDS_SUPPORT)
    #endif // (SECURE_CONNECTIONS)
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    // Set the BDADDR from the configuration structure
    ble_bdaddrl_set(common_read32p(&bd_addr.addr[0]));
    ble_bdaddru_set(common_read16p(&bd_addr.addr[4]));

    ble_advertfilt_en_setf(1);
    ble_advchmap_setf(BLE_ADVCHMAP_RESET);

    // Set white list addresses and count
    ble_wlpubaddptr_set(REG_BLE_EM_WPB_ADDR_GET(0));
    ble_wlprivaddptr_set(REG_BLE_EM_WPV_ADDR_GET(0));
    ble_wlnbdev_pack(BLE_WHITELIST_MAX, BLE_WHITELIST_MAX);

    ble_ralptr_set(REG_BLE_EM_RAL_ADDR_GET(0));
    ble_nbraldev_setf(BLE_RESOL_ADDR_LIST_MAX);


    // Enable dual-mode arbitration module
    #if (BT_DUAL_MODE)
    ble_bleprioscharb_pack(1, DM_ARB_MARGIN);
    #endif //(BT_DUAL_MODE)

    // Initialize the advertising control structure
    ble_syncwl_set(LLD_ADV_HDL, 0xBED6);
    ble_syncwh_set(LLD_ADV_HDL, 0x8E89);

    // Initialize the crcinit in the CS
    ble_crcinit0_set(LLD_ADV_HDL, 0x5555);
    ble_crcinit1_setf(LLD_ADV_HDL,0x55);

    // Initialize the channel map in the CS
    ble_chmap0_set(LLD_ADV_HDL,0x0000);
    ble_chmap1_set(LLD_ADV_HDL,0x0000);
    ble_chmap2_set(LLD_ADV_HDL,0x0000);

    // Initialize Rx Max Buf and Rx Max time in the CS
    ble_rxmaxbuf_set(LLD_ADV_HDL,0x0000);
    ble_rxmaxtime_set(LLD_ADV_HDL,0x0000);

    // Set the first RX descriptor pointer into the HW and the ET base ptr to 0
    ble_et_currentrxdescptr_set(REG_BLE_EM_RX_DESC_ADDR_GET(0) & BLE_CURRENTRXDESCPTR_MASK);

    // Initialize the txrx cntl in the CS
    ble_txrxcntl_set(LLD_ADV_HDL, LLM_ADV_CHANNEL_TXPWR);
    // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
    ble_thrcntl_ratecntl_pack(LLD_ADV_HDL, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);


    //Init conflict bit
    ble_conflict_setf(LLD_ADV_HDL, 0);
    //Clear RAL settings
    ble_filtpol_ralcntl_set(LLD_ADV_HDL, 0);

    #if (HW_AUDIO)
    // Init Synch found counter and event status
    ble_btcntsync0_set(LLD_ADV_HDL, 0);
    ble_btcntsync1_set(LLD_ADV_HDL, 0);
    ble_fcntsync_set(LLD_ADV_HDL, 0);
    //Init rx and tx counters for ACL and ISO
    ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
    ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
    #endif // (HW_AUDIO)

    // Clear the exchange table
    for (i = 0 ; i < EM_EXCH_TABLE_LEN; i++)
    {
        em_common_extab0_set(i, 0);
        em_common_extab1_set(i, 0);
    }

    // Initialize the event scheduler
    lld_evt_init(reset);

    #if (DEEP_SLEEP)
    #if (!BT_DUAL_MODE)
    // Initialize sleep manager
    lld_sleep_init();
	//	rom_env.lld_sleep_init();
    #endif //(!BT_DUAL_MODE)
    #endif //(DEEP_SLEEP)

    #if (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    ble_dnabort_setf(LLD_ADV_HDL,0);
    ble_txbsy_en_setf(LLD_ADV_HDL,0);
    ble_rxbsy_en_setf(LLD_ADV_HDL,0);
    #if (RW_BLE_WLAN_COEX)
    // Enable WL coexistence by default
    lld_wlcoex_set(BLECOEX_ENABLED);
    #endif //(RW_BLE_WLAN_COEX)
    #if (RW_BLE_MWS_COEX)
    // Enable WL coexistence by default
    lld_mwscoex_set(BLECOEX_ENABLED);
    #endif //(RW_BLE_MWS_COEX)
    #endif // (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    
    // Disable PRBS generator
    ble_txpldsrc_setf(0);

    // Enable the BLE core
    ble_rwble_en_setf(1);
}

void lld_core_reset(void)
{
    int i;

    ASSERT_INFO((ble_version_get() == BLE_VERSION_RESET), BLE_VERSION_RESET, ble_version_get());

    // Disable the BLE core
    ble_rwble_en_setf(0);

    // Reset the BLE state machines
    ble_master_soft_rst_setf(1);
    while(ble_master_soft_rst_getf());

    #if (!BT_DUAL_MODE)
    // Reset the timing generator
    ble_master_tgsoft_rst_setf(1);
    while(ble_master_tgsoft_rst_getf());
    #endif //!BT_DUAL_MODE

    // Clear the exchange table to prevent any further BLE processing
    for (i=0; i < EM_EXCH_TABLE_LEN; i++)
    {
        em_common_extab0_set(i, 0);
        em_common_extab1_set(i, 0);
    }

    // Disable all the BLE interrupts
    ble_intcntl_set(0);

    // And acknowledge any possible pending ones
    ble_intack_clear(0xFFFFFFFF);
}

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
struct ea_elt_tag* lld_adv_start(struct advertising_pdu_params *adv_par,
        struct em_desc_node *adv_pdu,
        struct em_desc_node *scan_rsp_pdu,
        uint8_t adv_pwr)
{
    struct ea_elt_tag *elt = NULL;
    struct lld_evt_tag *evt = NULL;
    uint32_t adv_int;
    bool restart_pol;


    // Check the advertising type to put the correct restart policy (high duty cycle
    // directed advertising is programmed only once)
    if ((adv_par->type == LL_ADV_CONN_DIR) && (!adv_par->adv_ldc_flag))
    {
        // For high duty cycle interval max is set to 1.28s = 2048 * 625
        // needed by EvtAPFM ISR to determine if the evt should be restarted
        adv_par->intervalmax = ADV_HDC_DURATION;
        restart_pol = false;
        adv_int = 1250;
        #if (BT_DUAL_MODE)
        // Set the priority properties
        lld_prio_init(LLD_ADV_HDL, rwip_priority[RWIP_PRIO_ADV_HDC_IDX].increment, rwip_priority[RWIP_PRIO_ADV_HDC_IDX].value, rwip_priority[RWIP_PRIO_ADV_HDC_IDX].value);
        #endif //(BT_DUAL_MODE)
    }
    else
    {
        restart_pol = true;
        adv_int = 1500;
        #if (BT_DUAL_MODE)
        // Set the priority properties
        lld_prio_init(LLD_ADV_HDL, rwip_priority[RWIP_PRIO_ADV_IDX].increment, rwip_priority[RWIP_PRIO_ADV_IDX].value,rwip_priority[RWIP_PRIO_ADV_IDX].value);
        #endif //(BT_DUAL_MODE)
    }

    // Create an event to handle the advertising
    elt = lld_evt_adv_create(LLD_ADV_HDL, adv_par->intervalmin, adv_par->intervalmax, restart_pol);

    if(elt != NULL)
    {
        bool ral_en = false;
        uint16_t ral_ptr = 0;

        // Get the associated BLE specific event environment
        evt = LLD_EVT_ENV_ADDR_GET(elt);


        if(GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE))
        {
            // retrieve if peer device is present in resolving list
            if(adv_par->own_addr_type & ADDR_RPA_MASK)
            {
                uint8_t ral_idx;
                // retrieve set ral_ptr
                if(llm_util_bd_addr_in_ral(&(adv_par->peer_addr), adv_par->peer_addr_type, &ral_idx))
                {
                    ral_ptr = REG_BLE_EM_RAL_ADDR_GET(ral_idx);
                }
            }

            // Resolving list has to be disabled when both peer and local
            // address are not RPA and ADV direct requested
            if((adv_par->own_addr_type & ADDR_RPA_MASK) != 0)
            {
                // for an ADV Direct, ensure that ral pointer is set else disable RAL mechanism
                if((adv_par->type != LL_ADV_CONN_DIR) || (ral_ptr != 0))
                {
                    ral_en = true;
                }
            }
        }

        ble_linklbl_setf(LLD_ADV_HDL,(uint8_t)(LLD_ADV_HDL & BLE_LINKLBL_MASK));
        // Update the control structure according to the parameters
        ble_cntl_pack(LLD_ADV_HDL, RW_BLE_PTI_PRIO_AUTO, RWIP_COEX_GET(ADV, TXBSY), RWIP_COEX_GET(ADV, RXBSY), RWIP_COEX_GET(ADV, DNABORT), ((adv_par->type == LL_ADV_CONN_DIR) && (!adv_par->adv_ldc_flag)) ? LLD_HD_ADVERTISER : LLD_LD_ADVERTISER);
        // update Resolving Address List mechanism configuration
        ble_filtpol_ralcntl_pack(LLD_ADV_HDL,
                                 /* FILTER_POLICY */  adv_par->filterpolicy,
                                 /* LOCAL_RPA_SEL */ ((adv_par->own_addr_type > ADDR_RAND) && (ral_ptr != 0)),
                                 /* RAL_MODE */      (ral_ptr != 0),
                                 /* RAL_EN */        ral_en);
        //Init conflict bit
        ble_conflict_setf(LLD_ADV_HDL, 0);

        #if (HW_AUDIO)
        // Init Synch found counter and event status
        ble_btcntsync0_set(LLD_ADV_HDL, 0);
        ble_btcntsync1_set(LLD_ADV_HDL, 0);
        ble_fcntsync_set(LLD_ADV_HDL, 0);
        //Init rx and tx counters for ACL and ISO
        ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
        ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
        #endif // (HW_AUDIO)

        // Set peer device configuration
        if(ral_ptr != 0)
        {
            // set ral_ptr
            ble_peer_ralptr_set(LLD_ADV_HDL, ral_ptr);
        }
        else
        {
            // set peer address
            // Do a burst write in the exchange memory
            em_wr(&(adv_par->peer_addr), REG_BLE_EM_CS_ADDR_GET(LLD_ADV_HDL) + BLE_ADV_BD_ADDR_INDEX*2, sizeof(struct bd_addr));
            ble_adv_bd_addr_type_setf(LLD_ADV_HDL, adv_par->peer_addr_type);
        }
				
			

        ble_hopcntl_set(LLD_ADV_HDL, BLE_FH_EN_BIT | 39);
        ble_crcinit1_set(LLD_ADV_HDL, 0x55);
        ble_rxwincntl_set(LLD_ADV_HDL, 0);
        ble_txrxcntl_set(LLD_ADV_HDL, adv_pwr);
        // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
        ble_thrcntl_ratecntl_pack(LLD_ADV_HDL, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);
        ble_fcntoffset_set(LLD_ADV_HDL,0);

        // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
        ble_rxmaxbuf_set(LLD_ADV_HDL,0x0);
        ble_rxmaxtime_set(LLD_ADV_HDL,0x0);

        // set event time to support 3 advs
        ble_maxevtime_set(LLD_ADV_HDL, ADV_LDC_DURATION);

        // Set the advertising channel map
        ble_advchmap_set(adv_par->channelmap);
        // Set advertising timing register
        ble_advtim_set(adv_int);

        // Update the TX Power in the event
        evt->tx_pwr = adv_pwr;

        // Chain the advertising data into the control structure
        lld_pdu_tx_push(elt, adv_pdu);

        // Chain the scan response data into the control structure
        if (scan_rsp_pdu != NULL)
        {
            lld_pdu_tx_push(elt, scan_rsp_pdu);
        }

        // Loop the advertising data
        lld_pdu_tx_loop(evt);

        GLOBAL_INT_DIS();
        // Schedule the event
        lld_evt_elt_insert(elt, true);
				

        // store the allocated timestamp in event anchor
        evt->evt.non_conn.anchor             = elt->timestamp;
        // calculate end of event
        evt->evt.non_conn.end_ts             = (evt->evt.non_conn.anchor + ADV_HDC_DURATION) & BLE_BASETIMECNT_MASK;
        // not an initiate procedure
        evt->evt.non_conn.initiate           = false;
        evt->evt.non_conn.connect_req_sent   = false;
        GLOBAL_INT_RES();
    }

    return (elt);
}

void lld_adv_stop(struct ea_elt_tag *elt)
{
    //Check if the elt will be programmed soon or already programmed
    if(!lld_evt_elt_delete(elt, true, true))
    {
        // Get the associated BLE specific event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

        LLD_EVT_FLAG_SET(evt, WAITING_EOEVT_TO_DELETE);
        // Abort the event
        ble_rwblecntl_set(ble_rwblecntl_get() | BLE_ADVERT_ABORT_BIT);
    }
}
#endif //(BLE_PERIPHERAL || BLE_BROADCASTER)

#if (BLE_CENTRAL || BLE_OBSERVER)
struct ea_elt_tag *lld_scan_start(struct scanning_pdu_params const *scan_par,
        struct em_desc_node *scan_req_pdu)
{
    struct ea_elt_tag *elt = NULL;
    struct lld_evt_tag *evt = NULL;
    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param ={0,0,0};

    ASSERT_ERR(scan_par->window != 0);

    // Create an event to handle the connection
    elt = lld_evt_scan_create(LLD_ADV_HDL, 0);

    if(elt != NULL)
    {
        input_param.action          = EA_PARAM_REQ_GET;
        input_param.interval_max    = scan_par->interval;
        input_param.interval_min    = scan_par->interval;
        input_param.duration_max    = scan_par->window;
        input_param.duration_min    = scan_par->window;
        input_param.pref_period     = 0;
        input_param.conhdl          = LLD_ADV_HDL;
        input_param.role            = UNKNOWN_ROLE;
        input_param.linkid          = REG_BLE_EM_CS_ADDR_GET(LLD_ADV_HDL);
        ea_interval_duration_req(&input_param, &set_param);
        
        input_param.offset          = 0;
        input_param.odd_offset      = false;

        if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
        {
            evt = LLD_EVT_ENV_ADDR_GET(elt);
            set_param.interval = input_param.interval_max;
            set_param.duration = input_param.duration_min;
            evt->interval      = set_param.interval;

            lld_util_connection_param_set(elt, &set_param);

            ble_linklbl_setf(LLD_ADV_HDL,(uint8_t)(LLD_ADV_HDL & BLE_LINKLBL_MASK));

            // Update the control structure according to the parameters
            ble_cntl_pack(LLD_ADV_HDL,RW_BLE_PTI_PRIO_AUTO, RWIP_COEX_GET(SCAN, TXBSY), RWIP_COEX_GET(SCAN, RXBSY),RWIP_COEX_GET(SCAN, DNABORT), (scan_req_pdu ? LLD_ACTIVE_SCANNING : LLD_PASSIVE_SCANNING));

            // update Resolving Address List mechanism configuration
            ble_filtpol_ralcntl_pack(LLD_ADV_HDL,
                                     /* FILTER_POLICY */ scan_par->filterpolicy,
                                     /* LOCAL_RPA_SEL */ (scan_par->own_addr_type > ADDR_RAND),
                                     /* RAL_MODE */      false,
                                     /* RAL_EN */        GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE));

            //ble_hopcntl_set(LLD_ADV_HDL, BLE_FH_EN_BIT | 39);
            // add by zhangheng to config channel by upper layer
            {
                switch (scan_par->channel_map)
                {
                    case 1:
                        ble_hopcntl_set(LLD_ADV_HDL, 37);
                        break;
                    case 2:
                        ble_hopcntl_set(LLD_ADV_HDL, 38);
                        break;
                    case 4:
                        ble_hopcntl_set(LLD_ADV_HDL, 39);
                        break;
                    default:
                        ble_hopcntl_set(LLD_ADV_HDL, BLE_FH_EN_BIT | 39);
                        break;
                }
                bk_printf("LLD_ADV_HDL=0x%x\n", ble_hopcntl_get(LLD_ADV_HDL));
            }
            ble_crcinit1_set(LLD_ADV_HDL, 0x55);
            // When the parameters are set in the CS, set the default value in the environment to ease the EA to find a place
            // Like the Scan can be aborted by other activity we try to insert it if the default BW is free.
            elt->duration_min  = EA_BW_USED_DFT_US;
            // Store Scan duration in sync window size
            evt->evt.non_conn.window = set_param.duration;
            // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
            ble_rxmaxbuf_set(LLD_ADV_HDL,0x0);
            ble_rxmaxtime_set(LLD_ADV_HDL,0x0);
            // Clear the fine counter offset
            ble_fcntoffset_set(LLD_ADV_HDL, 0);

            //Init conflict bit
            ble_conflict_setf(LLD_ADV_HDL, 0);

            #if (HW_AUDIO)
            // Init Synch found counter and event status
            ble_btcntsync0_set(LLD_ADV_HDL, 0);
            ble_btcntsync1_set(LLD_ADV_HDL, 0);
            ble_fcntsync_set(LLD_ADV_HDL, 0);
            //Init rx and tx counters for ACL and ISO
            ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
            ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
            #endif // (HW_AUDIO)

            // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
            ble_thrcntl_ratecntl_pack(LLD_ADV_HDL, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);
            #if (BT_DUAL_MODE)
            // Set the priority properties
            lld_prio_init(LLD_ADV_HDL, rwip_priority[RWIP_PRIO_SCAN_IDX].increment, rwip_priority[RWIP_PRIO_SCAN_IDX].value,rwip_priority[RWIP_PRIO_SCAN_IDX].value);
            #endif //(BT_DUAL_MODE)

            // If active scanning is requested, prepare the scan request
            if (scan_req_pdu)
            {
                // Push the scan request
                lld_pdu_tx_push(elt, scan_req_pdu);

                // Loop the scan request data
                lld_pdu_tx_loop(evt);
            }

            GLOBAL_INT_DIS();
            // Schedule the event
            lld_evt_elt_insert(elt, true);

            // store the allocated timestamp in event anchor
            evt->evt.non_conn.anchor           = elt->timestamp;
            // calculate end of event
            evt->evt.non_conn.end_ts           = (evt->evt.non_conn.anchor + (evt->evt.non_conn.window/SLOT_SIZE)) & BLE_BASETIMECNT_MASK;
            // not an initiate procedure
            evt->evt.non_conn.initiate         = false;
            evt->evt.non_conn.connect_req_sent = false;
            GLOBAL_INT_RES();
        }
        else
        {
            // Free the element associated
            lld_evt_delete_elt_push(elt, true, false);
            elt = NULL;
        }
    }
    return (elt);
}

void lld_scan_stop(struct ea_elt_tag *elt)
{
    //Check if the elt will be programmed soon or already programmed
    if(!lld_evt_elt_delete(elt, true, true))
    {
        // Get the associated BLE specific event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

        LLD_EVT_FLAG_SET(evt, WAITING_EOEVT_TO_DELETE);
        // Abort the event
        ble_rwblecntl_set(ble_rwblecntl_get() | BLE_SCAN_ABORT_BIT);
    }
}
#endif //(BLE_CENTRAL || BLE_OBSERVER)

#if (BLE_CENTRAL)
struct ea_elt_tag* lld_con_start(struct hci_le_create_con_cmd const *con_par,
        struct em_desc_node *con_req_pdu,
        uint16_t conhdl)
{
    // Allocate event structure for scanning
    struct ea_elt_tag *elt_scan  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated BLE event environment
    struct lld_evt_tag *evt_scan = LLD_EVT_ENV_ADDR_GET(elt_scan);
    // Allocate event structure for conenction
    struct ea_elt_tag *elt_connect  = ea_elt_create(sizeof(struct lld_evt_tag));
    // Get associated BLE event environment
    struct lld_evt_tag *evt_connect = LLD_EVT_ENV_ADDR_GET(elt_connect);

    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param ={0,0,0};

    // Sanity check: There should always be a free event
    if((elt_scan != NULL)&&(elt_connect != NULL))
    {
        elt_scan->start_latency    = RWBLE_PROG_LATENCY_DFT;
        elt_scan->stop_latency1    = 0;
        elt_scan->stop_latency2    = 0;
        elt_scan->duration_min     = EA_BW_USED_DFT_US;
        elt_connect->start_latency = RWBLE_PROG_LATENCY_DFT;
        elt_connect->stop_latency1 = 0;
        elt_connect->stop_latency2 = 0;

        evt_scan->evt.non_conn.window = con_par->scan_window*SLOT_SIZE;

        /*
         * Set the scanning event
         */
        // Initialize BLE event environment
        lld_evt_init_evt(evt_scan);

        evt_scan->conhdl       = LLD_ADV_HDL;
        evt_scan->mode         = LLD_EVT_SCAN_MODE;
        evt_scan->interval     = con_par->scan_intv;
        evt_scan->cs_ptr       = REG_BLE_EM_CS_ADDR_GET(LLD_ADV_HDL);
        /*
         * Set the scanning element
         */
        //Set the INITIATING priority
        lld_util_priority_set(elt_scan, RWIP_PRIO_INIT_IDX);
        elt_scan->ea_cb_start           = lld_evt_schedule;
        elt_scan->ea_cb_cancel          = lld_evt_canceled;
        elt_scan ->ea_cb_stop           = lld_evt_prevent_stop;
        EA_ASAP_STG_SET(elt_scan, EA_FLAG_ASAP_NO_LIMIT, EA_NO_PARITY, 0, 0, 0);

        #if (BT_DUAL_MODE)
        // Set the priority properties
        lld_prio_init(evt_scan->conhdl, rwip_priority[RWIP_PRIO_INIT_IDX].increment, rwip_priority[RWIP_PRIO_INIT_IDX].value,rwip_priority[RWIP_PRIO_INIT_IDX].value);
        #endif //(BT_DUAL_MODE)

        // Schedule event as soon as possible
        elt_scan->timestamp = COMMON_ALIGN2_HI(ea_time_get_halfslot_rounded() + elt_scan->start_latency + RWBLE_ASAP_LATENCY) & BLE_BASETIMECNT_MASK;


        // Compute the scheduling parameters for the scan element from the connection parameters
        // Init the input parameters to find the first free slot to match with a connection interval
        input_param.interval_min = con_par->con_intv_min*2;
        input_param.interval_max = con_par->con_intv_max*2;
        input_param.duration_min = con_par->ce_len_min;
        input_param.duration_max = con_par->ce_len_max;
        input_param.pref_period  = 0;
        input_param.action       = EA_PARAM_REQ_GET;
        input_param.conhdl       = conhdl;
        input_param.role         = MASTER_ROLE;
        input_param.linkid       = REG_BLE_EM_CS_ADDR_GET(conhdl);
        ea_interval_duration_req(&input_param, &set_param);
        
        input_param.offset       = 0;
        input_param.odd_offset   = false;

        if(ea_offset_req(&input_param, &set_param) != EA_ERROR_REJECTED)
        {
            uint32_t random_nb = 0xFFFFFF & common_rand_word();
            struct llm_pdu_con_req_tx data;
            uint16_t ral_ptr = 0;
            uint8_t hop;
            uint8_t length;
            bool ral_en = false;

            /**
             *  Initialize the CONNECT_REQ_pdu
             */
            // Fill the tc descriptor header
            ble_txphadv_pack(con_req_pdu->idx,                     // index for the CONNECTION REQ descriptor
                            34,                                    // Length of the connect req
                            con_par->peer_addr_type& ADDR_RAND,    // peer address type
                            0,                                     // updated by HW
                            LL_CONNECT_REQ);                       // type of PDU

            // gets the access address
            llm_util_aa_gen(&data.aa.addr[0]);
            // gets the crc init
            common_write16 (&data.crcinit.crc[0],common_htobs(0x0000FFFF & random_nb));
            common_write8 (&data.crcinit.crc[2],(0x00FF0000 & random_nb)>>16);
            // gets the latency
            data.latency = common_htobs(con_par->con_latency);
            // gets the TO
            data.timeout = common_htobs(con_par->superv_to);
            //Set the default map in the PDU
            llm_util_get_channel_map(&data.chm);

            // compute the hopping interval (random number between 5 and 16)
            hop = (random_nb)%16;
            hop = (hop<5)?(hop+5):hop;
            ASSERT_ERR((hop > 4)&&(hop < 17));

            // gets the hop and sca
            data.hop_sca = hop | (lld_evt_sca_get()<<5);
            // Init interval
            data.interval = common_htobs(set_param.interval >> 1);
            // Init window size
            data.winsize = 2;
            // Prepare the PDU to be copied in the EM
            lld_pdu_adv_pack(LL_CONNECT_REQ, (uint8_t*)&data, &length);
            // Copy in the EM
            em_wr((void *)&data, ble_txdataptr_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX), length);

            // set the address depending the type
            llm_util_apply_bd_addr(con_par->own_addr_type);

            // retrieve if peer device is present in resolving list
            if(GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE)
               && ((con_par->peer_addr_type & ADDR_RPA_MASK) || (con_par->own_addr_type & ADDR_RPA_MASK)))
            {
                if(con_par->init_filt_policy == INIT_FILT_IGNORE_WLST)
                {
                    uint8_t ral_idx;
                    // retrieve set ral_ptr
                    if(llm_util_bd_addr_in_ral(&(con_par->peer_addr), con_par->peer_addr_type, &ral_idx))
                    {
                        ral_ptr = REG_BLE_EM_RAL_ADDR_GET(ral_idx);
                    }

                    ral_en = true;
                }
                else
                {
                    ral_en = (con_par->own_addr_type & ADDR_RPA_MASK);
                }
            }

            //Link the connect element to the scanning one
            elt_scan->linked_element = elt_connect;

            // Push the connection request (linked to the scan element)
            lld_pdu_tx_push(elt_scan, con_req_pdu);

            // Set the timestamp according to the parameters
            lld_util_connection_param_set(elt_scan, &set_param);

            ble_linklbl_setf(LLD_ADV_HDL,(uint8_t)(LLD_ADV_HDL & BLE_LINKLBL_MASK));

            /*
             * Initialize the Control Structure according to the parameters for the scanning
             */

            // Update the control structure according to the parameters
            ble_cntl_pack(LLD_ADV_HDL,RW_BLE_PTI_PRIO_AUTO, RWIP_COEX_GET(INIT, TXBSY), RWIP_COEX_GET(INIT, RXBSY),RWIP_COEX_GET(INIT, DNABORT), LLD_INITIATING);

            // update Resolving Address List mechanism configuration
            ble_filtpol_ralcntl_pack(LLD_ADV_HDL,
                                     /* FILTER_POLICY */  con_par->init_filt_policy,
                                     /* LOCAL_RPA_SEL */  ((con_par->own_addr_type & ADDR_RPA_MASK) != 0),
                                     /* RAL_MODE */       (ral_ptr != 0),
                                     /* RAL_EN */         ral_en);


            ble_hopcntl_set(LLD_ADV_HDL, BLE_FH_EN_BIT | 39);

            //Init conflict bit
            ble_conflict_setf(LLD_ADV_HDL, 0);

            #if (HW_AUDIO)
            // Init Synch found counter and event status
            ble_btcntsync0_set(LLD_ADV_HDL, 0);
            ble_btcntsync1_set(LLD_ADV_HDL, 0);
            ble_fcntsync_set(LLD_ADV_HDL, 0);
            //Init rx and tx counters for ACL and ISO
            ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
            ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
            #endif // (HW_AUDIO)

            ble_fcntoffset_set(LLD_ADV_HDL, 0);

            // check if a dedicated address is targeted
            if(ral_ptr == 0)
            {
                // Copy the advertiser address into the control structure
                for (uint8_t i = 0; i < (sizeof(struct bd_addr) / 2); i++)
                {
                    ble_adv_bd_addr_set(LLD_ADV_HDL, i, common_read16p(&con_par->peer_addr.addr[i * 2]));
                }
                // Copy the advertiser address type
                ble_adv_bd_addr_type_setf(LLD_ADV_HDL, con_par->peer_addr_type & ADDR_MASK);
            }
            // check if a dedicated address is targeted - using RAL
            else
            {
                ble_peer_ralptr_setf(LLD_ADV_HDL, ral_ptr);
            }

            ble_crcinit1_set(LLD_ADV_HDL, 0x55);

            // Win offset should be at least ADV duration + CONNECT_REQ duration + 1.25 ms + margin (1.25ms) = 6 slots
            // Else set the offset to the next anchor point
            if(set_param.offset < 6)
            {
                set_param.offset += (data.interval << 1);
            }

            // Save the offset to for the 1st connection anchor computation
            ble_winoffset_set(conhdl, (set_param.offset / 2) - 1);

            ble_conninterval_set(LLD_ADV_HDL,set_param.interval / 2);


            /*
             * Set the connected event
             */
            // Initialize BLE event environment
            lld_evt_init_evt(evt_connect);

            evt_connect->conhdl     = conhdl;
            evt_connect->interval   = (data.interval << 1); // Interval should be in slot internally
            evt_connect->evt.conn.latency    = data.latency + 1;// If Latency = 0 internally should be 1
            evt_connect->mode       = LLD_EVT_MST_MODE;
            evt_connect->cs_ptr     = REG_BLE_EM_CS_ADDR_GET(conhdl);
            /*
             * Set the connected element
             */
            /* Like the offset can be greater than the interval (due to the above condition if(set_param.offset < 6))
             * and the goal is to have the connected timestamp in the past a module is needed
             */
            elt_connect->timestamp              = (elt_scan->timestamp - (evt_connect->interval - (set_param.offset % evt_connect->interval))) & BLE_BASETIMECNT_MASK;
            elt_connect->duration_min           = (con_par->ce_len_min == 0)? LLD_EVT_FRAME_DURATION : con_par->ce_len_min*SLOT_SIZE;
            elt_connect->current_prio           = rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value;
            elt_connect->ea_cb_start            = lld_evt_schedule;
            elt_connect->ea_cb_cancel           = lld_evt_canceled;
            elt_connect->ea_cb_stop             = lld_evt_prevent_stop;

            /*
             * Initialize the Control Structure according to the parameters for the future link
             */
            ble_cntl_set(conhdl, LLD_MASTER_CONNECTED);
            ble_fcntoffset_set(conhdl, 0);
            // Reset Encryption states
            ble_rxcrypt_en_setf(conhdl, 0);
            ble_txcrypt_en_setf(conhdl, 0);
            // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
            ble_thrcntl_ratecntl_pack(conhdl, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);
            ble_syncwl_set(conhdl, (data.aa.addr[1] << 8) | data.aa.addr[0]);
            ble_syncwh_set(conhdl, (data.aa.addr[3] << 8) | data.aa.addr[2]);
            ble_crcinit0_set(conhdl, (data.crcinit.crc[1] << 8) | data.crcinit.crc[0]);
            ble_crcinit1_set(conhdl, data.crcinit.crc[2]);
            ble_hopcntl_set(conhdl, BLE_FH_EN_BIT | ((data.hop_sca & 0x1F) << BLE_HOP_INT_LSB));
            ble_txrxcntl_set(conhdl, rwip_rf.txpwr_max);
            // Min event time will be set later
            ble_maxevtime_set(conhdl, evt_connect->interval - elt_connect->start_latency);
            ble_linklbl_setf(conhdl,(uint8_t)(conhdl & BLE_LINKLBL_MASK));
            // Set Rx Max buf and Rx Mxx Time @0x0 -> v4.0 behavior
            ble_rxmaxbuf_set(conhdl,0x0);
            ble_rxmaxtime_set(conhdl,0x0);
            //Init conflict bit
            ble_conflict_setf(conhdl, 0);

            #if (HW_AUDIO)
            // Init Synch found counter and event status
            ble_btcntsync0_set(conhdl, 0);
            ble_btcntsync1_set(conhdl, 0);
            ble_fcntsync_set(conhdl, 0);
            //Init rx and tx counters for ACL and ISO
            ble_txrxdesccnt_set(conhdl, 0);
            ble_isotxrxpktcnt_set(conhdl, 0);
            #endif // (HW_AUDIO)

            ble_chmap0_set(conhdl,common_read16(&data.chm.map[0]));
            ble_chmap1_set(conhdl,common_read16(&data.chm.map[2]));
            ble_chmap2_set(conhdl, (uint16_t)(llm_util_check_map_validity(&data.chm.map[0],LE_CHNL_MAP_LEN) << 8)
                    | data.chm.map[4]);


            GLOBAL_INT_DIS();
            // Insert the element in the wait to be programmed list
            lld_evt_elt_insert(elt_scan, true);
            // store the allocated timestamp in event anchor
            evt_scan->evt.non_conn.anchor           = elt_scan->timestamp;
            // calculate end of event
            evt_scan->evt.non_conn.end_ts           = (evt_scan->evt.non_conn.anchor + (evt_scan->evt.non_conn.window/SLOT_SIZE)) & BLE_BASETIMECNT_MASK;
            // Initiate procedure
            evt_scan->evt.non_conn.initiate         = true;
            evt_scan->evt.non_conn.connect_req_sent = false;
            elt_scan->duration_min                  = EA_BW_USED_DFT_US;
            GLOBAL_INT_RES();
        }
        else
        {
            kernel_free(elt_scan);
            kernel_free(elt_connect);

            elt_scan = NULL;
        }
    }
    else
    {
        if(elt_scan != NULL)
        {
            kernel_free(elt_scan);
            elt_scan = NULL;
        }
        else
        {
            kernel_free(elt_connect);
        }
    }
    return (elt_scan);
}

struct ea_elt_tag* lld_move_to_master(struct ea_elt_tag *elt, uint16_t conhdl, struct llc_create_con_req_ind const *param, uint8_t rx_hdl)
{
    // Move to master connection state
    struct ea_elt_tag * elt_connected = lld_evt_move_to_master(elt, conhdl, param, rx_hdl);


    // Set effective max tx time
    lld_util_eff_tx_time_set(elt_connected, BLE_MIN_TIME, BLE_MIN_OCTETS);
    // Set min and max event time
    lld_util_compute_ce_max(elt_connected, BLE_MIN_TIME, BLE_MIN_TIME);
    ble_minevtime_set(conhdl, ble_maxevtime_get(conhdl));
    ble_txdone_setf(conhdl, 1);

    // Reset Encryption states
    ble_rxcrypt_en_setf(conhdl, 0);
    ble_txcrypt_en_setf(conhdl, 0);
    // Initialize the crypto mode to CCM
    ble_crypt_mode_setf(conhdl, 0);
    // SW workaround:  cs_ral_en in connected mode must be cleared in order to be sure that
    // cs_sk[15:0] will be correctly loaded.
    ble_filtpol_ralcntl_set(conhdl, 0);
    
    // Initialize the CCM counters
    ble_txccmpktcnt0_set(conhdl,0);
    ble_txccmpktcnt1_set(conhdl,0);
    ble_txccmpktcnt2_set(conhdl,0);
    ble_rxccmpktcnt0_set(conhdl,0);
    ble_rxccmpktcnt1_set(conhdl,0);
    ble_rxccmpktcnt2_set(conhdl,0);
    //Init event counter
    ble_evtcnt_set(conhdl,0);

    //Clear tx descriptor pointer
    ble_txdescptr_set(conhdl, 0);

    #if (HW_AUDIO)
    #if (BLE_AUDIO)
    #if (BLE_AUDIO_AM0)
    // Enable the LLID null filtering
    ble_nullrxllidflt_setf(conhdl,1);
    #endif//#if (BLE_AUDIO_AM0)
    #else
    ble_nullrxllidflt_setf(conhdl,0);
    #endif // (BLE_AUDIO)
    #endif // (HW_AUDIO)

    #if (BT_DUAL_MODE)
    // Set the priority properties
    lld_prio_init(conhdl, rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].increment, rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value,rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value);
    #endif //(BT_DUAL_MODE)

    // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
    ble_thrcntl_ratecntl_pack(conhdl, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);

    // Schedule the event
    lld_evt_elt_insert(elt_connected, true);

    return (elt_connected);
}
#endif //(BLE_CENTRAL)

#if (BLE_CENTRAL)
void lld_con_update_req(struct ea_elt_tag *elt_old,
                        struct llc_con_upd_req_ind *param,
                        struct llcp_con_upd_ind *param_pdu)
{
    struct lld_evt_update_tag upd_par;

    // Create an event to handle the connection update
    struct ea_elt_tag *elt  = lld_evt_update_create(elt_old,
                                                    common_max(2, COMMON_ALIGN2_HI(param->ce_len_min)),
                                                    (param->interval_min)*2,
                                                    (param->interval_max)*2,
                                                    param->con_latency,
                                                    param->pref_period,
                                                    &upd_par);

    // Update the fields of the connection update request
    param_pdu->timeout  = param->superv_to;
    param_pdu->interv   = LLD_EVT_ENV_ADDR_GET(elt)->interval / 2;
    param_pdu->latency  = param->con_latency;
    param_pdu->win_size = upd_par.win_size;
    param_pdu->win_off  = upd_par.win_offset;
    param_pdu->instant  = upd_par.instant;
}
#endif //(BLE_CENTRAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t lld_con_update_after_param_req(uint16_t conhdl,
                                       struct ea_elt_tag *elt_old,
                                       struct llc_con_upd_req_ind *param,
                                       struct llcp_con_upd_ind *param_pdu,
                                       bool bypass_offchk)
{
    // Get associated BLE environment
    struct lld_evt_tag *evt_old = LLD_EVT_ENV_ADDR_GET(elt_old);
    uint8_t status = COMMON_ERROR_UNSPECIFIED_ERROR;

    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param;

    input_param.interval_min    = param->interval_min*2;
    input_param.interval_max    = param->interval_max*2;
    input_param.duration_min    = param->ce_len_min; // Already expressed in slots
    input_param.duration_max    = param->ce_len_max; // Already expressed in slots
    input_param.pref_period     = param->pref_period*2;
    input_param.action          = (param->offset0 != 0xFFFF) ? EA_PARAM_REQ_CHECK : EA_PARAM_REQ_GET;
    input_param.conhdl          = conhdl;
    input_param.role            = MASTER_ROLE;
    input_param.linkid          = REG_BLE_EM_CS_ADDR_GET(conhdl);
    ea_interval_duration_req(&input_param, &set_param);

    if (param->offset0 != 0xFFFF)
    {
        uint32_t ref_time;

        // Calculate local time at reference connection event count
        if (evt_old->evt.conn.counter > param->ref_con_event_count)
            ref_time = elt_old->timestamp - (evt_old->evt.conn.counter - param->ref_con_event_count)*evt_old->interval;
        else
            ref_time = elt_old->timestamp + (param->ref_con_event_count - evt_old->evt.conn.counter)*evt_old->interval;

        input_param.offset = lld_util_get_local_offset((param->offset0 << 1), set_param.interval, ref_time);
    }
    else
    {
        input_param.offset = elt_old->timestamp % set_param.interval;
    }
    input_param.odd_offset  = (input_param.offset & 0x01)? 1 : 0;

    if((input_param.action == EA_PARAM_REQ_CHECK) && (ea_offset_req(&input_param, &set_param) != EA_ERROR_OK))
    {
        input_param.action = EA_PARAM_REQ_GET;
        if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
        {
            status = COMMON_ERROR_NO_ERROR;
        }
    }
    else
    {
        status = COMMON_ERROR_NO_ERROR;
    }

    if((status == COMMON_ERROR_NO_ERROR) || (bypass_offchk))
    {
        // Allocate temporary event structure
        struct ea_elt_tag *elt  = ea_elt_create(sizeof(struct lld_evt_tag));

        if(elt != NULL)
        {
            // Get associated BLE environment
            struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
            // Fill in event properties
            memcpy(elt,elt_old,sizeof(struct ea_elt_tag));

            // Initialize BLE event environment
            lld_evt_init_evt(evt);

            evt->evt.conn.latency    = param->con_latency + 1;
            evt->mode       = LLD_EVT_MST_MODE;

            evt->interval = set_param.interval;

            uint16_t offset0, ref_con_event_count;
            if (param->offset0 == 0xFFFF)
            {
                offset0 = lld_util_get_peer_offset(set_param.offset, set_param.interval, elt_old->timestamp) / 2;
                ref_con_event_count = evt_old->evt.conn.counter;
            }
            else
            {
                offset0 = param->offset0;
                ref_con_event_count = param->ref_con_event_count;
            }

            // Compute the parameters of the connection update

            uint16_t dt;
            // The instant will be 7 wake-up times after the next event
            uint16_t count_to_inst = evt_old->evt.conn.latency * 7;
            // Compute update instant
            uint16_t instant  = (evt_old->evt.conn.counter + count_to_inst - 1) & RWBLE_INSTANT_MASK;
            // Compute the old event time at instant
            uint32_t time_old = (elt_old->timestamp + (evt_old->interval * (count_to_inst - 1))) & BLE_BASETIMECNT_MASK;

            // Compute dt (expressed in slots)
            if (ref_con_event_count > instant)
                dt = ((ref_con_event_count - instant)*evt_old->interval + offset0*2) % evt->interval;
            else
                dt = (evt->interval - ((instant - ref_con_event_count)*evt_old->interval - offset0*2) % evt->interval) % evt->interval;

            // The master should send the packet at the beginning of the tx window
            elt->timestamp = (time_old + dt) & BLE_BASETIMECNT_MASK;

            // Switch will be performed by the SW one event before the instant
            evt_old->evt.conn.instant = instant;

            // Store the new event pointer into the old one, in order to program it at instant
            elt_old->linked_element = elt;
            evt_old->evt.conn.instant_action = LLD_UTIL_PARAM_UPDATE;

            // New values are applied directly to control structure (before instant has passed)
            // It is acceptable as the "param update" procedure always finished and these
            // parameters are just given as information from the spec
            ble_minevtime_set(conhdl, (elt->duration_min/SLOT_SIZE));

            lld_util_compute_ce_max(elt, llc_env[conhdl]->data_len_ext_info.conn_eff_max_tx_time,
                                         llc_env[conhdl]->data_len_ext_info.conn_eff_max_rx_time);

            // Update the fields of the connection update request
            param_pdu->timeout  = param->superv_to;
            param_pdu->interv   = evt->interval / 2;
            param_pdu->latency  = param->con_latency;
            param_pdu->win_size = 1;
            param_pdu->win_off  = (dt/2);
            param_pdu->instant  = instant;
        }
    }

    return(status);
}

uint8_t lld_con_param_rsp(uint16_t conhdl,
                          struct ea_elt_tag *elt,
                          struct llc_con_upd_req_ind *param)
{
    // Get associated BLE environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    uint8_t status = COMMON_ERROR_UNSPECIFIED_ERROR;

    // Input parameters
    struct ea_param_input input_param;
    struct ea_param_output set_param;

    input_param.interval_min    = param->interval_min*2;
    input_param.interval_max    = param->interval_max*2;
    input_param.duration_min    = evt->evt.conn.duration_dft;
    input_param.duration_max    = evt->evt.conn.duration_dft;
    input_param.pref_period     = param->pref_period*2;
    input_param.action          = (param->offset0 != 0xFFFF) ? EA_PARAM_REQ_CHECK : EA_PARAM_REQ_GET;
    input_param.conhdl          = conhdl;
    input_param.role            = SLAVE_ROLE;
    input_param.linkid          = REG_BLE_EM_CS_ADDR_GET(conhdl);
    ea_interval_duration_req(&input_param, &set_param);

    if (param->offset0 != 0xFFFF)
    {
        uint32_t ref_time;

        // Calculate local time at reference connection event count
        if (evt->evt.conn.counter > param->ref_con_event_count)
            ref_time = elt->timestamp - (evt->evt.conn.counter - param->ref_con_event_count)*evt->interval;
        else
            ref_time = elt->timestamp + (param->ref_con_event_count - evt->evt.conn.counter)*evt->interval;

        input_param.offset = lld_util_get_local_offset(param->offset0*2, set_param.interval, ref_time);
    }
    else
    {
        input_param.offset = elt->timestamp % set_param.interval;
    }
    input_param.odd_offset  = (input_param.offset & 0x01)? 1 : 0;

    if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
    {
        param->interval_min = set_param.interval/2;
        param->interval_max = set_param.interval/2;
        param->pref_period  = 0;

        if (param->offset0 == 0xFFFF)
        {
            param->offset0 = lld_util_get_peer_offset(set_param.offset, set_param.interval, elt->timestamp) >> 1;
            param->ref_con_event_count = evt->evt.conn.counter;
        }
        param->offset1 = 0xFFFF;
        param->offset2 = 0xFFFF;
        param->offset3 = 0xFFFF;
        param->offset4 = 0xFFFF;
        param->offset5 = 0xFFFF;

        status = COMMON_ERROR_NO_ERROR;
    }

    return(status);
}

void lld_con_param_req(uint16_t conhdl, struct ea_elt_tag *elt, struct llc_con_upd_req_ind *param)
{
    // Get associated event
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    #if (BLE_TESTER)
    // ensure that we keep parameters given by HCI command
    if (param->pref_period != 0)
    {
        param->interval_min         = param->con_intv_min;
        param->interval_max         = param->con_intv_max;
        param->ref_con_event_count  = evt->evt.conn.counter + REF_COUNTER_MARGIN;
        param->offset1              = 0xFFFF;
        param->offset2              = 0xFFFF;
        param->offset3              = 0xFFFF;
        param->offset4              = 0xFFFF;
        param->offset5              = 0xFFFF;
    }
    else
    #endif // (BLE_TESTER)
    {
        // Input parameters
        struct ea_param_input input_param;
        struct ea_param_output set_param;
        input_param.interval_min    = param->con_intv_min*2;
        input_param.interval_max    = param->con_intv_max*2;

        if(param->ce_len_min == 0)
        {
            input_param.duration_min = evt->evt.conn.duration_dft;
        }
        else
        {
            input_param.duration_min = param->ce_len_min;
        }

        input_param.duration_max    = param->ce_len_max;
        input_param.pref_period     = 0;
        input_param.action          = EA_PARAM_REQ_GET;
        input_param.conhdl          = conhdl;
        input_param.role            = (evt->mode == LLD_EVT_MST_MODE) ? MASTER_ROLE : SLAVE_ROLE;
        input_param.linkid          = REG_BLE_EM_CS_ADDR_GET(conhdl);
        ea_interval_duration_req(&input_param, &set_param);
        
        input_param.offset          = elt->timestamp % set_param.interval;
        input_param.odd_offset      = (input_param.offset & 0x01)? 1 : 0;

        if(ea_offset_req(&input_param, &set_param) == EA_ERROR_OK)
        {
            param->interval_min         = set_param.interval/2;
            param->interval_max         = set_param.interval/2;
            param->offset0 = lld_util_get_peer_offset(input_param.offset, set_param.interval, elt->timestamp) >> 1;
        }
        else
        {
            param->interval_min         = input_param.interval_max/2;
            param->interval_max         = input_param.interval_max/2;
            param->offset0 = lld_util_get_peer_offset(input_param.offset, input_param.interval_max, elt->timestamp) >> 1;
        }

        param->pref_period          = 0;
        param->ref_con_event_count  = evt->evt.conn.counter + REF_COUNTER_MARGIN;
        param->offset1              = 0xFFFF;
        param->offset2              = 0xFFFF;
        param->offset3              = 0xFFFF;
        param->offset4              = 0xFFFF;
        param->offset5              = 0xFFFF;
    }
}


void lld_con_stop(struct ea_elt_tag *elt)
{
    lld_evt_delete_elt_push(elt, true, true);
}

uint8_t lld_get_mode(uint16_t conhdl)
{
    // Element allocated for the connection
    struct ea_elt_tag *elt = NULL;
    // Mode
    uint8_t mode = LLD_EVT_MODE_MAX;

    // Check connection handle in order to know if link managed by LLM or LLC
    if (conhdl == LLD_ADV_HDL)
    {
        elt = llm_le_env.elt;
    }
    else
    {
        // Check if the environment exists
        if (llc_env[conhdl])
        {
            elt = llc_env[conhdl]->elt;
        }
    }

    if (elt)
    {
        // Get the current mode
        mode = LLD_EVT_ENV_ADDR_GET(elt)->mode;
    }

    return (mode);
}

#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

#if (BLE_PERIPHERAL)
struct ea_elt_tag* lld_move_to_slave(struct llc_create_con_req_ind *con_par,
        struct llm_pdu_con_req_rx *con_req_pdu,
        struct ea_elt_tag *elt_adv,
        uint16_t conhdl)
{
	//UART_PRINTF("%s\r\n", __func__);
	
    // Element dedicated for the connection
    struct ea_elt_tag * elt_connected;

    // Update the connection control structure according to the parameters
    ble_cntl_set(conhdl, LLD_SLAVE_CONNECTED);
    ble_txdone_setf(conhdl, 1);
    ble_hopcntl_set(conhdl, BLE_FH_EN_BIT | ((con_req_pdu->hop_sca & 0x1F) << BLE_HOP_INT_LSB));
    ble_syncwl_set(conhdl, (con_req_pdu->aa.addr[1] << 8) | con_req_pdu->aa.addr[0]);
    ble_syncwh_set(conhdl, (con_req_pdu->aa.addr[3] << 8) | con_req_pdu->aa.addr[2]);
    ble_crcinit0_set(conhdl, (con_req_pdu->crcinit.crc[1] << 8) | con_req_pdu->crcinit.crc[0]);
    ble_crcinit1_set(conhdl, con_req_pdu->crcinit.crc[2]);
    ble_txrxcntl_set(conhdl, rwip_rf.txpwr_max);
    // Set the CS-fields rx and tx rate to 1MBPS and RX threshold
    ble_thrcntl_ratecntl_pack(conhdl, LLD_RX_IRQ_THRES,  LLD_CS_RATE_1MBPS, LLD_CS_RATE_1MBPS);
    ble_linklbl_setf(conhdl,(uint8_t)(conhdl & BLE_LINKLBL_MASK));
    ble_chmap0_set(conhdl,common_read16(&con_req_pdu->chm.map[0]));
    ble_chmap1_set(conhdl,common_read16(&con_req_pdu->chm.map[2]));
    ble_chmap2_set(conhdl,
            (uint16_t)(llm_util_check_map_validity(&con_req_pdu->chm.map[0],LE_CHNL_MAP_LEN) << 8)
            | con_req_pdu->chm.map[4]);

    // Reset Encryption states
    ble_rxcrypt_en_setf(conhdl, 0);
    ble_txcrypt_en_setf(conhdl, 0);
    // Initialize the crypto mode to CCM
    ble_crypt_mode_setf(conhdl, 0);
    // SW workaround:  cs_ral_en in connected mode must be cleared in order to be sure that
    // cs_sk[15:0] will be correctly loaded.
    ble_filtpol_ralcntl_set(conhdl, 0);


    //Init conflict bit
    ble_conflict_setf(conhdl, 0);

    #if (HW_AUDIO)
    // Init Synch found counter and event status
    ble_btcntsync0_set(conhdl, 0);
    ble_btcntsync1_set(conhdl, 0);
    ble_fcntsync_set(conhdl, 0);
    //Init rx and tx counters for ACL and ISO
    ble_txrxdesccnt_set(conhdl, 0);
    ble_isotxrxpktcnt_set(conhdl, 0);
    #endif // (HW_AUDIO)
    // Move to the slave connected state
    elt_connected = lld_evt_move_to_slave(con_par, con_req_pdu, elt_adv, conhdl);
    // Set effective max tx time and size
    lld_util_eff_tx_time_set(elt_connected, BLE_MIN_TIME, BLE_MIN_OCTETS);
    //Init max event time
    lld_util_compute_ce_max(elt_connected, BLE_MIN_TIME, BLE_MIN_TIME);
    ble_minevtime_set(conhdl, ble_maxevtime_get(conhdl));

    // Set Rx Max buf and Rx Mxx Time @0x0 -> v4.0 behavior
    ble_rxmaxbuf_set(conhdl,0x0);
    ble_rxmaxtime_set(conhdl,0x0);

    // Initialize the CCM counters
    ble_txccmpktcnt0_set(conhdl,0);
    ble_txccmpktcnt1_set(conhdl,0);
    ble_txccmpktcnt2_set(conhdl,0);
    ble_rxccmpktcnt0_set(conhdl,0);
    ble_rxccmpktcnt1_set(conhdl,0);
    ble_rxccmpktcnt2_set(conhdl,0);
    //Init event counter
    ble_evtcnt_set(conhdl,0);

    //Clear tx descriptor pointer
    ble_txdescptr_set(conhdl, 0);

    #if (HW_AUDIO)
    #if (BLE_AUDIO)
    #if (BLE_AUDIO_AM0)
    // Enable the LLID null filtering
    ble_nullrxllidflt_setf(conhdl,1);
    #endif //#if (BLE_AUDIO_AM0)
    #else
    ble_nullrxllidflt_setf(conhdl,0);
    #endif  // (BLE_AUDIO)
    #endif // (HW_AUDIO)

    #if (BT_DUAL_MODE)
    // Set the priority properties
    lld_prio_init(conhdl, rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].increment, rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value,rwip_priority[RWIP_PRIO_LE_CON_IDLE_IDX].value);
    #endif //(BT_DUAL_MODE)

    // Schedule the event
    lld_evt_elt_insert(elt_connected, true);

    return (elt_connected);
}

void lld_ch_map_ind(struct ea_elt_tag *elt, uint16_t instant)
{
    // Get address of the BLE specific event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);

    // Store the channel map update information in the event
    evt->evt.conn.instant        = instant;
    evt->evt.conn.instant_action = LLD_UTIL_CHMAP_UPDATE;
    LLD_EVT_FLAG_SET(evt, WAITING_ACK);

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
        // Apply immediately new channel map
        llc_map_update_ind(evt->conhdl);

        // Reset waiting instant flag
        LLD_EVT_FLAG_SET(evt, WAITING_INSTANT);
    }
}

void lld_con_update_ind(struct ea_elt_tag *elt_old,
        struct llcp_con_upd_ind const *param_pdu)
{
    // Create an event to handle the connection update
    lld_evt_slave_update(param_pdu, elt_old);
}
#endif //(BLE_PERIPHERAL)

void lld_crypt_isr(void)
{
    // Set kernel event for deferred handling
    kernel_event_set(KERNEL_EVENT_BLE_CRYPT);
}

#if (BLE_TEST_MODE_SUPPORT)
struct ea_elt_tag *lld_test_mode_tx(bool enh_cmd, struct em_desc_node *txdesc, uint8_t tx_freq, uint8_t phy)
{
    // Create an event to handle the tx test mode
    struct ea_elt_tag *elt = lld_evt_adv_create(LLD_ADV_HDL, 0, 1, false);

    LLD_EVT_ENV_ADDR_GET(elt)->mode = LLD_EVT_TEST_MODE;

    // Set the RF test control register
    // Tx packet count enabled, and reported in CS-TXCCMPKTCNT and RFTESTRXSTAT-TXPKTCNT on RF abort command
    ble_txpktcnten_setf(1);
    // Clear counter dedicated for the test mode
    ble_txccmpktcnt0_set(LLD_ADV_HDL,0);
    ble_txccmpktcnt1_set(LLD_ADV_HDL,0);
    ble_txccmpktcnt2_setf(LLD_ADV_HDL,0);

    // Update the control structure according to the parameters
    ble_cntl_set(LLD_ADV_HDL, LLD_TXTEST_MODE);

    // Initialize the advertising control structure
    ble_syncwl_set(LLD_ADV_HDL, 0x4129);
    ble_syncwh_set(LLD_ADV_HDL, 0x7176);
    ble_txpwr_setf(LLD_ADV_HDL, 0xF);
    // Clear the fine counter offset
    ble_fcntoffset_set(LLD_ADV_HDL, 0);
    //Init conflict bit
    ble_conflict_setf(LLD_ADV_HDL, 0);
    //Set link label
    ble_linklbl_setf(LLD_ADV_HDL,(uint8_t)(LLD_ADV_HDL & BLE_LINKLBL_MASK));

    #if (HW_AUDIO)
    // Init Synch found counter and event status
    ble_btcntsync0_set(LLD_ADV_HDL, 0);
    ble_btcntsync1_set(LLD_ADV_HDL, 0);
    ble_fcntsync_set(LLD_ADV_HDL, 0);
    //Init rx and tx counters for ACL and ISO
    ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
    ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
    #endif //(HW_AUDIO)
    #if (BLE_2MBPS)
    if(enh_cmd)
    {
        /**
         * CS-field converted
         * Transmit Rate:
         *  00: 1Mbps uncoded PHY
         *  01: 2Mbps uncoded PHY
         *  1x: Reserved
         */
        phy = (phy == PHYS_2MBPS_PREF)? LLD_CS_RATE_2MBPS : LLD_CS_RATE_1MBPS;
        //Init tx PHY
        ble_txrate_setf(LLD_ADV_HDL, phy);
    }
    else
    #endif // ((BLE_2MBPS)
    {
        ble_txrate_setf(LLD_ADV_HDL, LLD_CS_RATE_1MBPS);
    }

    #if (BT_DUAL_MODE)
    // Set the priority properties
    lld_prio_init(LLD_ADV_HDL, rwip_priority[RWIP_PRIO_SCAN_IDX].increment, rwip_priority[RWIP_PRIO_SCAN_IDX].value,rwip_priority[RWIP_PRIO_SCAN_IDX].value);
    #endif //(BT_DUAL_MODE)

    // Set tx power index for this channel
    rwnx_cal_set_txpwr_by_channel(tx_freq);

    // Get the channel index matching with the provided frequency
    ble_ch_idx_setf(LLD_ADV_HDL, lld_util_freq2chnl(tx_freq));
    if(elt != NULL)
    {
        // Chain the tx test mode data into the control structure
        lld_pdu_tx_push(elt, txdesc);

        // Loop the tx test mode data
        lld_pdu_tx_loop(LLD_EVT_ENV_ADDR_GET(elt));

        // Insert the event to the list
        lld_evt_elt_insert(elt, true);
    }

    return (elt);
}

struct ea_elt_tag *lld_test_mode_rx(bool enh_cmd, uint8_t rx_freq, uint8_t phy, uint8_t modul_idx)
{
    // Create an event to handle the advertising
    struct ea_elt_tag *elt = lld_evt_adv_create(LLD_ADV_HDL, 0, 1, false);

    LLD_EVT_ENV_ADDR_GET(elt)->mode = LLD_EVT_TEST_MODE;

    // Set the RF test control register
    // Rx packet count enabled, and reported in CS-RXCCMPKTCNT and RFTESTRXSTAT-RXPKTCNT on RF abort command
    ble_rxpktcnten_setf(1);
    // Clear counter dedicated for the test mode
    ble_rxccmpktcnt0_set(LLD_ADV_HDL,0);
    ble_rxccmpktcnt1_set(LLD_ADV_HDL,0);
    ble_rxccmpktcnt2_setf(LLD_ADV_HDL,0);

    // Update the control structure according to the parameters
    ble_cntl_set(LLD_ADV_HDL, LLD_RXTEST_MODE);
    ble_rxwincntl_set(LLD_ADV_HDL, BLE_RXWIDE_BIT | 2);
    // Initialize the advertising control structure
    ble_syncwl_set(LLD_ADV_HDL, 0x4129);
    ble_syncwh_set(LLD_ADV_HDL, 0x7176);
    // Clear the fine counter offset
    ble_fcntoffset_set(LLD_ADV_HDL, 0);

    // extend length to 251
    ble_rxmaxbuf_set(LLD_ADV_HDL, RX_TEST_SIZE_MAX);
    ble_rxmaxtime_set(LLD_ADV_HDL, 0);

    //Init conflict bit
    ble_conflict_setf(LLD_ADV_HDL, 0);

    //Set link label
    ble_linklbl_setf(LLD_ADV_HDL,(uint8_t)(LLD_ADV_HDL & BLE_LINKLBL_MASK));

    #if (HW_AUDIO)
    // Init Synch found counter and event status
    ble_btcntsync0_set(LLD_ADV_HDL, 0);
    ble_btcntsync1_set(LLD_ADV_HDL, 0);
    ble_fcntsync_set(LLD_ADV_HDL, 0);
    //Init rx and tx counters for ACL and ISO
    ble_txrxdesccnt_set(LLD_ADV_HDL, 0);
    ble_isotxrxpktcnt_set(LLD_ADV_HDL, 0);
	#endif // (HW_AUDIO)
    #if (BLE_2MBPS)
    if(enh_cmd)
    {
        /**
         * CS-field converted
         * Receive Rate:
         *  00: 1Mbps uncoded PHY
         *  01: 2Mbps uncoded PHY
         *  1x: Reserved
         */
        phy = (phy == PHYS_2MBPS_PREF)? LLD_CS_RATE_2MBPS : LLD_CS_RATE_1MBPS;
        //Init rx PHY
        ble_rxrate_setf(LLD_ADV_HDL, phy);
    }
    else
    #endif // ((BLE_2MBPS)
    {
        ble_rxrate_setf(LLD_ADV_HDL, LLD_CS_RATE_1MBPS);
    }


    #if (BT_DUAL_MODE)
    // Set the priority properties
    lld_prio_init(LLD_ADV_HDL, rwip_priority[RWIP_PRIO_SCAN_IDX].increment, rwip_priority[RWIP_PRIO_SCAN_IDX].value,rwip_priority[RWIP_PRIO_SCAN_IDX].value);
    #endif //(BT_DUAL_MODE)

    // Get the channel index matching with the provided frequency
    ble_ch_idx_setf(LLD_ADV_HDL, lld_util_freq2chnl(rx_freq));

    // Disable force AGC mechanism
    rwip_rf.force_agc_enable(false);
    if(elt != NULL)
    {
        // Insert the event to the list
        lld_evt_elt_insert(elt, true);
    }
    return (elt);
}

void lld_test_stop(struct ea_elt_tag *elt)
{
    // Abort the event
    ble_rwblecntl_set(ble_rwblecntl_get() | BLE_RFTEST_ABORT_BIT);

    // Re-enable force AGC mechanism
    rwip_rf.force_agc_enable(true);
    // Clear the Regulatory Body and RF Testing Register
    ble_rftestcntl_set(0);
    //Go back in basic rate
    ble_rxrate_setf(LLD_ADV_HDL, PHYS_NO_PREF);
}
#endif // (BLE_TEST_MODE_SUPPORT)


void lld_ral_renew_req(struct ea_elt_tag *elt)
{
    // request renewal of RPA
    GLOBAL_INT_DIS();
    lld_evt_env.renew = true;
    GLOBAL_INT_RES();
}

/// @} LLD

