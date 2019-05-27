/**
 ****************************************************************************************
 *
 * @file llm.c
 *
 * @brief Definition of the functions used by the logical link manager
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_compiler.h"
//#include "em_buf.h"
#include "llm.h"

#include "common_bt.h"
#include "common_endian.h"
#include "common_error.h"
#include "common_list.h"
#include "common_math.h"
#include "common_utils.h"
#include "kernel_event.h"
#include "kernel_task.h"
#include "kernel_timer.h"
#include "llm_task.h"
#include "llm_util.h"
#include "llc_task.h"
#include "llc_util.h"
#include "llcontrl.h"
#include "lld.h"
#include "lld_util.h"
#include "lld_evt.h"
#include "rwip.h"
#if (HCI_PRESENT)
#include "hci.h"
#endif //(HCI_PRESENT)

#include "ble_reg_access.h"
#include "reg_ble_em_wpb.h"
#include "reg_ble_em_wpv.h"
#include "reg_ble_em_ral.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_blecore.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif
#include "dbg_swdiag.h"
#if (SECURE_CONNECTIONS)
#include "ecc_p256.h"
#endif // (SECURE_CONNECTIONS)

#if((BLE_CENTRAL || BLE_PERIPHERAL) && BLE_AUDIO)
#include "audio.h"
#endif
#include "RomCallFlash.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define TEST_MODE_BUFF_IDX  (0)
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// LLM environment structures table to be in retention area
struct llm_le_env_tag llm_le_env;


/// DEFAULT BD address
static const struct bd_addr llm_dflt_bdaddr = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

/// Local supported commands
const struct supp_cmds llm_local_cmds =
    {{BLE_CMDS_BYTE0, 0, BLE_CMDS_BYTE2, 0, 0, BLE_CMDS_BYTE5, 0, 0, 0, 0,
      BLE_CMDS_BYTE10, 0, 0, 0, BLE_CMDS_BYTE14, BLE_CMDS_BYTE15, 0, 0 ,0 ,0 ,
      0, 0, BLE_CMDS_BYTE22, 0, 0, BLE_CMDS_BYTE25, BLE_CMDS_BYTE26, BLE_CMDS_BYTE27, BLE_CMDS_BYTE28, 0,
      0, 0, BLE_CMDS_BYTE32, BLE_CMDS_BYTE33, BLE_CMDS_BYTE34, BLE_CMDS_BYTE35, BLE_CMDS_BYTE36, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0}};

/// Local LE supported features
const struct le_features llm_local_le_feats =
    {{ BLE_FEATURES_BYTE0, BLE_FEATURES_BYTE1, BLE_FEATURES_BYTE2, BLE_FEATURES_BYTE3,
       BLE_FEATURES_BYTE4, BLE_FEATURES_BYTE5, BLE_FEATURES_BYTE6, BLE_FEATURES_BYTE7 }};

/// Local LE supported states
const struct le_states llm_local_le_states =
    {{ BLE_STATES_BYTE0, BLE_STATES_BYTE1, BLE_STATES_BYTE2, BLE_STATES_BYTE3,
       BLE_STATES_BYTE4, BLE_STATES_BYTE5, BLE_STATES_BYTE6, BLE_STATES_BYTE7 }};

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///Local LE supported value for the data length extension
/// Local LE supported features
const struct data_len_ext llm_local_data_len_values =
    {BLE_MIN_OCTETS, BLE_MIN_TIME, BLE_MAX_OCTETS, BLE_MAX_TIME, BLE_MAX_OCTETS,BLE_MAX_TIME };

#endif

/// LLM task descriptor
static const struct kernel_task_desc TASK_DESC_LLM = {NULL, &llm_default_handler, llm_state, LLM_STATE_MAX, LLM_IDX_MAX};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void llm_wlpub_addr_set(uint16_t elem_index, struct bd_addr const *bdaddr)
{
    // store the address at the desired index
	int i = 0;
    for ( i = 0; i < BLE_WLPUB_COUNT; i++)
    {
        ble_wlpub_set(elem_index, i, common_read16p(&bdaddr->addr[2 * i]));
    }
}

static void llm_wlpriv_addr_set(uint16_t elem_index, struct bd_addr const *bdaddr)
{
    // store the address at the desired index
	int i = 0;
    for ( i = 0; i < BLE_WLPRV_COUNT; i++)
    {
        ble_wlprv_set(elem_index, i, common_read16p(&bdaddr->addr[2 * i]));
    }
}

#if (BLE_OBSERVER || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Sets and send the value for the advertising report.
 *
 * This function gets the information from the received advertising packet and sets the
 * values in the advertising report event.
 *
 * @param[in] desc        Pointer on the received advertising.
 * @param[in] adv_type    Advertising type
 *
 ****************************************************************************************
 */
static void llm_adv_report_send(uint8_t rx_hdl, uint8_t adv_type)
{
    uint16_t ral_ptr   = ble_rxralptr_get(rx_hdl);
    #if (HW_AUDIO)
    uint16_t rx_status = ble_rxstat_get(rx_hdl);
    #endif // (HW_AUDIO)

    struct bd_addr inita;
    struct bd_addr adva;
    // retrieve InitA address info
    em_rd(&inita,(ble_rxdataptr_get(rx_hdl)+BD_ADDR_LEN),BD_ADDR_LEN);
    em_rd(&adva,ble_rxdataptr_get(rx_hdl),BD_ADDR_LEN);

    // check if a LE Direct ADV report should be triggered
    if((adv_type == LL_ADV_CONN_DIR)
            // only when scan filer policy = 2 or 3
            && (llm_le_env.scanning_params->filterpolicy >= SCAN_ALLOW_ADV_ALL_AND_INIT_RPA)
            // Resolvable Private address received
            && (ble_rxrxadd_getf(rx_hdl) && ((inita.addr[BD_ADDR_LEN-1] & 0xC0) == RND_RSLV_ADDR))
            // check that address has not been resolved
            && ((ral_ptr == 0)
                #if (HW_AUDIO)
                || ((rx_status & BLE_PRIV_ERROR_BIT) && (rx_status & BLE_PEER_ADD_MATCH_BIT))
                #endif // (HW_AUDIO)
                ))
    {
        // check if LE Direct ADV Report event is filtered or not
        if(llm_util_check_evt_mask(LE_DIR_ADV_REP_EVT_BIT))
        {
            struct hci_le_dir_adv_rep_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, 0, 0, hci_le_dir_adv_rep_evt);

            // gets event subcode
            event->subcode = HCI_LE_DIR_ADV_REP_EVT_SUBCODE;
            // gets the number of advertising received
            event->nb_reports = 1;

            event->adv_rep[0].rssi     = rwip_rf.rssi_convert(ble_rssi_getf(rx_hdl));
            event->adv_rep[0].evt_type = ble_rxtype_getf(rx_hdl);

            // copy AdvA Address
            if(ral_ptr != 0)
            {
                uint8_t ral_idx;

                // retrieve Identity index in RAL
                ral_idx = (ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;

                em_rd(event->adv_rep[0].addr.addr, ral_ptr + BLE_RAL_PEER_ID_INDEX*2,BD_ADDR_LEN);
                event->adv_rep[0].addr_type = ble_peer_id_type_getf(ral_idx);

                if((adva.addr[BD_ADDR_LEN-1] & 0xC0) == RND_RSLV_ADDR)
                {
                    event->adv_rep[0].addr_type |= ADDR_RPA_MASK;
                }

                // copy local RPA used by advertiser - present in the descriptor.
                memcpy(event->adv_rep[0].dir_addr.addr, inita.addr, BD_ADDR_LEN);
                event->adv_rep[0].dir_addr_type = ADDR_RAND;
            }
            // retrieve peer identity from data buffer
            else
            {
                em_rd(event->adv_rep[0].addr.addr,ble_rxdataptr_get(rx_hdl),BD_ADDR_LEN);
                event->adv_rep[0].addr_type = ble_rxtxadd_getf(rx_hdl);

                // copy local RPA used by advertiser - present in the descriptor.
                memcpy(event->adv_rep[0].dir_addr.addr, inita.addr, BD_ADDR_LEN);
                event->adv_rep[0].dir_addr_type = ADDR_RAND;
            }
            hci_send_2_host(event);
        }
    }
    // check if LE ADV Report event is filtered or not
    else if (llm_util_check_evt_mask(LE_ADV_REP_EVT_BIT))
    {
        // allocate the status event message
        struct hci_le_adv_report_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, 0, 0, hci_le_adv_report_evt);
        // gets event subcode
        event->subcode = HCI_LE_ADV_REPORT_EVT_SUBCODE;
        // gets the number of advertising received
        event->nb_reports = 1;

        event->adv_rep[0].rssi     = rwip_rf.rssi_convert(ble_rssi_getf(rx_hdl));
        event->adv_rep[0].evt_type = ble_rxtype_getf(rx_hdl);
        switch(event->adv_rep[0].evt_type)
        {
            case LL_ADV_CONN_UNDIR:
                event->adv_rep[0].evt_type = ADV_CONN_UNDIR;
                break;
            case LL_ADV_CONN_DIR:
                event->adv_rep[0].evt_type = ADV_CONN_DIR;
                break;
            case LL_ADV_NONCONN_UNDIR:
                event->adv_rep[0].evt_type = ADV_NONCONN_UNDIR;
                break;
            case LL_ADV_DISC_UNDIR:
                event->adv_rep[0].evt_type = ADV_DISC_UNDIR;
                break;
            case LL_SCAN_RSP:
                event->adv_rep[0].evt_type = LL_SCAN_RSP;
                break;
            default:
                break;
        }
        if(event->adv_rep[0].evt_type == ADV_CONN_DIR)
        {
            event->adv_rep[0].data_len = 0;
        }
        else
        {
            event->adv_rep[0].data_len = ble_rxadvlen_getf(rx_hdl) - BD_ADDR_LEN;
            if(event->adv_rep[0].data_len > ADV_DATA_LEN)
            {
                event->adv_rep[0].data_len = ADV_DATA_LEN;
            }
            em_rd(event->adv_rep[0].data, ble_rxdataptr_get(rx_hdl) + ADV_DATA_OFFSET , event->adv_rep[0].data_len);
        }

        // copy AdvA Address
        if((ral_ptr == 0)
                #if (HW_AUDIO)
                || ((rx_status & BLE_PRIV_ERROR_BIT)&& !(rx_status & BLE_PEER_ADD_MATCH_BIT))
                #endif //(HW_AUDIO)
                )
        {
            em_rd((void*)&event->adv_rep[0].adv_addr,ble_rxdataptr_get(rx_hdl),BD_ADDR_LEN);
            event->adv_rep[0].adv_addr_type = ble_rxtxadd_getf(rx_hdl);
        }
        // retrieve peer identity from resolving list
        else
        {
            // retrieve Identity index in RAL
            uint8_t ral_idx = (ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;
            // copy peer identity info

            em_rd(event->adv_rep[0].adv_addr.addr, ral_ptr + BLE_RAL_PEER_ID_INDEX*2, BD_ADDR_LEN);
            event->adv_rep[0].adv_addr_type = ble_peer_id_type_getf(ral_idx);

            if((adva.addr[BD_ADDR_LEN-1] & 0xC0) == RND_RSLV_ADDR)
            {
                event->adv_rep[0].adv_addr_type |= ADDR_RPA_MASK;
            }
        }
        // send the message
        hci_send_2_host(event);
    }
}
#endif // (BLE_OBSERVER || BLE_CENTRAL)


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void llm_init(bool reset)
{
    #if (BLE_CENTRAL && NVDS_SUPPORT && BLE_CHNL_ASSESS)
    uint8_t buffer_len;
    #endif

    if (!reset)
    {
        // Create LLM task
        kernel_task_create(TASK_LLM, &TASK_DESC_LLM);

        // Register BLE encryption event kernel event
        kernel_event_callback_set(KERNEL_EVENT_BLE_CRYPT, &llm_encryption_done);
    }
    else
    {
        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        if (llm_le_env.advertising_params)
        {
            if (llm_le_env.advertising_params->adv_data_req)
            {
                // Free the stored message
                kernel_msg_free(llm_le_env.advertising_params->adv_data_req);
            }

            if (llm_le_env.advertising_params->scan_rsp_req)
            {
                // Free the stored message
                kernel_msg_free(llm_le_env.advertising_params->scan_rsp_req);
            }

            kernel_free(llm_le_env.advertising_params);
        }
        #endif //(BLE_PERIPHERAL || BLE_BROADCASTER)

        #if (BLE_OBSERVER || BLE_CENTRAL)
        // Free structure containing scanning parameters if needed
        if (llm_le_env.scanning_params)
        {
            kernel_free(llm_le_env.scanning_params);
        }

        lld_util_flush_list(&llm_le_env.adv_list);
        #endif //(BLE_OBSERVER || BLE_CENTRAL)

        #if (BLE_PERIPHERAL || BLE_CENTRAL)
        lld_util_flush_list(&llm_le_env.cnx_list);
        #endif// (BLE_PERIPHERAL || BLE_CENTRAL)
    }

    kernel_state_set(TASK_LLM, LLM_IDLE);
    // Status init
    #if (BLE_TEST_MODE_SUPPORT)
    // Direct test mode
    llm_le_env.test_mode.end_of_tst=false;
    llm_le_env.test_mode.directtesttype  = TEST_END;
    #endif //(BLE_TEST_MODE_SUPPORT)

    //the random byte used for incrementing is generated only once
    llm_le_env.aa.intrand = common_rand_byte();
    llm_le_env.aa.ct1_idx = 0;
    llm_le_env.aa.ct2_idx = 0;

    #if (BLE_CENTRAL)
    //Init the default channel map
    memset(&llm_le_env.ch_map_assess.ch_map.map[0],0xFF,4);
    llm_le_env.ch_map_assess.ch_map.map[4] = 0x1F;
    #endif // (BLE_CENTRAL)

    // Init the event
    llm_le_env.elt = NULL;

    // WL init
    llm_wl_clr();

    // disable enhanced privacy feature flag
    SETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE, false);
    // set default timeout value
    llm_le_env.enh_priv_rpa_timeout = LLM_RPA_TIMEOUT_DEFAULT;
    // Initialize Resolving Address List
    llm_ral_clear();
    // clear the timer that manages renewal of resolvable private addresses.
    kernel_timer_clear(LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER, TASK_LLM);
    SETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN, false);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (SECURE_CONNECTIONS)
    llm_le_env.p256_byte_process_timeout = LLM_P256_BYTE_TIMOUT;
    llm_le_env.cur_ecc_multiplication = LLM_ECC_IDLE;
    #endif // (SECURE_CONNECTIONS)
    #if (BLE_2MBPS)
    // By default try to do 2MBPS
    llm_le_env.phys_val.rate_preference         = (ALL_PHYS_TX_NO_PREF | ALL_PHYS_RX_NO_PREF);
    llm_le_env.phys_val.conn_initial_rate_rx    = PHYS_1MBPS_PREF|PHYS_2MBPS_PREF;
    llm_le_env.phys_val.conn_initial_rate_tx    = PHYS_1MBPS_PREF|PHYS_2MBPS_PREF;
    #endif // (BLE_2MBPS)
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    // Init event mask
    llm_le_env.eventmask.mask[0] = LE_DFT_EVT_MSK;
    memset(&llm_le_env.eventmask.mask[1],0,(EVT_MASK_LEN-1));

    // reset the encryption pending flag
    llm_le_env.enc_pend = false;

    #if (BLE_OBSERVER || BLE_CENTRAL)
    llm_le_env.scanning_params = NULL;

    // Init advertising report list
    common_list_init(&llm_le_env.adv_list);
    #endif //(BLE_OBSERVER || BLE_CENTRAL)

    #if (BLE_BROADCASTER || BLE_PERIPHERAL)
    llm_le_env.advertising_params = NULL;
    #endif // BLE_BROADCASTER || BLE_PERIPHERAL

    // Clear the random address
    memset(&llm_le_env.rand_add.addr[0], 0x00, BD_ADDR_LEN);

    // save the current public address
    // WARNING the lld init should be done before the llm init
    common_write32p (&llm_le_env.public_add.addr[0],common_btohl(ble_bdaddrl_get()));
    common_write16p (&llm_le_env.public_add.addr[4],common_btohs(ble_bdaddru_get()));

    // Free the list of queued encryption requests
    while (1)
    {
        // Pop an encryption request from the list
        struct kernel_msg *msg = (struct kernel_msg *)common_list_pop_front(&llm_le_env.enc_req);

        // If no more encryption requests in the list, exit the loop
        if (msg == NULL)
            break;

        // Free the request
        kernel_msg_free(msg);
    }

    // Initialize the list of encryption requests
    common_list_init(&llm_le_env.enc_req);
    // Initialize the list of device(s) connected
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    common_list_init(&llm_le_env.cnx_list);
    // Initialize data length extension default values
    llm_le_env.data_len_val = llm_local_data_len_values;

    #if (BLE_CHNL_ASSESS && BLE_CENTRAL)
    // Channel assessment initialization
    llm_le_env.ch_map_assess.llm_le_set_host_ch_class_cmd_sto = true;

    // Retrieve the Channel Assessment parameters from NVDS or use the default ones
    #if (NVDS_SUPPORT)
    // Channel Assessment - Timer Duration
    buffer_len = NVDS_LEN_BLE_CA_TIMER_DUR;
    if (nvds_get(NVDS_TAG_BLE_CA_TIMER_DUR, &buffer_len,
                 (uint8_t *)&llm_le_env.ch_map_assess.assess_timer) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        llm_le_env.ch_map_assess.assess_timer = LLM_UTIL_CH_ASSES_DFLT_TIMER_DUR;
        if(llm_le_env.ch_map_assess.assess_timer > LLM_UTIL_CH_ASSES_MAX_TIMER_DUR)
        {
            llm_le_env.ch_map_assess.assess_timer = LLM_UTIL_CH_ASSES_MAX_TIMER_DUR;
        }
    }

    #if (NVDS_SUPPORT)
    // Channel Reassessment - Timer Duration
    buffer_len = NVDS_LEN_BLE_CRA_TIMER_CNT;
    if (nvds_get(NVDS_TAG_BLE_CRA_TIMER_CNT, &buffer_len,
                 (uint8_t *)&llm_le_env.ch_map_assess.reassess_count) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        llm_le_env.ch_map_assess.reassess_count = LLM_UTIL_CH_ASSES_DFLT_REASS_CNT;
    }

    #if (NVDS_SUPPORT)
    // Channel Assessment - Minimal Threshold
    buffer_len = NVDS_LEN_BLE_CA_MIN_THR;
    if (nvds_get(NVDS_TAG_BLE_CA_MIN_THR, &buffer_len,
                (uint8_t *)&llm_le_env.ch_map_assess.lower_limit) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        llm_le_env.ch_map_assess.lower_limit = LLM_UTIL_CH_ASSES_DFLT_MIN_THR;
    }

    #if (NVDS_SUPPORT)
    // Channel Assessment - Maximal Threshold
    buffer_len = NVDS_LEN_BLE_CA_MAX_THR;
    if (nvds_get(NVDS_TAG_BLE_CA_MAX_THR, &buffer_len,
                (uint8_t *)&llm_le_env.ch_map_assess.upper_limit) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        llm_le_env.ch_map_assess.upper_limit = LLM_UTIL_CH_ASSES_DFLT_MAX_THR;
    }
    #if (NVDS_SUPPORT)
    // Channel Assessment - Maximal Threshold
    buffer_len = NVDS_LEN_BLE_CA_NOISE_THR;
    if (nvds_get(NVDS_TAG_BLE_CA_NOISE_THR, &buffer_len,
                (uint8_t *)&llm_le_env.ch_map_assess.rssi_noise_limit) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        llm_le_env.ch_map_assess.rssi_noise_limit = LLM_UTIL_CH_ASSES_DFLT_NOISE_THR;
    }
    #endif //(BLE_CHNL_ASSESS && BLE_CENTRAL)
    #endif //(BLE_PERIPHERAL || BLE_CENTRAL)

    //Init deferred command variables
    llm_le_env.state = LLM_IDLE;
    llm_le_env.opcode = 0;
}

void llm_ble_ready(void)
{
    //when the initialization is done send the complete event NOP
    llm_common_cmd_complete_send(HCI_NO_OPERATION_CMD_OPCODE, COMMON_ERROR_NO_ERROR);
}

#if (BLE_PERIPHERAL)
void llm_con_req_ind(uint8_t rx_hdl, uint16_t status)
{
    // Connection Handle
    uint16_t conhdl;
    // Received connection parameters
    struct llm_pdu_con_req_rx data;
    // Connection parameters
    struct llc_create_con_req_ind param;
    em_rd((void*)&data, ble_rxdataptr_get(rx_hdl), LLCP_CON_REQ_LEN);
    do
    {
        uint16_t ral_ptr;
        // Check received parameters
        if ((data.timeout > LLC_CNX_SUP_TO_MAX)    || (data.timeout < LLC_CNX_SUP_TO_MIN)    ||
            #if(BLE_AUDIO)
            (data.interval < AUDIO_MIN_INTERVAL) || (data.interval > LLC_CNX_INTERVAL_MAX) ||
            #else // !(BLE_AUDIO)
            (data.interval < LLC_CNX_INTERVAL_MIN) || (data.interval > LLC_CNX_INTERVAL_MAX) ||
            #endif // (BLE_AUDIO)
            (data.latency > LLC_CNX_LATENCY_MAX)   ||
            // CSA/ESR6 : supervision timeout minimum value does not apply for connection request with a 4.0 device.
            // so supervision timeout must be <=  (1 + Conn_Latency) * Conn_Interval_Max *2
            // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
            // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
            // (sup_to * 10) <= ((1+latency)* con_interval*1.25*2)
            // to simplify computation and remove floating point we factor everything by 2/5
            // (hci_sup_to * 4) <= ((1+hci_latency)* hci_interval)
            ((((uint32_t)data.timeout)<<2) <= ((1+((uint32_t)data.latency)) * ((uint32_t)data.interval))))
        {
            break;
        }

        // Allocates connection handle
        if(llc_util_get_free_conhdl(&conhdl) != COMMON_ERROR_NO_ERROR)
        {
            break;
        }

        // retrieve RAL pointer
        ral_ptr = (status & BLE_PRIV_ERROR_BIT) ? 0 : ble_rxralptr_get(rx_hdl);
        // retrieve peer device identity
        if(ral_ptr != 0)
        {
            uint8_t ral_idx = (ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;
            // Do a burst read in the exchange memory
            em_rd(&(data.inita), ral_ptr + BLE_RAL_PEER_ID_INDEX*2,  BD_ADDR_LEN);
            // gets peer address type
            param.peer_addr_type = ble_peer_id_type_getf(ral_idx) | (ble_peer_irk_valid_getf(ral_idx)<<1);
        }
        else
        {
            // gets the bd_addr type
            param.peer_addr_type = ble_rxtxadd_getf(rx_hdl);
        }


        // Check if the two devices are not already connected
        if(llm_util_bl_check((struct bd_addr *)&data.inita, param.peer_addr_type,
                             &conhdl, LLM_UTIL_BL_NO_ACTION_WL, NULL) == COMMON_ERROR_NO_ERROR)
        {
            uint8_t status = COMMON_ERROR_NO_ERROR;

            // Save the address in the black list (BL).
            status = llm_util_bl_add(&data.inita, param.peer_addr_type, conhdl);

            // if the address cannot be added in the black list avoid the move to slave
            if(status == COMMON_ERROR_NO_ERROR)
            {
                // gets the connection interval
                param.con_int = data.interval;
                // gets the connection latency
                param.con_lat = data.latency;
                // gets supervision time out
                param.sup_to = data.timeout;
                // gets slow clock accuracy
                param.sleep_clk_acc = data.hop_sca  >> 5;
                // gets hop increment
                param.hop_inc = data.hop_sca & 0x1F;
                // retrieve RAL pointer
                param.ral_ptr = ral_ptr;
                // gets the peer address
                memcpy(&param.peer_addr.addr[0],&data.inita.addr[0],BD_ADDR_LEN);

                llm_le_env.elt = lld_move_to_slave(&param, &data, llm_le_env.elt, conhdl);

                llc_start(&param, llm_le_env.elt);
                llm_le_env.elt = NULL;
                // copy the channel map received in the connect request
                llc_util_update_channel_map(conhdl , &data.chm);
            }
            kernel_state_set(TASK_LLM, LLM_IDLE);
        }
    } while (0);
}
#endif // BLE_PERIPHERAL

#if (BLE_OBSERVER || BLE_CENTRAL)
void llm_le_adv_report_ind(uint8_t rx_hdl)
{
    // Enable or disable the transmission of the report to the host
    bool tx_evt;
    // Received advertising type
    uint8_t adv_type = ble_rxtype_getf(rx_hdl);
    // Advertiser address
    struct bd_addr adv_addr;

    em_rd((void*)&adv_addr.addr,ble_rxdataptr_get(rx_hdl),BD_ADDR_LEN);

    ASSERT_ERR(llm_le_env.scanning_params);

    // Checks if the filtering is enabled or packet is a directed advertising packet
    if ((llm_le_env.scanning_params->filter_duplicate == SCAN_FILT_DUPLIC_EN) ||
        (adv_type == LL_ADV_CONN_DIR))
    {
        // Checks if the bd address is not known in the list
        tx_evt = !llm_util_check_adv_report_list(&adv_addr, adv_type);
    }
    else // No filtering
    {
        tx_evt = true;
    }

    // Sends the report(s) to the host
    if (tx_evt)
    {
        // fulfill the field in the event before sending to the host
        llm_adv_report_send(rx_hdl, adv_type);
    }
}
#endif //(BLE_OBSERVER || BLE_CENTRAL)

#if (BLE_CENTRAL)
void llm_con_req_tx_cfm(uint8_t rx_hdl)
{
	bk_printf("%s\r\n", __FUNCTION__);
    uint16_t ral_ptr;
    struct llm_pdu_adv rxdata;
    // fulfill the CONNECT_REQ packet
    struct llm_pdu_con_req_tx txdata;
    // allocate a structure t start the connection
    struct llc_create_con_req_ind param;
    //copy data from EM to system ram
    em_rd((void*)&rxdata,ble_rxdataptr_get(rx_hdl),BD_ADDR_LEN);
    em_rd((void*)&txdata,ble_txdataptr_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX),(LLCP_CON_REQ_LEN-(2*BD_ADDR_LEN)));

    // retrieve RAL pointer
    ral_ptr = ble_rxralptr_get(rx_hdl);
    // retrieve peer device identity
    if(ral_ptr != 0)
    {
        uint8_t ral_idx = (ral_ptr - REG_BLE_EM_RAL_ADDR_GET(0)) / REG_BLE_EM_RAL_SIZE;
        // Do a burst read in the exchange memory
        em_rd(&(param.peer_addr), ral_ptr + BLE_RAL_PEER_ID_INDEX*2,  BD_ADDR_LEN);
        // gets peer address type
        param.peer_addr_type = ble_peer_id_type_getf(ral_idx) | (ble_peer_irk_valid_getf(ral_idx)<<1);
    }
    else
    {
        // gets the bd_addr type
        param.peer_addr_type = ble_rxtxadd_getf(rx_hdl);
        //gets the peer bd_address
        memcpy(&param.peer_addr.addr[0],&rxdata.adva.addr[0],BD_ADDR_LEN);
    }

    // gets all the data needed to start the connection in the CONNECT_REQ packet
    // gets the connection interval
    param.con_int = txdata.interval;
    // gets the connection latency
    param.con_lat = txdata.latency;
    // gets supervision time out
    param.sup_to = txdata.timeout;
    // gets slow clock accuracy
    param.sleep_clk_acc = txdata.hop_sca >> 5;
    // gets hop increment
    param.hop_inc = (txdata.hop_sca & 0x1F);
    // retrieve RAL pointer
    param.ral_ptr = ral_ptr;

    // Check if the cancel connection has been requested
    if(llm_le_env.elt != NULL)
    {
        uint8_t status = COMMON_ERROR_NO_ERROR;

        // Save the address in the black list (BL).
        status = llm_util_bl_add(&param.peer_addr, param.peer_addr_type, llm_le_env.conhdl_alloc);

        // if the address cannot be added in the black list avoid the move to master
        if(status == COMMON_ERROR_NO_ERROR)
        {
            // Confirm connection to LLD so that it programs the first connection event
            llm_le_env.elt = lld_move_to_master(llm_le_env.elt, llm_le_env.conhdl_alloc, &param , rx_hdl);
            llc_start(&param, llm_le_env.elt);
            llm_le_env.elt = NULL;
        }
    }
    //back to IDLE state
    kernel_state_set(TASK_LLM, LLM_IDLE);
}
#endif // BLE_CENTRAL

void llm_common_cmd_complete_send(uint16_t opcode, uint8_t status)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // update the status
    event->status = status;
    // send the message
    hci_send_2_host(event);
}

void llm_common_cmd_status_send(uint16_t opcode, uint8_t status)
{
    // allocate the status event message
    struct hci_cmd_stat_event *event = KERNEL_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // update the status
    event->status = status;
    // send the message
    hci_send_2_host(event);
}


#if (BLE_TEST_MODE_SUPPORT)
uint8_t llm_test_mode_start_tx(void const *param, kernel_msg_id_t const msgid)
{
    // Exchange memory buffer to be filled
    struct em_buf_node *tx_buff_node = em_buf_tx_alloc();
    struct em_desc_node* tx_desc_node = em_buf_tx_desc_alloc();
    // System ram buffer to be filled
    uint8_t data[BLE_TESTMODE_MAX_OCTETS];

    if(tx_buff_node && tx_desc_node)
    {
        ble_txdataptr_set(tx_desc_node->idx, tx_buff_node->buf_ptr);
        ble_freebuff_setf(tx_desc_node->idx, (uint8_t)true);
        // set the task in test tx mode state
        kernel_state_set(TASK_LLM, LLM_TEST);
        //disable the whitening
        ble_whit_dsb_setf(1);


        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        if (!llm_le_env.advertising_params)
        {
            llm_util_set_param_adv_dft();
        }
        #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

        #if ((BLE_2MBPS) && !(BLE_QUALIF))
        if (msgid == HCI_LE_ENH_TX_TEST_CMD_OPCODE)
        {
            struct hci_le_enh_tx_test_cmd* tx_test_mode = (struct hci_le_enh_tx_test_cmd*)param;
            //check the type of test
            switch(tx_test_mode->payload_type)
            {
                case PAYL_PSEUDO_RAND_9:
                case PAYL_PSEUDO_RAND_15:
                    // sets the type of the PRBS
                    ble_prbstype_setf(tx_test_mode->payload_type & 0x1);
                    // sets the source to PRBS generator
                    ble_txpldsrc_setf(1);
                    break;
                case PAYL_11110000:
                case PAYL_10101010:
                case PAYL_ALL_1:
                case PAYL_ALL_0:
                case PAYL_00001111:
                case PAYL_01010101:
                    llm_util_gen_pattern(tx_test_mode->payload_type, tx_test_mode->test_data_length, data);
                    em_wr((void *)&data, tx_buff_node->buf_ptr, tx_test_mode->test_data_length);
                    // sets the source to CS
                    ble_txpldsrc_setf(0);
                    break;
                default:
                    ASSERT_ERR(tx_test_mode->payload_type < PAYL_END);
                    break;
            }
            // set the pattern type and the length in the header
            ble_txphadv_pack(tx_desc_node->idx,// Descriptor index
                    tx_test_mode->test_data_length,   // Packet length
                    0,                      //Don't care txrxadd
                    0,                      //Don't care txtxadd,
                    tx_test_mode->payload_type);// Payload type

            if( (tx_test_mode->tx_channel > RX_TEST_FREQ_MAX)
                    ||(tx_test_mode->test_data_length == 0)
                    ||(tx_test_mode->payload_type > PAYL_01010101)
                    ||(tx_test_mode->phy == PHYS_NO_PREF)
                    ||(tx_test_mode->phy > PHYS_2MBPS_PREF))
            {
                // set the task in IDLE
                kernel_state_set(TASK_LLM, LLM_IDLE);
                return (COMMON_ERROR_PARAM_OUT_OF_MAND_RANGE);
            }

            // request to the HW to start the tx test mode
            llm_le_env.elt = lld_test_mode_tx(true, em_buf_node_get(tx_desc_node->idx), tx_test_mode->tx_channel, tx_test_mode->phy);
        }
        else
        #endif
        {
            struct hci_le_tx_test_cmd* tx_test_mode = (struct hci_le_tx_test_cmd*)param;
            //check the type of test
            switch(tx_test_mode->pk_payload_type)
            {
                case PAYL_PSEUDO_RAND_9:
                case PAYL_PSEUDO_RAND_15:
                    // sets the type of the PRBS
                    ble_prbstype_setf(tx_test_mode->pk_payload_type & 0x1);
                    // sets the source to PRBS generator
                    ble_txpldsrc_setf(1);
                    break;
                case PAYL_11110000:
                case PAYL_10101010:
                case PAYL_ALL_1:
                case PAYL_ALL_0:
                case PAYL_00001111:
                case PAYL_01010101:
                    llm_util_gen_pattern(tx_test_mode->pk_payload_type, tx_test_mode->test_data_len, data);
                    em_wr((void *)&data, tx_buff_node->buf_ptr, tx_test_mode->test_data_len);
                    // sets the source to CS
                    ble_txpldsrc_setf(0);
                    break;
                default:
                    ASSERT_ERR(tx_test_mode->pk_payload_type < PAYL_END);
                    break;
            }
            // set the pattern type and the length in the header
            ble_txphadv_pack(tx_desc_node->idx,// Descriptor index
                    tx_test_mode->test_data_len,   // Packet length
                    0,                      //Don't care txrxadd
                    0,                      //Don't care txtxadd,
                    tx_test_mode->pk_payload_type);// Payload type

            if( (tx_test_mode->tx_freq > RX_TEST_FREQ_MAX)
                    ||(tx_test_mode->test_data_len == 0)
                    ||(tx_test_mode->pk_payload_type > PAYL_01010101))
            {
                // set the task in IDLE
                kernel_state_set(TASK_LLM, LLM_IDLE);
                return (COMMON_ERROR_INVALID_HCI_PARAM);
            }
            // request to the HW to start the tx test mode
            llm_le_env.elt = lld_test_mode_tx(false, em_buf_node_get(tx_desc_node->idx), tx_test_mode->tx_freq, 0);
        }

        if(llm_le_env.elt == NULL)
        {

            // set the task in IDLE
            kernel_state_set(TASK_LLM, LLM_IDLE);
            //re-enable the whitening
            ble_whit_dsb_setf(0);

            return (COMMON_ERROR_UNSPECIFIED_ERROR);
        }
        else
        {
            // sets the test mode type
            llm_le_env.test_mode.directtesttype = TEST_TX;
            return (COMMON_ERROR_NO_ERROR);
        }
    }
    else
    {
        return (COMMON_ERROR_MEMORY_CAPA_EXCEED);
    }
}

uint8_t llm_test_mode_start_rx(void const *param, kernel_msg_id_t const msgid)
{

    #if (BLE_PERIPHERAL || BLE_BROADCASTER)
    if (!llm_le_env.advertising_params)
    {
        llm_util_set_param_adv_dft();
    }
    #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

    //disable the whitening
    ble_whit_dsb_setf(1);
    // set the task in test rx mode state
    kernel_state_set(TASK_LLM, LLM_TEST);

    if(msgid == HCI_LE_RX_TEST_CMD_OPCODE)
    {
        struct hci_le_rx_test_cmd* rx_test_cmd = (struct hci_le_rx_test_cmd*)param;

        if(rx_test_cmd->rx_freq > RX_TEST_FREQ_MAX)
        {
            // set the task in IDLE
            kernel_state_set(TASK_LLM, LLM_IDLE);
            return (COMMON_ERROR_PARAM_OUT_OF_MAND_RANGE);
        }
        // request to the HW to start the rx test mode
        llm_le_env.elt = lld_test_mode_rx(false, rx_test_cmd->rx_freq, 0, 0);
    }
    #if ((BLE_2MBPS) && !(BLE_QUALIF))
    else if (msgid == HCI_LE_ENH_RX_TEST_CMD_OPCODE)
    {
        struct hci_le_enh_rx_test_cmd* rx_enh_test_cmd = (struct hci_le_enh_rx_test_cmd*)param;
        if(rx_enh_test_cmd->rx_channel > RX_TEST_FREQ_MAX)
        {
            // set the task in IDLE
            kernel_state_set(TASK_LLM, LLM_IDLE);
            return (COMMON_ERROR_PARAM_OUT_OF_MAND_RANGE);
        }

        // request to the HW to start the rx test mode
        llm_le_env.elt = lld_test_mode_rx(true, rx_enh_test_cmd->rx_channel, rx_enh_test_cmd->phy, rx_enh_test_cmd->modulation_idx);
    }
    #endif // (BLE_2MBPS)
    else
    {
        //Should not be enter here
        ASSERT_INFO(0, msgid, msgid);
    }


    if(llm_le_env.elt == NULL)
    {
        //Re-enable the whitening
        ble_whit_dsb_setf(0);
        // set the task in test rx mode state
        kernel_state_set(TASK_LLM, LLM_IDLE);

        return (COMMON_ERROR_UNSPECIFIED_ERROR);
    }
    else
    {
        // sets the test mode type
        llm_le_env.test_mode.directtesttype = TEST_RX;
        return (COMMON_ERROR_NO_ERROR);
    }

}
#endif // (BLE_TEST_MODE_SUPPORT)

#if (BLE_BROADCASTER || BLE_PERIPHERAL)
uint8_t llm_set_adv_param(struct hci_le_set_adv_param_cmd const *param)
{
    #if (BLE_PERIPHERAL)
    uint8_t status;
    #endif //(BLE_PERIPHERAL)

    if ((param->adv_intv_min > param->adv_intv_max) ||
        (param->adv_chnl_map > ADV_ALL_CHNLS_EN) || (param->adv_filt_policy >= ADV_ALLOW_SCAN_END ))
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    //The advInterval shall be an integer multiple of 0.625 ms in the range of 20 ms
    //to 10.24 s. If the advertising event type is either a discoverable undirected
    //event type or a non-connectable undirected event type, the advInterval shall
    //not be less than 100 ms. If the advertising event type is a connectable undirected
    //event type, the advInterval can be 20 ms or greater.
    if ((((param->adv_type == ADV_CONN_UNDIR)      || (param->adv_type == ADV_CONN_DIR_LDC))  && (param->adv_intv_min < LLM_ADV_INTERVAL_MIN))
     || (((param->adv_type == ADV_DISC_UNDIR)      || (param->adv_type == ADV_NONCONN_UNDIR)) && (param->adv_intv_min < LLM_ADV_INTERVAL_MIN_NONCON_DISC))
     || (param->own_addr_type > ADDR_RPA_OR_RAND)  || (param->peer_addr_type > ADDR_RAND)
     || ((param->adv_type != ADV_CONN_DIR)         && (param->adv_intv_max > LLM_ADV_INTERVAL_MAX)))
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    if (!llm_le_env.advertising_params)
    {
          llm_util_set_param_adv_dft();
    }

    llm_le_env.advertising_params->type = LL_ADV_END;
    llm_le_env.advertising_params->adv_ldc_flag = true;

    switch (param->adv_type)
    {
        #if (BLE_PERIPHERAL)
        // direct advertising is performed
        case ADV_CONN_DIR:
        case ADV_CONN_DIR_LDC:
            llm_le_env.advertising_params->type = LL_ADV_CONN_DIR;
            // sets the type of advertising
            ble_txphadv_pack(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX,   // Descriptor index
                    0xC,                                        // Packet length
                    0,                                          //Don't care txrxadd
                    0,                                          //Don't care txtxadd,
                    llm_le_env.advertising_params->type);       // Payload type

            status = llm_util_check_address_validity((struct bd_addr*)&(param->peer_addr),
                    param->peer_addr_type);

            // If the address is available update the environment variable
            if (status == COMMON_ERROR_NO_ERROR)
            {
                // sets the address type of the initiator
                ble_txrxadd_setf(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX, param->peer_addr_type);
                // sets the initiator address
                em_wr((void *)&param->peer_addr.addr[0], ble_txdataptr_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX), BD_ADDR_LEN);
            }

            if (param->adv_type == ADV_CONN_DIR_LDC)
            {
                llm_le_env.advertising_params->intervalmax = param->adv_intv_max;
                llm_le_env.advertising_params->intervalmin = param->adv_intv_min;
            }
            else
            {
                llm_le_env.advertising_params->adv_ldc_flag = false;
            }
            break;
        #endif // BLE_PERIPHERAL
        case ADV_DISC_UNDIR:
            llm_le_env.advertising_params->type = LL_ADV_DISC_UNDIR;
            // no break
        case ADV_CONN_UNDIR:
            if(llm_le_env.advertising_params->type != LL_ADV_DISC_UNDIR)
            {
                llm_le_env.advertising_params->type = LL_ADV_CONN_UNDIR;
            }
            // no break
        case ADV_NONCONN_UNDIR:
            if( (llm_le_env.advertising_params->type != LL_ADV_DISC_UNDIR)
                && (llm_le_env.advertising_params->type != LL_ADV_CONN_UNDIR))
            {
                llm_le_env.advertising_params->type = LL_ADV_NONCONN_UNDIR;
            }
            llm_le_env.advertising_params->filterpolicy
                    = param->adv_filt_policy;
            llm_le_env.advertising_params->intervalmax
                    = param->adv_intv_max;
            llm_le_env.advertising_params->intervalmin
                    = param->adv_intv_min;

            break;
        default:
            return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    // Store peer address information
    llm_le_env.advertising_params->peer_addr_type = param->peer_addr_type;
    memcpy(&(llm_le_env.advertising_params->peer_addr), &(param->peer_addr), BD_ADDR_LEN);

    // Store own address type for advertising
    llm_le_env.advertising_params->own_addr_type = param->own_addr_type;

    if (llm_util_check_map_validity((uint8_t *) &param->adv_chnl_map, 1) != 0)
    {
        llm_le_env.advertising_params->channelmap = param->adv_chnl_map;
        return COMMON_ERROR_NO_ERROR;
    }
    else
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }
}

uint8_t llm_set_adv_en(struct hci_le_set_adv_en_cmd const *param)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    // if the advertising mode is enabled
    if(param->adv_en)
    {
        if (!llm_le_env.advertising_params)
        {
            llm_util_set_param_adv_dft();
        }

        //Check if the address type is static private and no static private address is available
        if((common_bdaddr_compare((struct bd_addr *)&llm_le_env.rand_add.addr[0], &common_null_bdaddr) == true) &&
                ((llm_le_env.advertising_params->own_addr_type & ADDR_MASK) == ADDR_RAND))
        {
            return (COMMON_ERROR_INVALID_HCI_PARAM);
        }

        // Configure the BD address for advertising
        llm_util_apply_bd_addr(llm_le_env.advertising_params->own_addr_type);

        // check the type of advertising
        switch (llm_le_env.advertising_params->type)
        {
            case LL_ADV_CONN_UNDIR:
            case LL_ADV_DISC_UNDIR:
            case LL_ADV_NONCONN_UNDIR:
                // sets the type of advertising, the length
                ble_txphadv_pack(LLM_LE_ADV_IDX,                                // Descriptor index
                        (BD_ADDR_LEN + llm_le_env.advertising_params->datalen), // Packet length
                        0,                                                      // Don't care txrxadd
                        0,                                                      // Don't care txtxadd,
                        llm_le_env.advertising_params->type);                   // Payload type


                if(llm_le_env.advertising_params->type == LL_ADV_NONCONN_UNDIR)
                {
                    llm_le_env.elt = lld_adv_start(llm_le_env.advertising_params,
                                                   em_buf_node_get(LLM_LE_ADV_IDX),
                                                   NULL,
                                                   LLM_ADV_CHANNEL_TXPWR);
                    if(llm_le_env.elt == NULL)
                    {
                        return (COMMON_ERROR_INVALID_HCI_PARAM);
                    }
                }
                else
                {
                    // prepare the scan response header
                    ble_txphadv_pack(LLM_LE_SCAN_RSP_IDX,                                    // Descriptor index
                            (BD_ADDR_LEN + llm_le_env.advertising_params->scanrsplen),  // Packet length
                            0,                                                          // Don't care txrxadd
                            0,                                                          // Don't care txtxadd,
                            LL_SCAN_RSP);                                               // Payload type


                    llm_le_env.elt = lld_adv_start(llm_le_env.advertising_params,
                                                   em_buf_node_get(LLM_LE_ADV_IDX),
                                                   em_buf_node_get(LLM_LE_SCAN_RSP_IDX),
                                                   LLM_ADV_CHANNEL_TXPWR);
					//UART_PRINTF("lld_adv_start \r\n");
                    if(llm_le_env.elt == NULL)
                    {
                        return  (COMMON_ERROR_UNSPECIFIED_ERROR);
                    }
                }
                break;

            #if (BLE_PERIPHERAL)
            case LL_ADV_CONN_DIR:
                llm_le_env.elt = lld_adv_start(llm_le_env.advertising_params,
                                               em_buf_node_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX),
                                               NULL,
                                               LLM_ADV_CHANNEL_TXPWR);
						
						
                if(llm_le_env.elt == NULL)
                {
                    return (COMMON_ERROR_INVALID_HCI_PARAM);
                }
                break;
            #endif // BLE_PERIPHERAL

            default:
                return (COMMON_ERROR_INVALID_HCI_PARAM);
        }
        // set the state to advertising
        kernel_state_set(TASK_LLM, LLM_ADVERTISING);

        // update state of the resolving address list
        llm_ral_update();
    }
    else // disabled
    {
        // Set the state to stopping
        kernel_state_set(TASK_LLM, LLM_STOPPING);
        // Stop the scanning
        lld_adv_stop(llm_le_env.elt);
    }
    return (status);
}


uint8_t llm_set_adv_data(struct hci_le_set_adv_data_cmd const *param)
{
    if (param->adv_data_len > ADV_DATA_LEN)
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }
    else
    {
        if (!llm_le_env.advertising_params)
        {
            llm_util_set_param_adv_dft();
        }

         // adds the length of the data
        llm_le_env.advertising_params->datalen = param->adv_data_len;
        // copy the data in the tx buffer
        if (param->adv_data_len > 0)
        {
            em_wr((void *)&param->data.data[0], ble_txdataptr_get(LLM_LE_ADV_IDX), param->adv_data_len);
        }
        // sets the length of advertising data
        ble_txadvlen_setf(LLM_LE_ADV_IDX,(BD_ADDR_LEN + llm_le_env.advertising_params->datalen));  // Packet length

    }
    return (COMMON_ERROR_NO_ERROR);
}

uint8_t llm_set_scan_rsp_data(struct hci_le_set_scan_rsp_data_cmd const *param)
{
    if (param->scan_rsp_data_len > SCAN_RSP_DATA_LEN)
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }
    else
    {
        if (!llm_le_env.advertising_params)
        {
            llm_util_set_param_adv_dft();
        }
        // save the length of the data
        llm_le_env.advertising_params->scanrsplen= param->scan_rsp_data_len;
        // copy the data in the tx buffer
        if (param->scan_rsp_data_len > 0)
        {
            em_wr((void *)&param->data.data[0], ble_txdataptr_get(LLM_LE_SCAN_RSP_IDX), param->scan_rsp_data_len);
        }
    }
    // Update the scan response header
    ble_txadvlen_setf(LLM_LE_SCAN_RSP_IDX,(BD_ADDR_LEN + llm_le_env.advertising_params->scanrsplen));
    return (COMMON_ERROR_NO_ERROR);
}
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)

#if (BLE_OBSERVER || BLE_CENTRAL)
uint8_t llm_set_scan_param(struct hci_le_set_scan_param_cmd const *param)
{
    // Check provided parameters
    if ((param->scan_window > param->scan_intv) || (param->scan_type > SCAN_BLE_ACTIVE) || (param->scan_window > LLM_SCAN_WINDOW_MAX) ||
        (param->scan_window < LLM_SCAN_WINDOW_MIN) || (param->scan_intv > LLM_SCAN_INTERVAL_MAX) || (param->scan_intv < LLM_SCAN_INTERVAL_MIN)
        || (param->scan_filt_policy >= SCAN_ALLOW_ADV_END))
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    // check privacy -> scan with filter policy 2 & 3 RAL must be enabled
    if((!GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE))
            && ((param->scan_filt_policy > SCAN_ALLOW_ADV_WLST) || ((param->own_addr_type & ADDR_RPA_MASK) != 0)))
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    if (!llm_le_env.scanning_params)
    {
        // Allocate a structure containing scan parameters if needed
        llm_le_env.scanning_params = (struct scanning_pdu_params *)kernel_malloc(sizeof(struct scanning_pdu_params),
                                                                             KERNEL_MEM_ENV);

        memset(llm_le_env.scanning_params, 0, sizeof(struct scanning_pdu_params));
    }

    // Save all the command parameters in the environment variable
    llm_le_env.scanning_params->filterpolicy  = param->scan_filt_policy;
    llm_le_env.scanning_params->interval      = param->scan_intv;
    llm_le_env.scanning_params->window        = param->scan_window;
    llm_le_env.scanning_params->type          = param->scan_type;
    llm_le_env.scanning_params->own_addr_type = param->own_addr_type;
    llm_le_env.scanning_params->channel_map = param->channel_map;

    if(param->scan_type == SCAN_BLE_ACTIVE)
    {
        // prepare the scan response header
        ble_txphadv_pack(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX,   // Descriptor index
                12,                                         // Packet length
                0,                                          // Don't care txrxadd
                0,                                          // Don't care txtxadd,
                LL_SCAN_REQ);                               // Payload type
    }
    // return status OK
    return (COMMON_ERROR_NO_ERROR);
}

uint8_t llm_set_scan_en(struct hci_le_set_scan_en_cmd const *param)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    if (param->scan_en == SCAN_DIS)
    {
        // Set the state to stopping
        kernel_state_set(TASK_LLM, LLM_STOPPING);

        // Stop the scanning
        lld_scan_stop(llm_le_env.elt);

        // Clear advertising report list
        lld_util_flush_list(&llm_le_env.adv_list);
    }
    else
    {
        if (!llm_le_env.scanning_params)
        {
            // Use scan default parameters
            llm_util_set_param_scan_dft();
        }

        //Check if the address type is static private and no static private address is available and Scan is active
        if((common_bdaddr_compare((struct bd_addr *)&llm_le_env.rand_add.addr[0], &common_null_bdaddr) == true)
                && ((llm_le_env.scanning_params->own_addr_type & ADDR_MASK) == ADDR_RAND)
                && (llm_le_env.scanning_params->type == SCAN_BLE_ACTIVE))
        {
            return  (COMMON_ERROR_INVALID_HCI_PARAM);
        }

        // Set the filter duplicated option
        llm_le_env.scanning_params->filter_duplicate  = param->filter_duplic_en;

        // Configure the BD address for scanning
        llm_util_apply_bd_addr(llm_le_env.scanning_params->own_addr_type);

        // Check the type of advertising
        switch (llm_le_env.scanning_params->type)
        {
            case SCAN_BLE_PASSIVE:
            {
                llm_le_env.elt = lld_scan_start(llm_le_env.scanning_params, NULL);
                if(llm_le_env.elt == NULL)
                {
                    return (COMMON_ERROR_UNSPECIFIED_ERROR);
                }
            } break;

            case SCAN_BLE_ACTIVE:
            {
                llm_le_env.elt = lld_scan_start(llm_le_env.scanning_params,
                                     em_buf_node_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX));
                if(llm_le_env.elt == NULL)
                {
                    return (COMMON_ERROR_UNSPECIFIED_ERROR);
                }
            } break;

            default:
            {
                // Do nothing
            } break;
        }

        // update state of the resolving address list
        llm_ral_update();

        // Set the state to scanning
        kernel_state_set(TASK_LLM, LLM_SCANNING);
    }

    return (status);
}
#endif //(BLE_OBSERVER || BLE_CENTRAL)

void llm_wl_clr(void)
{
    uint16_t nb_elem;

    // clear private WL
    for (nb_elem = 0; nb_elem < BLE_WHITELIST_MAX; nb_elem++)
    {
        llm_wlpriv_addr_set(nb_elem, &llm_dflt_bdaddr);
    }
    // clear public WL
    for (nb_elem = 0; nb_elem < BLE_WHITELIST_MAX; nb_elem++)
    {
        llm_wlpub_addr_set(nb_elem, &llm_dflt_bdaddr);
    }

    // ensure that white list present flag in Resolving list correctly removed
    for(nb_elem = 0 ; nb_elem < BLE_RESOL_ADDR_LIST_MAX ; nb_elem++)
    {
        ble_in_whlist_setf(nb_elem, 0);
    }

    llm_le_env.nb_dev_in_wl    = 0;
    llm_le_env.nb_dev_in_hw_wl = 0;
}



void llm_wl_dev_add(struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint16_t position = llm_util_bd_addr_wl_position(&llm_dflt_bdaddr, bd_addr_type);

    switch (bd_addr_type)
    {
        // public case
        case ADDR_PUBLIC:
            llm_wlpub_addr_set(position, bd_addr);
            break;
            // private case
        case ADDR_RAND:
            llm_wlpriv_addr_set(position, bd_addr);
            break;
        default: /* Nothnig to do, already checked */ break;
    }

    llm_le_env.nb_dev_in_hw_wl += 1;
}

void llm_wl_dev_rem(struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    // Check if the BD address is in WL an get the position
    uint16_t position = llm_util_bd_addr_wl_position(bd_addr, bd_addr_type);

    if (position < BLE_WHITELIST_MAX)
    {
        // if it is a private address
        if (bd_addr_type == ADDR_RAND)
        {
            llm_wlpriv_addr_set(position, &llm_dflt_bdaddr);
        }
        else
        {
            llm_wlpub_addr_set(position, &llm_dflt_bdaddr);
        }

        llm_le_env.nb_dev_in_hw_wl -= 1;
    }

}

uint8_t llm_wl_dev_add_hdl(struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;

    do
    {
        uint8_t  ral_pos;

        if(bd_addr_type > ADDR_RAND)
        {
            status = COMMON_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Check if the BD address is already in the WL
        if (llm_util_bd_addr_in_wl(bd_addr, bd_addr_type, NULL))
        {
            status = COMMON_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Look for a free space in the white list
        if (llm_le_env.nb_dev_in_wl >= BLE_WHITELIST_MAX)
        {
            // No free space anymore
            status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
            break;
        }

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        // Check black list
        if(llm_util_bl_check(bd_addr, bd_addr_type, NULL, LLM_UTIL_BL_SET_WL, NULL) != COMMON_ERROR_ACL_CON_EXISTS)
        #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
        {
            llm_wl_dev_add(bd_addr, bd_addr_type);
        }

        llm_le_env.nb_dev_in_wl += 1;

        // check if device present in white list
        if(llm_util_bd_addr_in_ral(bd_addr, bd_addr_type, &ral_pos))
        {
            ble_in_whlist_setf(ral_pos, 1);
        }

    } while (0);

    return (status);
}


uint8_t llm_wl_dev_rem_hdl(struct bd_addr *bd_addr, uint8_t bd_addr_type)
{
    uint8_t  ral_pos;
    bool in_black_list;

    if(bd_addr_type > ADDR_RAND)
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }

    // Check if the BD address is in WL an get the position
    if (llm_util_bd_addr_in_wl(bd_addr, bd_addr_type, &in_black_list))
    {
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        // check if device is present in black list
        if(in_black_list)
        {
            llm_util_bl_check(bd_addr, bd_addr_type, NULL, LLM_UTIL_BL_CLEAR_WL, NULL);
        }
        else
        #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
        {
            llm_wl_dev_rem(bd_addr, bd_addr_type);
        }

        // check if device present in white list
        if(llm_util_bd_addr_in_ral(bd_addr, bd_addr_type, &ral_pos))
        {
            ble_in_whlist_setf(ral_pos, 0);
        }

        llm_le_env.nb_dev_in_wl -= 1;

        return (COMMON_ERROR_NO_ERROR);
    }
    else
    {
        return (COMMON_ERROR_INVALID_HCI_PARAM);
    }
}



#if (BLE_CENTRAL)
uint8_t llm_create_con(struct hci_le_create_con_cmd const *param)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    do
    {

        if(llc_util_get_free_conhdl(&llm_le_env.conhdl_alloc) != COMMON_ERROR_NO_ERROR)
        {
            // Lack of resources
            status = COMMON_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        if(param->init_filt_policy != INIT_FILT_USE_WLST)
        {
            status = llm_util_bl_check((struct bd_addr *)&param->peer_addr, param->peer_addr_type,
                                       &llm_le_env.conhdl_alloc, LLM_UTIL_BL_NO_ACTION_WL, NULL);
        }
        // check if there are some devices in HW white list
        else if(llm_le_env.nb_dev_in_hw_wl == 0)
        {
            status = (llm_le_env.nb_dev_in_wl == 0) ? COMMON_ERROR_COMMAND_DISALLOWED : COMMON_ERROR_ACL_CON_EXISTS;
        }


        // Status of the connection
        if(status == COMMON_ERROR_NO_ERROR)
        {
            // parameter checking
            if ((param->scan_window > param->scan_intv)      || (param->con_intv_min > param->con_intv_max) ||
                (param->scan_window > LLM_SCAN_WINDOW_MAX)   || (param->scan_window < LLM_SCAN_WINDOW_MIN)  ||
                (param->scan_intv > LLM_SCAN_INTERVAL_MAX)   || (param->scan_intv < LLM_SCAN_INTERVAL_MIN)  ||
                (param->ce_len_min > param->ce_len_max)      || (param->superv_to > LLC_CNX_SUP_TO_MAX)     ||
                #if(BLE_AUDIO)
                (param->con_intv_min < AUDIO_MIN_INTERVAL) || (param->con_intv_max > LLC_CNX_INTERVAL_MAX)  ||
                #else // !(BLE_AUDIO)
                (param->con_intv_min < LLC_CNX_INTERVAL_MIN) || (param->con_intv_max > LLC_CNX_INTERVAL_MAX)||
                #endif // (BLE_AUDIO)
                (param->superv_to < LLC_CNX_SUP_TO_MIN)      || (param->con_latency > LLC_CNX_LATENCY_MAX)  ||
                // CSA/ESR6 : supervision timeout minimum value does not apply for connection request with a 4.0 device.
                // so supervision timeout must be <=  (1 + Conn_Latency) * Conn_Interval_Max *2
                // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
                // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
                // (sup_to * 10) <= ((1+latency)* con_interval*1.25*2)
                // to simplify computation and remove floating point we factor everything by 2/5
                // (hci_sup_to * 4) <= ((1+hci_latency)* hci_interval)
                ((((uint32_t) param->superv_to)<<2) <= ((1+((uint32_t)param->con_latency)) * ((uint32_t)param->con_intv_max))))
            {
                // Invalid parameters
                status = COMMON_ERROR_INVALID_HCI_PARAM;
                break;
            }
            //Check if the address type is static private and no static private address is available
            if((common_bdaddr_compare((struct bd_addr *)&llm_le_env.rand_add.addr[0], &common_null_bdaddr) == true) &&
               ((param->own_addr_type & ADDR_MASK) == ADDR_RAND))
            {
                // Invalid parameters
                status = COMMON_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // check privacy -> if RAL disabled, ensure that own address and peer address are not RPA
            if(!GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE)
                    && (((param->own_addr_type & ADDR_RPA_MASK) != 0)
                            || ((param->peer_addr_type & ADDR_RPA_MASK) != 0)))
            {
                return (COMMON_ERROR_INVALID_HCI_PARAM);
            }

            // check privacy -> for own address type 0/1 peer address type shall be 0/1
            if(((param->own_addr_type & ADDR_RPA_MASK) == 0)
                    && ((param->peer_addr_type & ADDR_RPA_MASK) != 0))
            {
                return (COMMON_ERROR_INVALID_HCI_PARAM);
            }

            // check privacy -> peer address type is 2/3, Address shall be in resolving list and peer IRK valid
            if((param->peer_addr_type & ADDR_RPA_MASK) != 0)
            {
                    uint8_t ral_idx;

                    // retrieve Resolving list index
                    if(llm_util_bd_addr_in_ral(&(param->peer_addr), param->peer_addr_type, &ral_idx))
                    {
                        // check that peer IRK is valid
                        if(!ble_peer_irk_valid_getf(ral_idx))
                        {
                            return (COMMON_ERROR_INVALID_HCI_PARAM);
                        }
                    }
                    else
                    {
                        return (COMMON_ERROR_INVALID_HCI_PARAM);
                    }
            }

            // goes in INITIATING state
            kernel_state_set(TASK_LLM, LLM_INITIATING);
            // Request the LLD to schedule the connection initiation
            llm_le_env.elt = lld_con_start(param, em_buf_node_get(LLM_LE_SCAN_CON_REQ_ADV_DIR_IDX),
                                        llm_le_env.conhdl_alloc);

            if(llm_le_env.elt == NULL)
            {
                status = COMMON_ERROR_UNSPECIFIED_ERROR;
            }
        }
    } while(0);

    if(status != COMMON_ERROR_NO_ERROR)
    {
        // update state of the resolving address list
        llm_ral_update();
    }

    return (status);
}
#endif // BLE_CENTRAL


void llm_encryption_done(void)
{
    struct kernel_msg *msg = (struct kernel_msg *)common_list_pop_front(&llm_le_env.enc_req);

    DBG_SWDIAG(EVT, BLE_CRYPT, 1);
    // Clear the kernel event
    kernel_event_clear(KERNEL_EVENT_BLE_CRYPT);

    // Check if the popped message is valid. If the message is NULL it means that the LLM
    // was reset during an encryption and therefore we can exit immediately the function
    if (msg == NULL)
    {
        DBG_SWDIAG(EVT, BLE_CRYPT, 0);
        return;
    }

    // For requests coming from the host, the encrypted data must be swapped
    if(KERNEL_TYPE_GET(msg->src_id) != TASK_LLC)
    {
        // Allocate the message for the response
        struct hci_le_enc_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_ENC_CMD_OPCODE, hci_le_enc_cmd_cmp_evt);

        // copy data from em to sys ram
        em_rd(&(event->encrypted_data[0]), EM_BLE_ENC_CIPHER_OFFSET, ENC_DATA_LEN);

        // Set the status
        event->status = COMMON_ERROR_NO_ERROR;

        // Send the event
        hci_send_2_host(event);
    }
    else
    {
        // Allocate the message for the response
        struct llm_enc_ind *ind = KERNEL_MSG_ALLOC(LLM_ENC_IND, msg->src_id, TASK_LLM, llm_enc_ind);

        // copy data from em to sys ram
        em_rd(&ind->encrypted_data[0], EM_BLE_ENC_CIPHER_OFFSET, ENC_DATA_LEN);

        // Set the status
        ind->status = COMMON_ERROR_NO_ERROR;

        // Send the indication
        kernel_msg_send(ind);
    }

    // Free the message
    kernel_msg_free(msg);

    // Check if a new encryption has to be launched
    msg = (struct kernel_msg *)common_list_pick(&llm_le_env.enc_req);
    if (msg !=  NULL)
    {
        // Start the encryption
        llm_encryption_start((struct llm_enc_req *)kernel_msg2param(msg));
    }
    else
    {
        #if (DEEP_SLEEP)
        // Allow again the deep sleep
        rwip_prevent_sleep_clear(RW_CRYPT_ONGOING);
			//	rom_env.rwip_prevent_sleep_clear(RW_CRYPT_ONGOING);
        #endif //DEEP_SLEEP

        // The encryption engine is now IDLE
        llm_le_env.enc_pend = false;
    }

    DBG_SWDIAG(EVT, BLE_CRYPT, 0);
}

void llm_encryption_start(struct llm_enc_req const *param)
{
    // copy the key in the register dedicated for the encryption
    ble_aeskey31_0_set(common_read32p(&param->key.ltk[0]));
    ble_aeskey63_32_set(common_read32p(&param->key.ltk[4]));
    ble_aeskey95_64_set(common_read32p(&param->key.ltk[8]));
    ble_aeskey127_96_set(common_read32p(&param->key.ltk[12]));

    // copy data from sys ram to em
    em_wr(&param->plain_data[0], EM_BLE_ENC_PLAIN_OFFSET, EM_BLE_ENC_LEN);

    // set the pointer on the data to encrypt.
    ble_aesptr_set(EM_BLE_ENC_PLAIN_OFFSET);

    // start the encryption
    ble_aescntl_set(BLE_AES_START_BIT);

    // An encryption is now pending
    llm_le_env.enc_pend = true;
    #if (DEEP_SLEEP)
    // Prevent from going to deep sleep
    rwip_prevent_sleep_set(RW_CRYPT_ONGOING);
    #endif //DEEP_SLEEP
}

void llm_ral_clear(void)
{
    uint8_t i;
    for(i = 0 ; i < BLE_RESOL_ADDR_LIST_MAX ; i++)
    {
        ble_entry_valid_setf(i, 0);
    }
}

uint8_t llm_ral_dev_add(struct hci_le_add_dev_to_rslv_list_cmd * param)
{
    uint8_t position;
    uint8_t status = COMMON_ERROR_NO_ERROR;

    if(param->peer_id_addr_type > ADDR_RAND)
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
    else if(llm_util_bd_addr_in_ral(&(param->peer_id_addr), param->peer_id_addr_type, &position))
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
    else if(position == BLE_RESOL_ADDR_LIST_MAX)
    {
        status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
    }
    else
    {
        uint8_t empty_key[KEY_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        bool l_irk_valid = (memcmp(param->local_irk.key, empty_key, KEY_LEN) != 0);
        bool p_irk_valid = (memcmp(param->peer_irk.key,  empty_key, KEY_LEN) != 0);
        bool connected = false;
        // check if address is in white list.
        bool in_wl = llm_util_bd_addr_in_wl(&(param->peer_id_addr), param->peer_id_addr_type, &connected);

        // Do a burst write in the exchange memory
        em_wr(&(param->peer_id_addr), REG_BLE_EM_RAL_ADDR_GET(position) + BLE_RAL_PEER_ID_INDEX*2, sizeof(struct bd_addr));

        // check if local irk should be copied
        if(l_irk_valid)
        {
            // Do a burst write in the exchange memory
            em_wr(param->local_irk.key, REG_BLE_EM_RAL_ADDR_GET(position) + BLE_RAL_LOCAL_IRK_INDEX*2, KEY_LEN);
        }

        // check if peer irk should be copied
        if(p_irk_valid)
        {
            // Do a burst write in the exchange memory
            em_wr(param->peer_irk.key, REG_BLE_EM_RAL_ADDR_GET(position) + BLE_RAL_PEER_IRK_INDEX*2, KEY_LEN);
        }

        //                          entryvalid, connected,
        ble_ral_info_pack(position, 1,          connected,
        //      in_wl, lrpavalid, lrparenew,   lirkvalid,   prpavalid, prparenew,   pirkvalid,   paddridtype
                in_wl, 0,         l_irk_valid, l_irk_valid, 0,         p_irk_valid, p_irk_valid, param->peer_id_addr_type);

        status = COMMON_ERROR_NO_ERROR;
    }

    return (status);
}

uint8_t llm_ral_dev_rm(struct hci_le_rmv_dev_from_rslv_list_cmd * param)
{
    uint8_t position;
    uint8_t status = COMMON_ERROR_NO_ERROR;

    if(param->peer_id_addr_type > ADDR_RAND)
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
    // find if address is in resolving address list
    else if(!llm_util_bd_addr_in_ral(&(param->peer_id_addr), param->peer_id_addr_type, &position))
    {
        status = COMMON_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else
    {
        // make device address invalid
        ble_entry_valid_setf(position, 0);
    }

    return (status);
}

uint8_t llm_ral_get_rpa(struct hci_le_rd_loc_rslv_addr_cmd *param, struct bd_addr * addr, bool local)
{
    uint8_t position;
    uint8_t status = COMMON_ERROR_UNKNOWN_CONNECTION_ID ;

    if(param->peer_id_addr_type > ADDR_RAND)
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
     // find if address is in resolving address list
    else if(llm_util_bd_addr_in_ral(&(param->peer_id_addr), param->peer_id_addr_type, &position))
    {
        uint16_t offset = 0;
        if(local)
        {
            offset = BLE_RAL_LOCAL_RPA_INDEX*2;
            if(ble_local_rpa_valid_getf(position))
            {
                status = COMMON_ERROR_NO_ERROR;
            }
        }
        else
        {
            offset = BLE_RAL_PEER_RPA_INDEX*2;
            if(ble_peer_rpa_valid_getf(position))
            {
                status = COMMON_ERROR_NO_ERROR;
            }
        }

        if((offset != 0) && (status == COMMON_ERROR_NO_ERROR))
        {
            // Do a burst write in the exchange memory
            em_rd(addr, REG_BLE_EM_RAL_ADDR_GET(position) + offset, sizeof(struct bd_addr));
        }
        else
        {
            memset(addr, 0, sizeof(struct bd_addr));
        }
    }

    return (status);
}

uint8_t llm_ral_set_timeout(struct hci_le_set_rslv_priv_addr_to_cmd *param)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;

    // check timeout range
    if((param->rpa_timeout == 0) || (param->rpa_timeout > LLM_RPA_TIMEOUT_MAX))
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // update RPA renew timeout
        llm_le_env.enh_priv_rpa_timeout = param->rpa_timeout;

        // initialize random seed
        ble_ral_local_rnd_pack(1, common_rand_word() & BLE_LRND_VAL_MASK);
        ble_ral_peer_rnd_pack(1, common_rand_word() & BLE_PRND_VAL_MASK);
    }

    return (status);
}


void llm_ral_update(void)
{
    if(GETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE))
    {
        // check if renewal timer should be restarted
        if(!GETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN))
        {
            // restart the timer that manages renewal of resolvable private addresses.
            kernel_timer_set(LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER, TASK_LLM, KERNEL_TIME_IN_SEC(llm_le_env.enh_priv_rpa_timeout));
            SETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN, true);
        }
    }
}


/// @} LLM
