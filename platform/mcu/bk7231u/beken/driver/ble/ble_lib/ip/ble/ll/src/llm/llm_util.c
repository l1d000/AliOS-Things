/**
 ****************************************************************************************
 *
 * @file llm_util.c
 *
 * @brief Link layer manager utilities declaration
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMUTIL
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
#include "llm.h"
#include "llm_util.h"

#include "common_bt.h"
#include "common_error.h"
#include "common_math.h"
#include "common_utils.h"
#include "llc_util.h"
#include "lld_util.h"
#include "reg_blecore.h"
#include "reg_ble_em_wpv.h"
#include "reg_ble_em_wpb.h"
#include "reg_ble_em_ral.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Constant nibble to use as top 4 MSBs, to have at least 2 transitions
const uint8_t LLM_AA_CT1[3]  = {0x02, 0x04, 0x06};

///Constant nibble to use in AA to get more than 1 bit different from Advertising AA
const uint8_t LLM_AA_CT2[2]  = {0x0C};

uint8_t max_scan_numbers;

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_PERIPHERAL)
struct llm_util_cnx_bd_addr_tag
{
    /// List element for chaining in the scheduling lists
    struct common_list_hdr  hdr;
    ///Address of device to be added to White List
    struct bd_addr      dev_addr;
    /// Connection handle
    uint16_t            conhdl;
    /// Type of address of the device to be added to the White List - 0=public/1=random
    uint8_t             dev_addr_type;
    /// True if the device address is in white list
    bool                in_wl;
};
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


uint16_t llm_util_bd_addr_wl_position(struct bd_addr const *bd_address, uint8_t bd_addr_type)
{
    uint16_t position;
    struct bd_addr tmp_addr;

    if ((bd_addr_type & ADDR_MASK)  == ADDR_PUBLIC)
    {
        // public case
        for(position = 0; position < BLE_WHITELIST_MAX ; position++)
        {
            // Do a burst read in the exchange memory
            em_rd(&tmp_addr,
                    REG_BLE_EM_WPB_ADDR_GET(position),
                    sizeof(struct bd_addr));
            // Compare the 2 BD addresses
            if (common_bdaddr_compare(&tmp_addr, bd_address))
            {
                break;
            }
        }
    }
    else
    {
        // private case
        for(position = 0; position < BLE_WHITELIST_MAX ; position++)
        {
            // Do a burst read in the exchange memory
            em_rd(&tmp_addr,
                    REG_BLE_EM_WPV_ADDR_GET(position),
                    sizeof(struct bd_addr));
            // Compare the 2 BD addresses
            if (common_bdaddr_compare(&tmp_addr, bd_address))
            {
                break;
            }
        }
    }

    return position;
}

bool llm_util_bd_addr_in_wl(struct bd_addr const *bd_address, uint8_t bd_addr_type, bool* in_black_list)
{
    bool found = false;

    if(in_black_list != NULL)
    {
        *in_black_list = false;
    }

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // check if device present in black list
    if(llm_util_bl_check(bd_address, bd_addr_type, NULL, LLM_UTIL_BL_NO_ACTION_WL, &found) == COMMON_ERROR_ACL_CON_EXISTS)
    {
        if(in_black_list != NULL)
        {
            *in_black_list = true;
        }
    }
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    if(!found)
    {
        if(llm_util_bd_addr_wl_position(bd_address, bd_addr_type) < BLE_WHITELIST_MAX)
        {
            found = true;
        }
    }

    return (found);
}


/**
 ****************************************************************************************
 * @brief Checks the validity of the device address
 *
 ****************************************************************************************
 */
uint8_t llm_util_check_address_validity(struct bd_addr *bd_address, uint8_t addr_type)
{
    bool result;

    // Check if the BD address is not NULL
    result = common_bdaddr_compare(bd_address, &common_null_bdaddr);
    if(result == true)
    {
        return(COMMON_ERROR_INVALID_HCI_PARAM);
    }
    return(COMMON_ERROR_NO_ERROR);
}
/**
 ****************************************************************************************
 * @brief verify if the channel map contains at least 1 channel
 *
 *
 ****************************************************************************************
 */
uint8_t llm_util_check_map_validity(uint8_t *channel_map, uint8_t nb_octet)
{
    uint8_t i ;
    uint8_t j ;
    uint8_t sum =0;

    /// checks that at no more than the advertising channel are allowed
    if((*channel_map & 0xF8) && (nb_octet == 1))
    {
        sum = 0;
    }
    else
    {
        /// checks that at least one channel is available
        for (j = 0; j < nb_octet; j++)
        {
            // check if unavailable channel(s) is(are) sets
            if((j == 4) && (*channel_map & 0xE0))
            {
                sum = LE_NB_CH_MAP_MAX + 1;
                break;
            }
            for (i = 0; i < 8; i++)
            {
                sum +=(*channel_map>>i) & 0x1;
            }
            channel_map++;
        }
    }
    /// return sum of all channels

    return(sum);
}
/**
 ****************************************************************************************
 * @brief verify if current address type is the requested
 *
 *
 ****************************************************************************************
 */
void llm_util_apply_bd_addr(uint8_t addr_type)
{
    switch(addr_type)
    {
    case ADDR_RPA_OR_RAND:
    case ADDR_RAND:
        // Set the private address in the dedicated register
        lld_util_set_bd_address(&llm_le_env.rand_add, ADDR_RAND);
        break;
    default:
        // Set the public address in the dedicated register
        lld_util_set_bd_address(&llm_le_env.public_add, ADDR_PUBLIC);
        break;
    }
}

/**
 ****************************************************************************************
 * @brief Set public address
 *
 *
 ****************************************************************************************
 */
void llm_util_set_public_addr(struct bd_addr *bd_addr)
{
    memcpy(&llm_le_env.public_add.addr[0], &bd_addr->addr[0], BD_ADDR_LEN);
}

/**
 ****************************************************************************************
 * @brief Check the event mask
 *
 *
 ****************************************************************************************
 */
bool llm_util_check_evt_mask(uint8_t event_id)
{
    return ((llm_le_env.eventmask.mask[event_id/8] & (1<<(event_id - ((event_id/8)*8)))) != 0);
}

void llm_util_get_channel_map(struct le_chnl_map *map)
{
#if (BLE_CENTRAL)
    memcpy(&map->map[0], &llm_le_env.ch_map_assess.ch_map.map[0], LE_CHNL_MAP_LEN);
#endif // (BLE_CENTRAL)
}


void llm_util_get_supp_features(struct le_features *feats)
{
    memcpy(&feats->feats[0], &llm_local_le_feats.feats[0], LE_FEATS_LEN);
}

#if (BLE_BROADCASTER || BLE_PERIPHERAL)
void llm_util_adv_data_update(void)
{
    // Check if new advertising data pending
    if (llm_le_env.advertising_params->adv_data_req)
    {
        // Get the SET_ADV_DATA message
        llm_set_adv_data((struct hci_le_set_adv_data_cmd const *)kernel_msg2param(llm_le_env.advertising_params->adv_data_req));

        // Free the stored message
        kernel_msg_free(llm_le_env.advertising_params->adv_data_req);

        llm_le_env.advertising_params->adv_data_req = (struct kernel_msg*)NULL;
    }

    // Check if new scan response data pending
    if (llm_le_env.advertising_params->scan_rsp_req)
    {
        // Get the SET_ADV_DATA message
        llm_set_scan_rsp_data((struct hci_le_set_scan_rsp_data_cmd const *)kernel_msg2param(llm_le_env.advertising_params->scan_rsp_req));

        // Free the stored message
        kernel_msg_free(llm_le_env.advertising_params->scan_rsp_req);

        llm_le_env.advertising_params->scan_rsp_req = (struct kernel_msg*)NULL;
    }
}
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t llm_util_bl_check(const struct bd_addr *bd_addr_to_add, uint8_t bd_addr_type, uint16_t *conhdl, uint8_t wl_flag_action, bool* in_wl)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    // Pick the first element in the connected list
    struct llm_util_cnx_bd_addr_tag *scan = (struct llm_util_cnx_bd_addr_tag *)common_list_pick(&llm_le_env.cnx_list);
    // scan the list until the end
    while (scan)
    {
        //Clear all flag
        if((bd_addr_to_add == NULL)&&(wl_flag_action == LLM_UTIL_BL_CLEAR_WL))
        {
            scan->in_wl = false;
        }
        // Check if the BD address is not one contains in the list
        else if(common_bdaddr_compare(bd_addr_to_add, &scan->dev_addr) && (scan->dev_addr_type == (bd_addr_type&ADDR_MASK)))
        {
            status = COMMON_ERROR_ACL_CON_EXISTS;
            if(conhdl != NULL)
            {
                *conhdl = scan->conhdl;
            }

            if(wl_flag_action == LLM_UTIL_BL_CLEAR_WL)
            {
                scan->in_wl = false;
            }
            else if (wl_flag_action == LLM_UTIL_BL_SET_WL)
            {
                scan->in_wl = true;
            }


            if(in_wl != NULL)
            {
                *in_wl = scan->in_wl;
            }
            break;
        }
        scan = (struct llm_util_cnx_bd_addr_tag *)scan->hdr.next;
    }
    return(status);
}
uint8_t llm_util_bl_add(struct bd_addr *bd_addr_to_add, uint8_t bd_addr_type, uint16_t conhdl)
{
    // allocate an element for the list
    struct llm_util_cnx_bd_addr_tag *elt = (struct llm_util_cnx_bd_addr_tag *)kernel_malloc(sizeof(struct llm_util_cnx_bd_addr_tag), KERNEL_MEM_ENV);
    uint8_t status = COMMON_ERROR_NO_ERROR;
    uint8_t position;

    // check resolving list
    if(llm_util_bd_addr_in_ral(bd_addr_to_add, (bd_addr_type&ADDR_MASK), &position))
    {
        ble_connected_setf(position, 1);
    }

    if(elt)
    {
        // save all the value in the created element and push it in the list
        memcpy(&elt->dev_addr, bd_addr_to_add, BD_ADDR_LEN);

        elt->conhdl = conhdl;
        elt->dev_addr_type = (bd_addr_type&ADDR_MASK);
        // check if device is in white list
        elt->in_wl = (llm_util_bd_addr_wl_position(bd_addr_to_add, bd_addr_type) < BLE_WHITELIST_MAX) ? true :  false;

        // Add the element in the connected list
        common_list_push_back(&llm_le_env.cnx_list, &elt->hdr);

        if(elt->in_wl)
        {
            llm_wl_dev_rem(&elt->dev_addr,  (bd_addr_type&ADDR_MASK));
        }
    }
    else
    {
        status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
    }

    return(status);
}

uint8_t llm_util_bl_rem(uint16_t conhdl)
{
    uint8_t status = COMMON_ERROR_UNKNOWN_CONNECTION_ID;
    // Pick the first element in the connected list
    struct llm_util_cnx_bd_addr_tag *scan = (struct llm_util_cnx_bd_addr_tag *)common_list_pick(&llm_le_env.cnx_list);
    // scan the list until the end
    while (scan)
    {
        // Check if the BD address is not one contains in the list
       if(scan->conhdl == conhdl)
       {
           status = COMMON_ERROR_NO_ERROR;
           break;
       }
       scan = (struct llm_util_cnx_bd_addr_tag *)scan->hdr.next;
    }
    // If not device in the list or the bd address doesn't match
    if(status == COMMON_ERROR_NO_ERROR)
    {
        uint8_t position;

        // check resolving list
        if(llm_util_bd_addr_in_ral(&(scan->dev_addr), scan->dev_addr_type, &position))
        {
            ble_connected_setf(position, false);
        }

        // extract element from the list
        common_list_extract(&llm_le_env.cnx_list, &scan->hdr, 0);
        
        if(scan->in_wl)
        {
            llm_wl_dev_add(&scan->dev_addr,  scan->dev_addr_type);
        }


        // Free the memory
        kernel_free(scan);
    }


    return(status);
}

#if(BLE_2MBPS)
void llm_util_get_default_phy(uint8_t *tx_phy, uint8_t *rx_phy)
{
    // Init phys
    *tx_phy = PHYS_1MBPS_PREF | PHYS_2MBPS_PREF;
    *rx_phy = PHYS_1MBPS_PREF | PHYS_2MBPS_PREF;
    //If TX phy preference should be used
    if(!(llm_le_env.phys_val.rate_preference & ALL_PHYS_TX_NO_PREF))
    {
        *tx_phy = llm_le_env.phys_val.conn_initial_rate_tx;
    }
    if(!(llm_le_env.phys_val.rate_preference & ALL_PHYS_RX_NO_PREF))
    {
        *rx_phy = llm_le_env.phys_val.conn_initial_rate_rx;
    }
}

#endif
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)


bool llm_util_bd_addr_in_ral(struct bd_addr const *bd_address, uint8_t bd_addr_type, uint8_t *position)
{
    struct bd_addr tmp_addr;
    uint16_t cursor;
    *position = BLE_RESOL_ADDR_LIST_MAX;

    for(cursor = 0 ; cursor < BLE_RESOL_ADDR_LIST_MAX ; cursor++)
    {
        if(ble_entry_valid_getf(cursor))
        {
            // Do a burst read in the exchange memory
            em_rd(&tmp_addr,
                    REG_BLE_EM_RAL_ADDR_GET(cursor) + BLE_RAL_PEER_ID_INDEX*2,
                    sizeof(struct bd_addr));

            // Compare the 2 BD addresses
            if (((bd_addr_type&ADDR_RAND) == ble_peer_id_type_getf(cursor)) && (common_bdaddr_compare(&tmp_addr, bd_address)))
            {
                *position = cursor;
                break;
            }
        }
        // keep value of first free element
        else if ((*position == BLE_RESOL_ADDR_LIST_MAX))
        {
            *position = cursor;
        }
    }

    return (cursor != BLE_RESOL_ADDR_LIST_MAX);
}



#if (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)
void llm_end_evt_defer(void)
{
    /*
     ****************************************************************************************
       DIRECTED ADVERTISING TO MGT
     ****************************************************************************************
     */
    #if (BLE_PERIPHERAL)
    kernel_state_t current_state = kernel_state_get(TASK_LLM);
    // In case where the directed time out occurred
    if ((current_state == LLM_ADVERTISING) &&
        (llm_le_env.advertising_params->type == LL_ADV_CONN_DIR) &&
        (!llm_le_env.advertising_params->adv_ldc_flag))
    {
        struct llc_create_con_req_ind msg_param;
        memcpy(msg_param.peer_addr.addr, llm_le_env.advertising_params->peer_addr.addr,BD_ADDR_LEN);
        msg_param.peer_addr_type = llm_le_env.advertising_params->peer_addr_type;

        // The Link Layer notifies the host of the loss of connection.
        llc_le_con_cmp_evt_send(COMMON_ERROR_DIRECT_ADV_TO, 0, &msg_param);

        // Delete the event
        lld_evt_delete_elt_push(llm_le_env.elt, true, false);

        // set the state to advertising
        kernel_state_set(TASK_LLM, LLM_IDLE);
    }
    #endif //(BLE_PERIPHERAL)

    /*
     *********************************************************************************
           TEST MODE ENDING MGT
     *********************************************************************************
     */

    // check if test mode is finished and send the event to the host
    llm_util_chk_tst_mode();

    // update enhance privacy state to update random number used for address generation
    llm_ral_update();
}


bool llm_pdu_defer(uint16_t status, uint8_t rx_hdl, uint8_t tx_cnt)
{
    kernel_state_t current_state = kernel_state_get(TASK_LLM);
    bool elt_deleted = false;
    /*
    *********************************************************************************
       INITIATING TX MGT
    *********************************************************************************
    */
    #if (BLE_CENTRAL)
    // Check if we are in initiating state
    if ((current_state == LLM_INITIATING) && !(status & (BLE_TYPE_ERR_BIT | BLE_SYNC_ERR_BIT)))
    {
        // Check if connect request has been transmitted
      //  if (tx_cnt != 0)
        {
            // Initiating state, so it is a connection request confirmation
            llm_con_req_tx_cfm(rx_hdl);
            elt_deleted = true;
        }
    }
    else
    #endif // BLE_CENTRAL

    /*
    *********************************************************************************
       ADVERTISING AND INITIATING RX MGT
    *********************************************************************************
    */
    if ((current_state != LLM_STOPPING) && (current_state != LLM_IDLE) && (current_state != LLM_TEST))
    {
	
        // In test mode the SW should filter the error by itself
        if(!(status & (BLE_MIC_ERR_BIT | BLE_CRC_ERR_BIT | BLE_LEN_ERR_BIT | BLE_TYPE_ERR_BIT | BLE_SYNC_ERR_BIT)))
        {
            // Get the type of the advertising packet received
            uint8_t pdu_type = ble_rxtype_getf(rx_hdl);
            switch (pdu_type)
            {
                #if (BLE_CENTRAL || BLE_OBSERVER)
                case LL_ADV_CONN_UNDIR:
                case LL_ADV_CONN_DIR:
                case LL_ADV_DISC_UNDIR:
                case LL_ADV_NONCONN_UNDIR:
                case LL_SCAN_RSP:
                {
                    if(current_state == LLM_SCANNING)
                    {
                        llm_le_adv_report_ind(rx_hdl);
                    }
                }break;
                #endif // BLE_CENTRAL || BLE_OBSERVER
                #if (BLE_PERIPHERAL)
                case LL_CONNECT_REQ:
                {
                    if(current_state == LLM_ADVERTISING)
                    {
                        llm_con_req_ind(rx_hdl, status);
                    }
                }break;
                #endif // BLE_PERIPHERAL
                case LL_SCAN_REQ:
								//	UART_PRINTF("LL_SCAN_REQ \r\n");
                    break;
                default:
                    ASSERT_INFO(0, current_state, pdu_type);
                    break;
            }
        }
    }

    // Free the received buffer
    em_buf_rx_free(rx_hdl);
    return (elt_deleted);
}
#endif // #if (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)


/// @} LLM
