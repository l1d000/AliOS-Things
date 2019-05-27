/**
 ****************************************************************************************
 *
 * @file llm_task.c
 *
 * @brief LLM task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMTASK
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
#include "common_bt.h"
#include "common_endian.h"
#include "common_error.h"
#include "common_list.h"
#include "common_math.h"
#include "common_utils.h"
#include "common_version.h"
#include "RomCallFlash.h"
#include "kernel_event.h"
#include "kernel_msg.h"
#include "kernel_timer.h"

#include "rwip_config.h"

#include "em_buf.h"
#include "llm.h"
#include "llm_task.h"
#include "llm_util.h"
#include "llcontrl.h"
#include "llc_task.h"
#include "llc_util.h"
#include "lld.h"
#include "lld_util.h"
#include "lld_pdu.h"
#include "lld_evt.h"
#include "rwip.h"
#include "dbg.h"
#include "reg_ble_em_wpb.h"
#include "reg_ble_em_wpv.h"

#if (HCI_PRESENT)
#include "hci.h"
#endif //(HCI_PRESENT)

#if (SECURE_CONNECTIONS)
#include "ecc_p256.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)
#endif // (SECURE_CONNECTIONS)

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Handles the command HCI set event mask.
 * The handler processes the command parameters to configure the authorized event sent to
 * the host, and sends back the dedicated command complete event with the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_evt_mask_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_evt_mask_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    // Set the event mask in the environment variable
    memcpy(&llm_le_env.eventmask.mask[0], &param->le_mask.mask[0],EVT_MASK_LEN);

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, COMMON_ERROR_NO_ERROR);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI set random address.
 * The handler processes the command parameters to configure the random address in the
 * controller, and sends back the dedicated command complete event with the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_rand_add_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_rand_addr_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    uint8_t status;

    if(common_bdaddr_compare((struct bd_addr *)&param->rand_addr, &common_null_bdaddr) != true)
    {
        // save the private address
        memcpy(&llm_le_env.rand_add.addr[0],&param->rand_addr.addr[0],BD_ADDR_LEN);

        status = COMMON_ERROR_NO_ERROR;
    }
    else
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }
    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_BROADCASTER || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles the command HCI set advertising parameters.
 * The handler processes the command parameters to configure the controller by waiting
 * the start advertising command, and sends back the dedicated command complete event with
 * the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_adv_param_cmd_handler(kernel_msg_id_t const msgid,
                                            struct hci_le_set_adv_param_cmd const *param,
                                            kernel_task_id_t const dest_id,
                                            kernel_task_id_t const src_id)
{
    // status returned in the command complete event
    uint8_t status;

    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_ADVERTISING:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
        default:
        {
            // Set the advertising parameters in the environment variable
            status = llm_set_adv_param(param);
        }break;
    }

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI read advertising channel transmit power.
 * The handler processes the command by checking the transmit power set for the
 * advertising and sends back the dedicated command complete event with the status and
 * the level of the power.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
hci_le_rd_adv_ch_tx_pw_cmd_handler(kernel_msg_id_t const msgid,
        void const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt *event;

    // allocate the complete event message
    event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE, hci_rd_adv_chnl_tx_pw_cmd_cmp_evt);

    // Put advertising TX power
    event->adv_tx_pw_lvl = rwip_rf.txpwr_dbm_get(LLM_ADV_CHANNEL_TXPWR, MOD_GFSK);

    // update the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set advertising data.
 * The handler processes the command parameters to configure the data that will be sent
 * if the host want to send data by advertising packet, and sends back the dedicated
 * command complete event with the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_adv_data_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_adv_data_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t status = COMMON_ERROR_NO_ERROR;

    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_ADVERTISING:
        {
            if (param->adv_data_len > ADV_DATA_LEN)
            {
                status = COMMON_ERROR_INVALID_HCI_PARAM;
            }
            else
            {
                ASSERT_ERR(llm_le_env.advertising_params);

                // ensure delete old data even if it has never been applied
                GLOBAL_INT_DIS();
                if(llm_le_env.advertising_params->adv_data_req)
                {
                    // Free the stored message
                    kernel_msg_free(llm_le_env.advertising_params->adv_data_req);
                    llm_le_env.advertising_params->adv_data_req = NULL;
                }
                GLOBAL_INT_RES();
                
                // Extract the kernel_msg pointer from the param passed
                llm_le_env.advertising_params->adv_data_req = kernel_param2msg(param);
                msg_status = KERNEL_MSG_NO_FREE;
            }
        }break;
        default:
        {
            // Set the advertising parameters in the environment variable
            status = llm_set_adv_data(param);
        }break;
    }
    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI set advertising enable.
 * The handler processes the parameters to enable or disable the advertising mode, to
 * change the state of the controller, and sends back the dedicated command complete
 * event with the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
hci_le_set_adv_en_cmd_handler(kernel_msg_id_t const msgid,
        struct hci_le_set_adv_en_cmd const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    // status returned in the command complete event
    uint8_t status;

    // not in advertising  mode only, the enable is accepted
    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_STOPPING:
        {
            return (KERNEL_MSG_SAVED);
        }
        case LLM_ADVERTISING:
        {
            if(param->adv_en == ADV_DIS)
            {
                status = COMMON_ERROR_NO_ERROR;
                break;
            }
        }
        // No break
        case LLM_SCANNING:
        case LLM_INITIATING:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
        default:
        {
            if(param->adv_en == ADV_EN)
            {
                status = COMMON_ERROR_NO_ERROR;
            }
            else
            {
                status = COMMON_ERROR_COMMAND_DISALLOWED;
            }
        }break;
    }
    if(status == COMMON_ERROR_NO_ERROR)
    {
        //Save the mode in case where the end will be postponed
        llm_le_env.state = kernel_state_get(TASK_LLM);
        llm_le_env.opcode = src_id;
        status = llm_set_adv_en(param);
    }
    //If the controller is busy postpone the command complete event when the procedure ended.
    if((status != COMMON_ERROR_NO_ERROR)||(param->adv_en == ADV_EN))
    {
        // Send the command complete event
        llm_common_cmd_complete_send(src_id, status);
    }

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI set scan response data.
 * The handler processes the command parameters to configure the data that will be sent
 * if a scan response is sent, and sends back the dedicated command complete event with
 * the status.
 *
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
hci_le_set_scan_rsp_data_cmd_handler(kernel_msg_id_t const msgid,
        struct hci_le_set_scan_rsp_data_cmd const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{

    int msg_status = KERNEL_MSG_CONSUMED;
    uint8_t status = COMMON_ERROR_NO_ERROR;

    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_SCANNING:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
        case LLM_ADVERTISING:
        {
            if (param->scan_rsp_data_len > SCAN_RSP_DATA_LEN)
            {
                status = COMMON_ERROR_INVALID_HCI_PARAM;
            }
            else
            {
                ASSERT_ERR(llm_le_env.advertising_params);

                // ensure delete old data even if it has never been applied
                GLOBAL_INT_DIS();
                if(llm_le_env.advertising_params->scan_rsp_req)
                {
                    // Free the stored message
                    kernel_msg_free(llm_le_env.advertising_params->scan_rsp_req);
                    llm_le_env.advertising_params->scan_rsp_req = NULL;
                }
                GLOBAL_INT_RES();

                // Extract the kernel_msg pointer from the param passed
                llm_le_env.advertising_params->scan_rsp_req = kernel_param2msg(param);
                msg_status = KERNEL_MSG_NO_FREE;
            }
        }break;
        default:
        {
            // Set the advertising parameters in the environment variable
            status = llm_set_scan_rsp_data(param);
        }break;
    }
    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (msg_status);
}
#endif // BLE_BROADCASTER || BLE_PERIPHERAL

#if (BLE_OBSERVER || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles the command HCI set scan parameters.
 * The handler processes the command parameters to configure the controller by waiting
 * the start scanning command, and sends back the dedicated command complete event with
 * the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
hci_le_set_scan_param_cmd_handler(kernel_msg_id_t const msgid,
        struct hci_le_set_scan_param_cmd const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    // status returned in the command complete event
    uint8_t status;

    // not in scanning  mode only, the enable is accepted
    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_SCANNING:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
        default:
        {
            status = llm_set_scan_param(param);
        }break;
    }
    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI set scan enable.
 * The handler processes the parameters to enable or disable the scanning mode, to change
 * the state of the controller, and sends back the dedicated command complete event with
 * the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
hci_le_set_scan_en_cmd_handler(kernel_msg_id_t const msgid,
        struct hci_le_set_scan_en_cmd const *param,
        kernel_task_id_t const dest_id,
        kernel_task_id_t const src_id)
{
    // status returned in the command complete event
    uint8_t status;

    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_STOPPING:
        {
            return (KERNEL_MSG_SAVED);
        }
        case LLM_SCANNING:
        {
            if(param->scan_en == SCAN_DIS)
            {
                status = COMMON_ERROR_NO_ERROR;
                break;
            }
        }
        // No break
        case LLM_ADVERTISING:
        case LLM_INITIATING:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
        default:
        {
            if(param->scan_en == SCAN_EN)
            {
                status = COMMON_ERROR_NO_ERROR;
            }
            else
            {
                status = COMMON_ERROR_COMMAND_DISALLOWED;
            }
        }break;
    }
    if(status == COMMON_ERROR_NO_ERROR)
    {
        //Save the mode as the end will be deferred
        llm_le_env.state = kernel_state_get(TASK_LLM);
        //Save the opcode of the command sent in the complete event
        llm_le_env.opcode = src_id;
        status = llm_set_scan_en(param);
    }

    if((status != COMMON_ERROR_NO_ERROR)||(param->scan_en == SCAN_EN))
    {
        // Send the command complete event
        llm_common_cmd_complete_send(src_id, status);
        llm_le_env.state = LLM_IDLE;
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif // BLE_OBSERVER || BLE_CENTRAL


/**
 ****************************************************************************************
 * @brief Handles the HCI commands dedicated for White List management
 * - Clear White List
 *      The handler process the command by clearing all the devices from the WL in the
 *      controller, changing the size of the WL and sends back the dedicated command complete
 *      event with the status.
 * - Add device to White List
 *      The handler processes the command by adding the device from the WL in the controller,
 *      changing the size of the WL and sends back the dedicated command complete event with
 *      the status.
 * - Remove device from White List.
 *      The handler processes the command by removing the device from the WL in the controller,
 *      changing the size of the WL and sends back the dedicated command complete event with
 *      the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_wl_mngt_cmd_handler(kernel_msg_id_t const msgid,
                                      void const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    uint16_t conhdl;
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
    // Status returned in the command complete event
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    do
    {
        // Get the current state of the LLM task
        kernel_state_t current_state =  kernel_state_get(TASK_LLM);

        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        if (current_state == LLM_ADVERTISING)
        {
            if (llm_le_env.advertising_params->filterpolicy != ADV_ALLOW_SCAN_ANY_CON_ANY)
            {
                break;
            }
        }
        #endif //(BLE_PERIPHERAL || BLE_BROADCASTER)

        #if (BLE_CENTRAL || BLE_OBSERVER)
        if ((current_state == LLM_SCANNING) || (current_state== LLM_INITIATING))
        {
            if (llm_le_env.scanning_params->filterpolicy != SCAN_ALLOW_ADV_ALL)
            {
                break;
            }
        }
        #endif //(BLE_CENTRAL || BLE_OBSERVER)

        // Status is OK
        status = COMMON_ERROR_NO_ERROR;

        switch (msgid)
        {
            case HCI_LE_CLEAR_WLST_CMD_OPCODE:
            {
                // Clear the white list
                llm_wl_clr();
                #if (BLE_CENTRAL || BLE_PERIPHERAL)
                llm_util_bl_check((void*)NULL, 0, &conhdl, LLM_UTIL_BL_CLEAR_WL, NULL);
                #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
            } break;

            case HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE:
            {
                struct hci_le_add_dev_to_wlst_cmd *add_p = (struct hci_le_add_dev_to_wlst_cmd *)param;

                // Add the bd address of the device in the white list
                status = llm_wl_dev_add_hdl(&add_p->dev_addr, add_p->dev_addr_type);

            } break;

            case HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE:
            {
                struct hci_le_add_dev_to_wlst_cmd *del_p = (struct hci_le_add_dev_to_wlst_cmd *)param;
                // Remove the device bd address in the white list
                status = llm_wl_dev_rem_hdl(&del_p->dev_addr, del_p->dev_addr_type);
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }
    } while(0);

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles the command HCI create connection.
 * The handler processes the command by starting the initiation of the connection, setting
 * all the necessary values to maintain the connection and set the state in the controller
 * no event is generated.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_create_con_cmd_handler(kernel_msg_id_t const msgid,
                                         struct hci_le_create_con_cmd const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    // status returned in the command complete event
    uint8_t status = (uint8_t) COMMON_ERROR_NO_ERROR;

    //get the current state of the llm task
    switch(kernel_state_get(TASK_LLM))
    {
        case LLM_STOPPING:
        {
            return (KERNEL_MSG_SAVED);
        }
        case LLM_IDLE:
        {
            // initiate the connection
            status = llm_create_con(param);
            if(status == COMMON_ERROR_NO_ERROR)
            {
                llm_le_env.opcode = src_id;
                llm_le_env.state = kernel_state_get(TASK_LLM);
            }
        }break;
        default:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        }break;
    }

    // Send the command complete event
    llm_common_cmd_status_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI  cancel create connection.
 * The handler processes the command by stopping the initiation of the connection, setting
 * the state in the controller and sends back the dedicated command complete event with
 * the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_create_con_cancel_cmd_handler(kernel_msg_id_t const msgid,
                                                void const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    // Status returned in the command complete event
    uint8_t status;

    switch (kernel_state_get(TASK_LLM))
    {
        case LLM_INITIATING:
        {
            // Stop Initiating
            lld_scan_stop(llm_le_env.elt);

            //Save the mode in case where the end will be postponed
            llm_le_env.state = LLM_INITIATING;
            llm_le_env.opcode = src_id;
            // Go to stopping state
            kernel_state_set(TASK_LLM, LLM_STOPPING);

            // Sets the status
            status = COMMON_ERROR_NO_ERROR;
        } break;
        // In case where a connection is requested on a existing link teh status is not changed
        default:
        {
            status = COMMON_ERROR_COMMAND_DISALLOWED;
        } break;
    }
    //If the controller is busy postpone the command complete event when the procedure ended.
    if(status == COMMON_ERROR_COMMAND_DISALLOWED)
    {
        // Send the command complete event
        llm_common_cmd_complete_send(src_id, status);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set host channel classification.
 * The handler processes the command by using the parameters to specify channel
 * classification for data channels based on host "local information", and sends back
 * the dedicated command complete event with the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_host_ch_class_cmd_handler(kernel_msg_id_t const msgid,
                                                struct hci_le_set_host_ch_class_cmd const *param,
                                                kernel_task_id_t const dest_id,
                                                kernel_task_id_t const src_id)
{
    // Command status
    uint8_t status;
    #if (BLE_CENTRAL && BLE_CHNL_ASSESS)
    if (llm_le_env.ch_map_assess.llm_le_set_host_ch_class_cmd_sto == true)
    {
        // Compute number of good channels in the provided Channel Map value
        uint8_t nd_good_channel =
                llm_util_check_map_validity((uint8_t*) &param->chmap.map[0], LE_CHNL_MAP_LEN);

        /*
         * The interval between two successive commands sent shall be at least one second.
         * 1 sec --> 100*10ms
         */
        kernel_timer_set(LLM_LE_SET_HOST_CH_CLASS_CMD_STO, TASK_LLM, 100);

        // Reset timeout status flag
        llm_le_env.ch_map_assess.llm_le_set_host_ch_class_cmd_sto = false;

        // at least 2 channel must be enabled
        if ((nd_good_channel >= 2) && (nd_good_channel <= LE_NB_CH_MAP_MAX))
        {
            // Copy the new channel map for the next connection
            llm_le_env.ch_map_assess.ch_map = param->chmap;

            status = COMMON_ERROR_NO_ERROR;
        }
        else
        {
            status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
    }
    else
    #endif //(BLE_CENTRAL && BLE_CHNL_ASSESS)
    {
        status = COMMON_ERROR_COMMAND_DISALLOWED;
    }

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Handles the command HCI read local supported features.
 * The handler processes the command by checking the local supported features set in the
 * controller and sends back the dedicated command complete event with the status and the
 * features.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_local_supp_feats_cmd_handler(kernel_msg_id_t const msgid,
                                                  void const *param,
                                                  kernel_task_id_t const dest_id,
                                                  kernel_task_id_t const src_id)
{
    // allocate the complete event message
    struct hci_le_rd_local_supp_feats_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE, hci_le_rd_local_supp_feats_cmd_cmp_evt);

    // get the local features
    memcpy(&event->feats.feats[0],&llm_local_le_feats.feats[0],LE_FEATS_LEN);

    // update the status

    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI read white list size.
 * The handler processes the command by reading the total number of white list entries that
 * can be stored in the controller and sends back the dedicated command complete event with
 * the status and the size of WL.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_wl_size_cmd_handler(kernel_msg_id_t const msgid,
                                         void const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_rd_wlst_size_cmd_cmp_evt *event;

    // allocate the complete event message
    event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_WLST_SIZE_CMD_OPCODE, hci_rd_wlst_size_cmd_cmp_evt);
    // update the status
    event->status = COMMON_ERROR_NO_ERROR;
    // update the status
    event->wlst_size = BLE_WHITELIST_MAX;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);

}
/**
 ****************************************************************************************
 * @brief Handles the command HCI read supported states.
 * The handler processes the command by reading the useful variable to sends back the
 * dedicated command complete event with the status and the value of the supported states.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_supp_states_cmd_handler(kernel_msg_id_t const msgid,
                                             void const *param,
                                             kernel_task_id_t const dest_id,
                                             kernel_task_id_t const src_id)
{

    // allocate the complete event message
    struct hci_rd_supp_states_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_SUPP_STATES_CMD_OPCODE, hci_rd_supp_states_cmd_cmp_evt);

    memcpy(&event->states.supp_states[0],&llm_local_le_states.supp_states[0],LE_STATES_LEN);

    // update the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI read buffer size.
 * The handler processes the command by reading the number of data buffer available in the
 * controller and sends back the dedicated command complete event with the status and the
 * value of the number of free buffer.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL || BLE_CENTRAL)
static int hci_le_rd_buff_size_cmd_handler(kernel_msg_id_t const msgid,
                                           void const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{

    // structure type for the complete command event
    struct hci_le_rd_buff_size_cmd_cmp_evt *event= KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_BUFF_SIZE_CMD_OPCODE, hci_le_rd_buff_size_cmd_cmp_evt);

    // Fill in message parameters
    event->hc_tot_nb_data_pkts =  BLE_TX_BUFFER_CNT;
    event->hc_data_pk_len = llm_le_env.data_len_val.suppted_max_tx_octets;

    // update the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles the command HCI encrypt data.
 * The handler processes the command by encrypt the plain text data in the command
 * using the key given in the command and returns the encrypted data to the Host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_enc_cmd_handler(kernel_msg_id_t const msgid,
                                  struct hci_le_enc_cmd const *param,
                                  kernel_task_id_t const dest_id,
                                  kernel_task_id_t const src_id)
{
    //extract the kernel_msg pointer from the param passed
    struct kernel_msg * msg = kernel_param2msg(param);

    // Modify the HCI command to an internal encryption request from HCI
    msg->id = LLM_ENC_REQ;
    msg->src_id = TASK_ID_HCI;

    // Push the request in the list of requests
    common_list_push_back(&llm_le_env.enc_req, &msg->hdr);

    // Check if an encryption is pending
    if (!llm_le_env.enc_pend)
    {
        // No encryption pending, so we can proceed to the encryption
        llm_encryption_start((struct llm_enc_req *) param);
    }

    // Message is not freed immediately. It will be freed when encryption is completed
    return (KERNEL_MSG_NO_FREE);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI rand.
 * The handler processes the command by calling the random generator and sends back the
 * dedicated command complete event with the status and the value of random number.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rand_cmd_handler(kernel_msg_id_t const msgid,
                                   void const *param,
                                   kernel_task_id_t const dest_id,
                                   kernel_task_id_t const src_id)
{
    // allocate the complete event message
    struct hci_le_rand_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RAND_CMD_OPCODE, hci_le_rand_cmd_cmp_evt);

    // gets random number
#if 0 // DEBUG ONLY
    {

        uint32_t dbg_rand_word = 0x12345678;

        common_write32p(&event->nb.nb[0], dbg_rand_word);
        common_write32p(&event->nb.nb[4], dbg_rand_word);
    }
#else
    common_write32p(&event->nb.nb[0], common_rand_word());
    common_write32p(&event->nb.nb[4], common_rand_word());
#endif
    // update the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

#if !BT_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Handles the reset request from the host
 * Reset the link layer manager, controller and driver
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_reset_cmd_handler(kernel_msg_id_t const msgid,
                                 void const *param,
                                 kernel_task_id_t const dest_id,
                                 kernel_task_id_t const src_id)
{
    // Reset BLE
    rwip_reset();
	//	rom_env.rwip_reset();
    // Send the command complete event
    llm_common_cmd_complete_send(src_id, COMMON_ERROR_NO_ERROR);


    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the read bd address request from the host
 * Send to the host the bd address
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_bd_addr_cmd_handler(kernel_msg_id_t const msgid,
                                      void const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    // allocate the Command Complete event message
    struct hci_rd_bd_addr_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_BD_ADDR_CMD_OPCODE, hci_rd_bd_addr_cmd_cmp_evt);

    // gets the device address from the registers
    memcpy(&event->local_addr.addr[0],&llm_le_env.public_add.addr[0],BD_ADDR_LEN);
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the read local version informations command request from the host
 * Reads the values for the version information for the local Controller.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_local_ver_info_cmd_handler(kernel_msg_id_t const msgid,
                                             void const *param,
                                             kernel_task_id_t const dest_id,
                                             kernel_task_id_t const src_id)
{
    // allocate the Command Complete event message
    struct hci_rd_local_ver_info_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_LOCAL_VER_INFO_CMD_OPCODE, hci_rd_local_ver_info_cmd_cmp_evt);

    // gets the hci version
    event->hci_ver = RWBT_SW_VERSION_MAJOR;
    // gets the hci revision
    event->hci_rev = COMMON_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);
    // gets the lmp version
    event->lmp_ver = RWBT_SW_VERSION_MAJOR;
    // gets the lmp subversion
    event->lmp_subver = COMMON_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR,
            RWBT_SW_VERSION_BUILD);
    // gets the manufacturer name
    event->manuf_name = RW_COMP_ID;
    // sets the status
    event->status = COMMON_ERROR_NO_ERROR;
    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the read local version informations command request from the host
 * Reads the values for the version information for the local Controller.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_local_supp_cmds_cmd_handler(kernel_msg_id_t const msgid,
                                             void const *param,
                                             kernel_task_id_t const dest_id,
                                             kernel_task_id_t const src_id)
{
    // allocate the Command Complete event message
    struct hci_rd_local_supp_cmds_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE, hci_rd_local_supp_cmds_cmd_cmp_evt);

    // gets the local supported commands
    memcpy(&event->local_cmds.cmds[0],&llm_local_cmds.cmds[0],SUPP_CMDS_LEN);

    // sets the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the command HCI read local legacy supported features.
 * The handler processes the command by checking the local legacy supported features set
 * in the controller and sends back the dedicated command complete event with the status
 * and the features.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_local_supp_feats_cmd_handler(kernel_msg_id_t const msgid, void const *param,
                                               kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // allocate the complete event message
    struct hci_rd_local_supp_feats_cmd_cmp_evt *evt = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, src_id, hci_rd_local_supp_feats_cmd_cmp_evt);

    // get the local features
    memset(&evt->feats.feats[0], 0, FEATS_LEN);

    // update the status
    evt->status = COMMON_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set event mask.
 * The handler processes the command by setting the legacy event supported by the HCI.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_set_evt_mask_cmd_handler(kernel_msg_id_t const msgid,
                                        struct hci_set_evt_mask_cmd const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_DFT);

    // send the command complete event message
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set event mask page 2.
 * The handler processes the command by setting the legacy event supported by the HCI.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_set_evt_mask_page_2_cmd_handler(kernel_msg_id_t const msgid,
                                               struct hci_set_evt_mask_cmd const *param,
                                               kernel_task_id_t const dest_id,
                                               kernel_task_id_t const src_id)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_2);

    // send the command complete event message
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set controller to host flow control.
 * This command is used by the Host to turn flow control on or off for data a
 * sent in the direction from the Controller to the Host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_set_ctrl_to_host_flow_ctrl_cmd_handler(kernel_msg_id_t const msgid,
                                                      struct hci_set_ctrl_to_host_flow_ctrl_cmd const *param,
                                                      kernel_task_id_t const dest_id,
                                                      kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_INVALID_HCI_PARAM;
    bool acl_flow_cntl_en  = false;

    if (param->flow_cntl <= FLOW_CONTROL_ACL_SCO)
    {
        switch(param->flow_cntl)
        {
            case FLOW_CONTROL_OFF:
            case FLOW_CONTROL_SCO:
            {
                // disabled by default
            }break;
            case FLOW_CONTROL_ACL:
            case FLOW_CONTROL_ACL_SCO:
            {
                acl_flow_cntl_en = true;
            }break;
            default:
                break;
        }

        // Enable HCI Host flow control for ACL data
        status = hci_fc_acl_en(acl_flow_cntl_en);
    }

    // send the command complete event message
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI host buffer size.
 * The command is used by the host to notify the controller about the maximum size of the
 * data portion of HCI data packets sent from the controller to the host.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_host_buf_size_cmd_handler(kernel_msg_id_t const msgid,
                                         struct hci_host_buf_size_cmd const *param,
                                         kernel_task_id_t const dest_id,
                                         kernel_task_id_t const src_id)
{
    uint8_t status = hci_fc_acl_buf_size_set(param->acl_pkt_len, param->nb_acl_pkts);

    // sends the command complete event message
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI host number of completed packets.
 * The command is used by the host indicate to the controller the number of HCI data
 * packets that have been completed for each connection handle.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_host_nb_cmp_pkts_cmd_handler(kernel_msg_id_t const msgid,
                                            struct hci_host_nb_cmp_pkts_cmd const *param,
                                            kernel_task_id_t const dest_id,
                                            kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    uint8_t idx;
    uint16_t acl_pkt_cnt = 0;


    for (idx=0; idx < param->nb_of_hdl; idx++)
    {
        if (param->con_hdl[idx] < BLE_CONNECTION_MAX)
        { // BLE ACL link
            acl_pkt_cnt += param->nb_comp_pkt[idx];
        }
        else
        { // not ACL handle reported
            ASSERT_ERR(0);
            status = COMMON_ERROR_INVALID_HCI_PARAM;
        }
    }

    // update the Flow Control module with counted packets
    hci_fc_host_nb_acl_pkts_complete(acl_pkt_cnt);

    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
    // A command complete event must be sent to host only if there is an error
    if (status != COMMON_ERROR_NO_ERROR)
    {
        // Send the command complete event
        llm_common_cmd_complete_send(src_id, status);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI read buffer size.
 * The handler processes the command by reading the number of data buffer available in the
 * controller and sends back the dedicated command complete event with the status and the
 * value of the number of free buffer.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_buff_size_cmd_handler(kernel_msg_id_t const msgid,
                                        void const *param,
                                        kernel_task_id_t const dest_id,
                                        kernel_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_rd_buff_size_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_BUFF_SIZE_CMD_OPCODE, hci_rd_buff_size_cmd_cmp_evt);

    // in single mode for the legacy command zero is returned
    event->hc_tot_nb_data_pkts = 0;
    event->hc_data_pk_len = 0;
    event->hc_sync_pk_len= 0;
    event->hc_tot_nb_sync_pkts =0;
    // update the status
    event->status = COMMON_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}
#endif // !BT_EMB_PRESENT

#if !(BLE_QUALIF)
/**
 ***************************************************************************************
 * BLE 4.2 handlers
 ***************************************************************************************
*/
/**
 ****************************************************************************************
 * @brief Handles the LE Read Suggested Default Data Length Command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL || BLE_CENTRAL)
static int hci_le_rd_suggted_dft_data_len_cmd_handler(kernel_msg_id_t const msgid,
                                                      void const *param,
                                                      kernel_task_id_t const dest_id,
                                                      kernel_task_id_t const src_id)
{
    // allocate the Command Complete event message
    struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, hci_le_rd_suggted_dft_data_len_cmd_cmp_evt);
    // gets the max size for TX
    event->suggted_max_tx_octets = llm_le_env.data_len_val.conn_initial_max_tx_octets;
    // gets the max time for TX
    event->suggted_max_tx_time = llm_le_env.data_len_val.conn_initial_max_tx_time;
    // sets the status
    event->status = COMMON_ERROR_NO_ERROR;
    // send the message
    hci_send_2_host(event);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LE Write Suggested Default Data Length Command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_wr_suggted_dft_data_len_cmd_handler(kernel_msg_id_t const msgid,
                                                      struct hci_le_wr_suggted_dft_data_len_cmd const *param,
                                                      kernel_task_id_t const dest_id,
                                                      kernel_task_id_t const src_id)
{
    // Set returned status to the HCI
    uint8_t status = COMMON_ERROR_NO_ERROR;
    #if (RW_DEBUG)
    if( (param->suggted_max_tx_octets < BLE_MIN_OCTETS ) || (param->suggted_max_tx_octets > llm_le_env.data_len_val.suppted_max_tx_octets)
    || (param->suggted_max_tx_time < BLE_MIN_TIME) || (param->suggted_max_tx_time > llm_le_env.data_len_val.suppted_max_tx_time))
    #else
    if( (param->suggted_max_tx_octets < BLE_MIN_OCTETS ) || (param->suggted_max_tx_octets > BLE_MAX_OCTETS)
                 || (param->suggted_max_tx_time < BLE_MIN_TIME) || (param->suggted_max_tx_time > BLE_MAX_TIME))
    #endif // (RW_DEBUG)
    {
        status = COMMON_ERROR_INVALID_HCI_PARAM;
    }

    else
    {
        llm_le_env.data_len_val.conn_initial_max_tx_octets = param->suggted_max_tx_octets;
        llm_le_env.data_len_val.conn_initial_max_tx_time = param->suggted_max_tx_time;
    }

    // send the command complete event message
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LE Read Suggested Default Data Length Command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_max_data_len_cmd_handler(kernel_msg_id_t const msgid,
                                              void const *param,
                                              kernel_task_id_t const dest_id,
                                              kernel_task_id_t const src_id)
{
    // allocate the Command Complete event message
    printf("hci_le_rd_max_data_len_cmd_handler start\r\n");
    struct hci_le_rd_max_data_len_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE, hci_le_rd_max_data_len_cmd_cmp_evt);

    // gets the max size supported for RX
    event->suppted_max_rx_octets = llm_le_env.data_len_val.suppted_max_rx_octets;
    // gets the max time supported for RX
    event->suppted_max_rx_time = llm_le_env.data_len_val.suppted_max_rx_time;
    // gets the max size supported for TX
    event->suppted_max_tx_octets = llm_le_env.data_len_val.suppted_max_tx_octets;
    // gets the max time supported for TX
    event->suppted_max_tx_time = llm_le_env.data_len_val.suppted_max_tx_time;
    // sets the status
    event->status = COMMON_ERROR_NO_ERROR;
    // send the message
    hci_send_2_host(event);
    printf("hci_le_rd_max_data_len_cmd_handler end \r\n");
    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Handles the LE Enhanced Privacy Management Commands handler
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_enh_privacy_mgmt_cmd_handler(kernel_msg_id_t const msgid,
                                               void const *param,
                                               kernel_task_id_t const dest_id,
                                               kernel_task_id_t const src_id)
{
    // Status returned in the command complete event
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    do
    {
        // Get the current state of the LLM task
        kernel_state_t current_state =  kernel_state_get(TASK_LLM);

        if(msgid != HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE)
        {
            #if (BLE_PERIPHERAL || BLE_BROADCASTER)
            if (current_state == LLM_ADVERTISING)
            {
                break;
            }
            #endif //(BLE_PERIPHERAL || BLE_BROADCASTER)

            #if (BLE_CENTRAL || BLE_OBSERVER)
            if ((current_state == LLM_SCANNING) || (current_state== LLM_INITIATING))
            {
                break;
            }
            #endif //(BLE_CENTRAL || BLE_OBSERVER)

            // wait end of Advertising procedure before executing the command
            if(current_state == LLM_STOPPING)
            {
                return (KERNEL_MSG_SAVED);
            }
        }

        // Status is OK
        status = COMMON_ERROR_NO_ERROR;

        switch (msgid)
        {
            case HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE:
            {
                struct hci_le_set_addr_resol_en_cmd* en_cmd = (struct hci_le_set_addr_resol_en_cmd*) param;

                // value range checks
                if(en_cmd->enable > 1)
                {
                    status = COMMON_ERROR_INVALID_HCI_PARAM;
                    break;
                }

                // update enhanced privacy feature flag
                SETF(llm_le_env.enh_priv_info, LLM_PRIV_ENABLE, en_cmd->enable);
                SETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN, en_cmd->enable);

                // manage renewal timer
                if(en_cmd->enable)
                {
                    // start the timer that manages renewal of resolvable private addresses.
                    kernel_timer_set(LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER, TASK_LLM, KERNEL_TIME_IN_SEC(llm_le_env.enh_priv_rpa_timeout));
                    // force renewal of resolvable private addresses
                    lld_util_ral_force_rpa_renew();
                }
                else
                {
                    // Stop the timer that manages renewal of resolvable private addresses.
                    kernel_timer_clear(LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER, TASK_LLM);
                }

            } break;

            case HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE:
            {
                // Clear the resolving address list
                llm_ral_clear();
            } break;

            case HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE:
            {
                // Add the bd address of the device in the Resolving Adress List
                status = llm_ral_dev_add((struct hci_le_add_dev_to_rslv_list_cmd *)param);
            } break;

            case HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE:
            {
                // Remove the bd address of the device from the Resolving Address List
                status = llm_ral_dev_rm((struct hci_le_rmv_dev_from_rslv_list_cmd *)param);
            } break;

            case HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE:
            {
                status = llm_ral_set_timeout((struct hci_le_set_rslv_priv_addr_to_cmd *)param);
            } break;

            default:
            {
                ASSERT_INFO(0, msgid, src_id);
            } break;
        }
    } while(0);

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the LE Enhanced Privacy Info Request Commands handler
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_enh_privacy_info_cmd_handler(kernel_msg_id_t const msgid,
                                               void const *param,
                                               kernel_task_id_t const dest_id,
                                               kernel_task_id_t const src_id)
{
    switch (msgid)
    {
        case HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE:
        {
            // structure type for the complete command event
            struct hci_le_rd_rslv_list_size_cmd_cmp_evt *event;

            // allocate the complete event message
            event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE, hci_le_rd_rslv_list_size_cmd_cmp_evt);
            // update the status
            event->status = COMMON_ERROR_NO_ERROR;
            // update the status
            event->size = BLE_RESOL_ADDR_LIST_MAX;

            // send the message
            hci_send_2_host(event);
        } break;

        case HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE:
        case HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE:
        {
            // structure type for the complete command event
            struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt *rd_cmp_evt;

            // allocate the complete event message
            rd_cmp_evt = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, msgid, hci_le_rd_peer_rslv_addr_cmd_cmp_evt);

            // Read Peer Resolvable Random address from the Resolving Address List
            rd_cmp_evt->status = llm_ral_get_rpa((struct hci_le_rd_loc_rslv_addr_cmd *)param,
                    &(rd_cmp_evt->peer_rslv_addr),
                    (msgid == HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE));
            // send the message
            hci_send_2_host(rd_cmp_evt);
        } break;

        default:
        {
            ASSERT_INFO(0, msgid, src_id);
        } break;
    }

    return (KERNEL_MSG_CONSUMED);
}



#if (BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles the HCI request to generate a new Public Key.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 
static int hci_le_rd_local_p256_public_key_cmd_handler(kernel_msg_id_t const msgid,
                                                       void const *param,
                                                       kernel_task_id_t const dest_id,
                                                       kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;

    // Check if ECDH is busy or not
    if(llm_le_env.cur_ecc_multiplication != LLM_ECC_IDLE)
    {
        status = COMMON_ERROR_COMMAND_DISALLOWED;
    }

    // Send the command complete event
    llm_common_cmd_status_send(src_id, status);

    // Need to get a 32 Byte random number - which we will use as the secret key.
    // We then ECC multiply this by the Base Points, to get a new Public Key.
    if(status == COMMON_ERROR_NO_ERROR)
    {
        bool forced_key;
        #if (NVDS_SUPPORT)
        uint8_t len = NVDS_LEN_LE_DBG_FIXED_P256_KEY_EN;
        #endif

        uint8_t* secret_key256 = &llm_le_env.secret_key256[0];

        // Check if private key is forced by host
        #if (NVDS_SUPPORT)
        if (nvds_get(NVDS_TAG_LE_DBG_FIXED_P256_KEY_EN, &len, (uint8_t *)&forced_key) != NVDS_OK)
        #endif //(NVDS_SUPPORT)
        {
            forced_key = false;
        }

        // Load the forced private key
        #if (NVDS_SUPPORT)
        len = NVDS_LEN_LE_PRIVATE_KEY_P256;
        if (forced_key && nvds_get(NVDS_TAG_LE_PRIVATE_KEY_P256, &len, secret_key256) != NVDS_OK)
        #endif //(NVDS_SUPPORT)
        {
            forced_key = false;
        }
        // Could not write new secret key to NVDS
        ecc_gen_new_secret_key(secret_key256, forced_key);

        llm_le_env.cur_ecc_multiplication = LLM_PUBLIC_KEY_GENERATION;

        ecc_gen_new_public_key(secret_key256, LLM_ECC_RESULT_IND, TASK_LLM);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the HCI request to calculate the DH-Key.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int hci_le_generate_dhkey_cmd_handler(kernel_msg_id_t const msgid,
                                             struct hci_le_generate_dh_key_cmd *param,
                                             kernel_task_id_t const dest_id,
                                             kernel_task_id_t const src_id)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;

    // Check if ECDH is busy or not
    if(llm_le_env.cur_ecc_multiplication != LLM_ECC_IDLE)
    {
        status = COMMON_ERROR_COMMAND_DISALLOWED;
    }

    // Send the command complete event
    llm_common_cmd_status_send(src_id, status);

    // check if ECDH algo can be started
    if(status == COMMON_ERROR_NO_ERROR)
    {
        llm_le_env.cur_ecc_multiplication = LLM_DHKEY_GENERATION;

        ecc_generate_key256(&llm_le_env.secret_key256[0], &param->public_key[0], &param->public_key[32], LLM_ECC_RESULT_IND, TASK_LLM);
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif // (SECURE_CONNECTIONS)

#if (BLE_2MBPS)
/**
 ****************************************************************************************
 * @brief Handles the command HCI set default PHY comamnd.
 * The handler processes the command parameters to configure the authorized PHY for a new
 * connection.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_dft_phy_cmd_handler(kernel_msg_id_t const msgid,
                                           struct hci_le_set_dft_phy_cmd const *param,
                                           kernel_task_id_t const dest_id,
                                           kernel_task_id_t const src_id)
{
    // Set the value in the global environment to be used for the next connection

    llm_le_env.phys_val.rate_preference         = param->all_phys;
    llm_le_env.phys_val.conn_initial_rate_tx    = param->tx_phys;
    llm_le_env.phys_val.conn_initial_rate_rx    = param->rx_phys;

    // Send the command complete event
    llm_common_cmd_complete_send(src_id, COMMON_ERROR_NO_ERROR);

    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_2MBPS)
#endif //#if (BLE_CENTRAL || BLE_PERIPHERAL)
#endif // !(BLE_QUALIF)

#if (BLE_TEST_MODE_SUPPORT)
/**
 ****************************************************************************************
 * @brief Handles the HCI commands dedicated for White List management
 * - Clear White List
 *      The handler process the command by clearing all the devices from the WL in the
 *      controller, changing the size of the WL and sends back the dedicated command complete
 *      event with the status.
 * - Add device to White List
 *      The handler processes the command by adding the device from the WL in the controller,
 *      changing the size of the WL and sends back the dedicated command complete event with
 *      the status.
 * - Remove device from White List.
 *      The handler processes the command by removing the device from the WL in the controller,
 *      changing the size of the WL and sends back the dedicated command complete event with
 *      the status.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_test_mode_mngt_cmd_handler(kernel_msg_id_t const msgid,
                                      void const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    // Status returned in the command complete event
    uint8_t status = COMMON_ERROR_COMMAND_DISALLOWED;

    do
    {
        // Get the current state of the LLM task
        kernel_state_t current_state =  kernel_state_get(TASK_LLM);
        if ((current_state != LLM_TEST) && (current_state != LLM_STOPPING) && (current_state != LLM_IDLE))
        {
            break;
        }

        if(current_state == LLM_STOPPING)
        {
            return (KERNEL_MSG_SAVED);
        }

        // Status is OK
        status = COMMON_ERROR_NO_ERROR;

        if(current_state == LLM_TEST)
        {
            // Stop test event
            lld_test_stop(llm_le_env.elt);
            // Go to STOPPING status
            kernel_state_set(TASK_LLM, LLM_STOPPING);
            //If the command is to stop the test mode
            if(msgid == HCI_LE_TEST_END_CMD_OPCODE)
            {
                //Set teh variable to prevent that is a test mode end and not a switch
                llm_le_env.test_mode.end_of_tst = true;

                return (KERNEL_MSG_CONSUMED);
            }
            else
            {
                return (KERNEL_MSG_SAVED);
            }
            break;
        }

        switch (msgid)
        {
            #if((BLE_2MBPS) && !(BLE_QUALIF))
            case HCI_LE_ENH_RX_TEST_CMD_OPCODE:
            #endif
            case HCI_LE_RX_TEST_CMD_OPCODE:
            {
                // Get the param and start rx test
                status = llm_test_mode_start_rx(param, msgid);
            } break;
            #if((BLE_2MBPS) && !(BLE_QUALIF))
            case HCI_LE_ENH_TX_TEST_CMD_OPCODE:
            #endif
            case HCI_LE_TX_TEST_CMD_OPCODE:
            {
                //  Get the param and start tx test
                status = llm_test_mode_start_tx(param, msgid);
            } break;
            case HCI_LE_TEST_END_CMD_OPCODE:
            {
                 // allocate the complete event message
                 struct hci_test_end_cmd_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_TEST_END_CMD_OPCODE, hci_test_end_cmd_cmp_evt);

                 // gets random number
                 event->nb_packet_received = 0;

                 // update the status
                 event->status = COMMON_ERROR_COMMAND_DISALLOWED;

                 // send the message
                 hci_send_2_host(event);


            } return (KERNEL_MSG_CONSUMED);
            //No break but return
            default:
            {
                ASSERT_ERR(0);
            } break;
        }
    } while(0);
		
    // Send the command complete event
	//	Delay_ms(200);
    llm_common_cmd_complete_send(src_id, status);

    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_TEST_MODE_SUPPORT)


/// The message handlers for HCI command complete events
static const struct kernel_msg_handler llm_hci_command_handler_tab[] =
{
    /// low energy commands
    {HCI_LE_SET_EVT_MASK_CMD_OPCODE,            (kernel_msg_func_t)hci_le_set_evt_mask_cmd_handler},
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_BUFF_SIZE_CMD_OPCODE,            (kernel_msg_func_t)hci_le_rd_buff_size_cmd_handler},
    #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,     (kernel_msg_func_t)hci_le_rd_local_supp_feats_cmd_handler},
    {HCI_LE_SET_RAND_ADDR_CMD_OPCODE,           (kernel_msg_func_t)hci_le_set_rand_add_cmd_handler},

    #if (BLE_BROADCASTER || BLE_PERIPHERAL)
    {HCI_LE_SET_ADV_PARAM_CMD_OPCODE,           (kernel_msg_func_t)hci_le_set_adv_param_cmd_handler},
    {HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE,       (kernel_msg_func_t)hci_le_rd_adv_ch_tx_pw_cmd_handler},
    {HCI_LE_SET_ADV_DATA_CMD_OPCODE,            (kernel_msg_func_t)hci_le_set_adv_data_cmd_handler},
    {HCI_LE_SET_ADV_EN_CMD_OPCODE,              (kernel_msg_func_t)hci_le_set_adv_en_cmd_handler},
    {HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE,       (kernel_msg_func_t)hci_le_set_scan_rsp_data_cmd_handler},
    #endif //(BLE_BROADCASTER || BLE_PERIPHERAL)

    #if (BLE_OBSERVER || BLE_CENTRAL)
    {HCI_LE_SET_SCAN_PARAM_CMD_OPCODE,          (kernel_msg_func_t)hci_le_set_scan_param_cmd_handler},
    {HCI_LE_SET_SCAN_EN_CMD_OPCODE,             (kernel_msg_func_t)hci_le_set_scan_en_cmd_handler},
    #endif //(BLE_CENTRAL || BLE_OBSERVER)

    #if (BLE_CENTRAL)
    {HCI_LE_CREATE_CON_CMD_OPCODE,              (kernel_msg_func_t)hci_le_create_con_cmd_handler},
    {HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE,       (kernel_msg_func_t)hci_le_create_con_cancel_cmd_handler},
    {HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE,       (kernel_msg_func_t)hci_le_set_host_ch_class_cmd_handler},
    #endif //(BLE_CENTRAL)

    {HCI_LE_RD_WLST_SIZE_CMD_OPCODE,            (kernel_msg_func_t)hci_le_rd_wl_size_cmd_handler},
    {HCI_LE_CLEAR_WLST_CMD_OPCODE,              (kernel_msg_func_t)hci_le_wl_mngt_cmd_handler},
    {HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE,         (kernel_msg_func_t)hci_le_wl_mngt_cmd_handler},
    {HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE,       (kernel_msg_func_t)hci_le_wl_mngt_cmd_handler},
    {HCI_LE_ENC_CMD_OPCODE,                     (kernel_msg_func_t)hci_le_enc_cmd_handler},
    {HCI_LE_RAND_CMD_OPCODE,                    (kernel_msg_func_t)hci_le_rand_cmd_handler},
    {HCI_LE_RD_SUPP_STATES_CMD_OPCODE,          (kernel_msg_func_t)hci_le_rd_supp_states_cmd_handler},
#if !(BLE_QUALIF)
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (kernel_msg_func_t)hci_le_rd_suggted_dft_data_len_cmd_handler},
    {HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (kernel_msg_func_t)hci_le_wr_suggted_dft_data_len_cmd_handler},
    {HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE,         (kernel_msg_func_t)hci_le_rd_max_data_len_cmd_handler},
    #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
#endif
    #if (BLE_TEST_MODE_SUPPORT)
    {HCI_LE_RX_TEST_CMD_OPCODE,                 (kernel_msg_func_t)hci_le_test_mode_mngt_cmd_handler},
    {HCI_LE_TX_TEST_CMD_OPCODE,                 (kernel_msg_func_t)hci_le_test_mode_mngt_cmd_handler},
    {HCI_LE_TEST_END_CMD_OPCODE,                (kernel_msg_func_t)hci_le_test_mode_mngt_cmd_handler},
    #endif //(BLE_TEST_MODE_SUPPORT)

    /// Legacy commands
    #if !BT_EMB_PRESENT
    {HCI_RESET_CMD_OPCODE,                       (kernel_msg_func_t)hci_reset_cmd_handler},
    {HCI_RD_BD_ADDR_CMD_OPCODE,                  (kernel_msg_func_t)hci_rd_bd_addr_cmd_handler},
    {HCI_RD_LOCAL_VER_INFO_CMD_OPCODE,           (kernel_msg_func_t)hci_rd_local_ver_info_cmd_handler},
    {HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE,          (kernel_msg_func_t)hci_rd_local_supp_cmds_cmd_handler},
    {HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,         (kernel_msg_func_t)hci_rd_local_supp_feats_cmd_handler},
    {HCI_SET_CTRL_TO_HOST_FLOW_CTRL_CMD_OPCODE,  (kernel_msg_func_t)hci_set_ctrl_to_host_flow_ctrl_cmd_handler},
    {HCI_SET_EVT_MASK_CMD_OPCODE,                (kernel_msg_func_t)hci_set_evt_mask_cmd_handler},
    {HCI_SET_EVT_MASK_PAGE_2_CMD_OPCODE,         (kernel_msg_func_t)hci_set_evt_mask_page_2_cmd_handler},
    {HCI_HOST_BUF_SIZE_CMD_OPCODE,               (kernel_msg_func_t)hci_host_buf_size_cmd_handler},
    {HCI_HOST_NB_CMP_PKTS_CMD_OPCODE,            (kernel_msg_func_t)hci_host_nb_cmp_pkts_cmd_handler},
    {HCI_RD_BUFF_SIZE_CMD_OPCODE,                (kernel_msg_func_t)hci_rd_buff_size_cmd_handler},
    #endif // !BT_EMB_PRESENT
#if !(BLE_QUALIF)
    /// Privacy 1.2
    {HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE,    (kernel_msg_func_t)hci_le_enh_privacy_mgmt_cmd_handler},
    {HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE,  (kernel_msg_func_t)hci_le_enh_privacy_mgmt_cmd_handler},
    {HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE,         (kernel_msg_func_t)hci_le_enh_privacy_mgmt_cmd_handler},
    {HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE,       (kernel_msg_func_t)hci_le_enh_privacy_mgmt_cmd_handler},
    {HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE,   (kernel_msg_func_t)hci_le_enh_privacy_mgmt_cmd_handler},
    {HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE,       (kernel_msg_func_t)hci_le_enh_privacy_info_cmd_handler},
    {HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE,        (kernel_msg_func_t)hci_le_enh_privacy_info_cmd_handler},
    {HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE,       (kernel_msg_func_t)hci_le_enh_privacy_info_cmd_handler},

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (SECURE_CONNECTIONS)
    /// BT 4.2 Secure Connections
    {HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE,     (kernel_msg_func_t)hci_le_rd_local_p256_public_key_cmd_handler},
    {HCI_LE_GEN_DHKEY_CMD_OPCODE,               (kernel_msg_func_t)hci_le_generate_dhkey_cmd_handler},
    #endif // (SECURE_CONNECTIONS)
    #if(BLE_2MBPS)
    {HCI_LE_SET_DFT_PHY_CMD_OPCODE,             (kernel_msg_func_t)hci_le_set_dft_phy_cmd_handler},
    {HCI_LE_ENH_RX_TEST_CMD_OPCODE,             (kernel_msg_func_t)hci_le_test_mode_mngt_cmd_handler},
    {HCI_LE_ENH_TX_TEST_CMD_OPCODE,             (kernel_msg_func_t)hci_le_test_mode_mngt_cmd_handler}
    #endif
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
#endif //#if !(BLE_QUALIF)
};

/**
 ****************************************************************************************
 * @brief Handles any HCI command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_command_handler(kernel_msg_id_t const msgid, void const *param, kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    int return_status = KERNEL_MSG_CONSUMED;
    int i = 0;
  
    // Check if there is a handler corresponding to the original command opcode
    for( i = 0; i < (sizeof(llm_hci_command_handler_tab)/sizeof(llm_hci_command_handler_tab[0])); i++)
    {
        // Check if opcode matches
        if(llm_hci_command_handler_tab[i].id == src_id)
        {
            // Check if there is a handler function
            if(llm_hci_command_handler_tab[i].func != NULL)
            {

		
                // Call handler
                return_status = llm_hci_command_handler_tab[i].func(src_id, param, src_id, src_id);
		
            }
            break;
        }
    }
    return (return_status);
}


/// @} LLMTASK
