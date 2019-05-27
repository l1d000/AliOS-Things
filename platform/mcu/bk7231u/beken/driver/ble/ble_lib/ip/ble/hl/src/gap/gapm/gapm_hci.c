/**
 ****************************************************************************************
 *
 * @file gapm_hci.c
 *
 * @brief Generic Access Profile Manager HCI handler implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_HCI Generic Access Profile Manager HCI Hander
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwble_config.h"

#include "rwble_hl.h"

#include <string.h>
#include "common_bt.h"
#include "common_error.h"
#include "common_utils.h"
#include "common_version.h"

#include "gapm_int.h"
#include "gapm_task.h"
#include "gapm_util.h"

#include "l2cc_pdu.h"
#include "l2cm.h"

#include "smpm_api.h" // Access to internal API required

#include "hci.h"

#if (NVDS_SUPPORT)
#include "nvds.h"               // NVDS definitions
#endif //(NVDS_SUPPORT)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Handles common complete event for configuration and reset purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_cfg_handler(kernel_msg_id_t const msgid, struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    switch (msgid)
    {
        // LL reset completed
        case HCI_RESET_CMD_OPCODE:
        {
            gapm_op_reset_continue(GAPM_OP_RESET_HCI, RW_ERR_HCI_TO_HL(event->status));
        }
        break;

        case HCI_SET_EVT_MASK_CMD_OPCODE:
        {
            gapm_op_reset_continue(GAPM_OP_RESET_SET_EVT_MASK, RW_ERR_HCI_TO_HL(event->status));
        }
        break;

        case HCI_LE_SET_EVT_MASK_CMD_OPCODE:
        {
            #if BLE_DEBUG
            // event not triggered by a reset, managed by debug
            if(gapm_get_operation(GAPM_OP_CFG) == GAPM_NO_OP)
            {
                gapm_op_setup_continue(GAPM_OP_SETUP_SET_4_0_LE_EVT_MASK, RW_ERR_HCI_TO_HL(event->status));
            }
            else
            #endif // BLE_DEBUG
            {
                gapm_op_reset_continue(GAPM_OP_RESET_LE_SET_EVT_MASK, RW_ERR_HCI_TO_HL(event->status));
            }
        }
        break;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        // Set Host channel map completed
        case HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE:
        {
            // finish operation execution with given status code
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
        }
        break;
        #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

        // Set suggested default data length completed
        case HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE:
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_WR_LE_DFT_DATA_LEN_CMD, RW_ERR_HCI_TO_HL(event->status));
        }
        break;

        #if (BLE_2MBPS)
        // Set default PHY  command completed
        case HCI_LE_SET_DFT_PHY_CMD_OPCODE:
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_SET_LE_DFT_PHY_CMD, RW_ERR_HCI_TO_HL(event->status));
        }
        break;
        #endif // (BLE_2MBPS)

        default: /* Nothing to do */ break;
    } /* end of switch */

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles LE read buffer size command complete event.
 * Used to read the maximum size of the data portion of HCI LE ACL Data Packets sent
 * from the Host to the Controller.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_buff_size_cmd_cmp_evt_handler(kernel_msg_id_t const msgid, struct hci_le_rd_buff_size_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    ASSERT_ERR(event->status == COMMON_ERROR_NO_ERROR);

    if(event->status == COMMON_ERROR_NO_ERROR)
    {
        /* update the buffer size */
        l2cm_set_link_layer_buff_size(event->hc_data_pk_len, event->hc_tot_nb_data_pkts);
    }

    gapm_op_reset_continue(GAPM_OP_RESET_LE_RD_BUFF_SIZE, RW_ERR_HCI_TO_HL(event->status));


    // message is consumed
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles read buffer size command complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_buff_size_cmd_cmp_evt_handler(kernel_msg_id_t const msgid, struct hci_rd_buff_size_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    ASSERT_ERR(event->status == COMMON_ERROR_NO_ERROR);

    // sanity check
    ASSERT_ERR(event->hc_tot_nb_data_pkts != 0);

    if(event->status == COMMON_ERROR_NO_ERROR)
    {
        /* update the buffer size */
        l2cm_set_link_layer_buff_size(event->hc_data_pk_len, event->hc_tot_nb_data_pkts);
    }

    gapm_op_reset_continue(GAPM_OP_RESET_RD_BUFF_SIZE, RW_ERR_HCI_TO_HL(event->status));

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Handles No Operation Complete Event from lower layer.
 * When No Operation Complete Event is received from link layer.
 * Host will issue reset command to link layer (see reset procedure)
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_no_operation_cmd_cmp_evt_handler(kernel_msg_id_t const msgid, void const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // inform host application that lower layers are ready but do nothing
    kernel_msg_send_basic(GAPM_DEVICE_READY_IND, APP_MAIN_TASK, TASK_GAPM);

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles the read Bluetooth device version complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_local_ver_info_cmd_cmp_evt_handler(kernel_msg_id_t const msgid, struct hci_rd_local_ver_info_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    if(state != GAPM_DEVICE_SETUP)
    {
        // check if there is no protocol issues.
        if(gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_DEV_VERSION)
        {
            // finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            /* status should be OK */
            ASSERT_ERR(event->status == COMMON_ERROR_NO_ERROR);

            // send device version value
            if(event->status == COMMON_ERROR_NO_ERROR)
            {
                struct gapm_dev_version_ind *version_ind =
                        KERNEL_MSG_ALLOC(GAPM_DEV_VERSION_IND, gapm_get_requester(GAPM_OP_CFG),
                                dest_id, gapm_dev_version_ind);
                /* fill up the parameters */
                /* HCI version */
                version_ind->hci_ver = event->hci_ver;
                /* HCI sub version */
                version_ind->hci_subver = event->hci_rev;
                /* LMP sub version */
                version_ind->lmp_subver = event->lmp_subver;
                /* LMP version */
                version_ind->lmp_ver = event->lmp_ver;
                /* Manufacturing name */
                version_ind->manuf_name = event->manuf_name;
                /* Host version */
                version_ind->host_ver = RWBT_SW_VERSION_MAJOR;
                /* Host sub version */
                version_ind->host_subver = COMMON_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR,
                                                               RWBT_SW_VERSION_BUILD);

                /* send the message indication */
                kernel_msg_send(version_ind);
            }

            // finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the read Bluetooth device address complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_bd_addr_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_rd_bd_addr_cmd_cmp_evt const *event, kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    /* status should be OK */
    ASSERT_ERR(event->status == COMMON_ERROR_NO_ERROR);

    /* Store Local BD address */
    memcpy(&(gapm_env.addr), &(event->local_addr), BD_ADDR_LEN);

    if(gapm_get_operation(GAPM_OP_CFG) == GAPM_SET_DEV_CONFIG)
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_ADDR_MGT, RW_ERR_HCI_TO_HL(event->status));
    }
    else
    {
        gapm_op_reset_continue(GAPM_OP_RESET_RD_BD_ADDR, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles the LE read local public key complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] cmp_evt   Pointer to the parameters of the event.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 
static int hci_le_rd_local_p256_public_key_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_rd_local_p256_public_key_cmp_evt const *cmp_evt,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    if(cmp_evt->status == COMMON_ERROR_NO_ERROR)
    {
        // Current process state
        memcpy(&gapm_env.public_key.x[0],&cmp_evt->public_key[0],32);
        memcpy(&gapm_env.public_key.y[0],&cmp_evt->public_key[32],32);

        #if (NVDS_SUPPORT)
        if (NVDS_OK != nvds_put(NVDS_TAG_LE_PUBLIC_KEY_P256, NVDS_LEN_LE_PUBLIC_KEY_P256, (uint8_t*)&cmp_evt->public_key[0]))
        {
            // Could not write new secret key to NVDS
        }
        #endif //(NVDS_SUPPORT)
    }

    gapm_op_setup_continue(GAPM_OP_SETUP_RD_PRIV_KEY, RW_ERR_HCI_TO_HL(cmp_evt->status));

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LE generate dh key complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the event (32 byte DH Key)
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_generate_dhkey_cmp_evt_handler(kernel_msg_id_t const msgid,
                                      struct hci_le_generate_dhkey_cmp_evt const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (kernel_state_get(dest_id) != GAPM_DEVICE_SETUP)
    {
        if (gapm_get_operation(GAPM_OP_DHKEY)== GAPM_GEN_DH_KEY)
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // Check the returned status
            if (param->status == GAP_ERR_NO_ERROR)
            {
                // Indication sent to the requester
                struct gapm_gen_dh_key_ind *ind = KERNEL_MSG_ALLOC(GAPM_GEN_DH_KEY_IND,
                                                               gapm_get_requester(GAPM_OP_DHKEY), dest_id,
                                                               gapm_gen_dh_key_ind);


                memcpy(&ind->result[0], &param->dh_key[0], 32);

                // Send the indication
                kernel_msg_send(ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(param->status);
            }

            // Send the command complete event message
            gapm_send_complete_evt(GAPM_OP_DHKEY, status);
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

#endif // (SECURE_CONNECTIONS)


/**
 ****************************************************************************************
 * @brief Read white list size complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_wlst_size_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_rd_wlst_size_cmd_cmp_evt const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    if(state != GAPM_DEVICE_SETUP)
    {
        // check if there is no protocol issues.
        if(gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_WLIST_SIZE)
        {
            // finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // send white list size indication
            if(param->status == COMMON_ERROR_NO_ERROR)
            {
                struct gapm_white_list_size_ind *wlist_size_ind =
                        KERNEL_MSG_ALLOC(GAPM_WHITE_LIST_SIZE_IND, gapm_get_requester(GAPM_OP_CFG),
                                dest_id, gapm_white_list_size_ind);
                /* fill up the parameters */
                wlist_size_ind->size = param->wlst_size;
                /* send the message indication */
                kernel_msg_send(wlist_size_ind);
            }

            // finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(param->status));
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}
#endif /* (BLE_CENTRAL || BLE_PERIPHERAL) */



/**
 ****************************************************************************************
 * @brief Handles common complete event for White list management purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_wl_handler(kernel_msg_id_t const msgid, struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
	
	//UART_PRINTF("%s\r\n",__func__);
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;

    if(state != GAPM_DEVICE_SETUP)
    {
        bool op_finished = true;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        uint8_t operation = gapm_get_operation(GAPM_OP_CFG);

        switch (msgid)
        {
            case HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE:
            case HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE:
            {
                // check if there is no protocol issues.
                if((operation == GAPM_ADD_DEV_IN_WLIST) || (operation == GAPM_RMV_DEV_FRM_WLIST))
                {
                    // send white list size indication
                    if(event->status == COMMON_ERROR_NO_ERROR)
                    {
                        // retrieve operation
                        struct gapm_white_list_mgt_cmd* white_list_mgt_cmd =
                                (struct gapm_white_list_mgt_cmd*) gapm_get_operation_ptr(GAPM_OP_CFG);

                        // decrement cursor to continue update of white list
                        white_list_mgt_cmd->nb--;

                        if(white_list_mgt_cmd->nb > 0)
                        {
                            // continue operation execution
                            op_finished = !(gapm_reschedule_operation(GAPM_OP_CFG));
                            break; // exit from case in order to not send command complete
                        }
                        // finish operation execution because operation finished.
                        status = GAP_ERR_NO_ERROR;
                    }
                    else
                    {
                        status = RW_ERR_HCI_TO_HL(event->status);
                    }
                }
                // else finish operation execution with an error: unexpected response received
            }
            break;

            case HCI_LE_CLEAR_WLST_CMD_OPCODE:
            {
                // check if there is no protocol issues.
                if((operation == GAPM_CLEAR_WLIST))
                {
                    // finish operation execution because operation finished.
                    status = RW_ERR_HCI_TO_HL(event->status);
                }
                // else finish operation execution with an error: unexpected response received
            }
            break;

            default:
            {
                /* Nothing to do */
            }
            break;
        } /* end of switch */
        #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

        // check that operation not rescheduled
        if(op_finished)
        {
            // send command complete
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }

    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL || BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Handles common complete event for advertising purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_adv_handler(kernel_msg_id_t const msgid, struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    bool update_air_state = true;

    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t new_state = GAPM_OP_ERROR;

        // retrieve air operation
        struct gapm_air_operation* air_op =
                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

        if ((event->status == COMMON_ERROR_NO_ERROR) && (air_op != NULL))
        {
            switch (msgid)
            {
                // Set Advertising parameters + data completed
                case HCI_LE_SET_ADV_PARAM_CMD_OPCODE:
                {
                    new_state = GAPM_OP_SET_PARAMS;
					//UART_PRINTF("ADV_Step2_1 : HCI_LE_SET_ADV_PARAM_CMD_OPCODE command complete\r\n");
                }
                break;
                // Set Advertising data completed
                case HCI_LE_SET_ADV_DATA_CMD_OPCODE:
                {
                    new_state = GAPM_OP_SET_ADV_DATA;
					//UART_PRINTF("ADV_Step3_1 : HCI_LE_SET_ADV_DATA_CMD_OPCODE command complete\r\n");
                }
                break;
                // Set scan response data completed
                case HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE:
                {
                    new_state = GAPM_OP_SET_SCAN_RSP_DATA;
					//UART_PRINTF("ADV_Step4_1 : HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE command complete\r\n");
                }
                break;
                // set advertising state completed
                case HCI_LE_SET_ADV_EN_CMD_OPCODE:
                {
                    new_state = ((GAPM_GET_OP_STATE(*air_op) == GAPM_OP_START)
                                ? GAPM_OP_START : GAPM_OP_STOP);
					//UART_PRINTF("ADV_Step5_1 : HCI_LE_SET_ADV_EN_CMD_OPCODE command complete new_state = 0x%x\r\n",new_state);
                }/* end of switch */
                break;
                default: /* Nothing to so*/ break;
            } /* end of switch */
        }

        // if advertising data should be updated on the fly
        if(gapm_get_operation(GAPM_OP_CFG) == GAPM_UPDATE_ADVERTISE_DATA)
        {
            bool completed = false;
            if(new_state == GAPM_OP_SET_ADV_DATA)
            {
                struct gapm_update_advertise_data_cmd *up_adv =
                        (struct gapm_update_advertise_data_cmd *) gapm_get_operation_ptr(GAPM_OP_CFG);
                update_air_state = false;

                // send scan response data
                if(event->status == GAP_ERR_NO_ERROR)
                {
                    struct hci_le_set_scan_rsp_data_cmd *scan_resp = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE, hci_le_set_scan_rsp_data_cmd);
                    // retrieve scan response data length
                    scan_resp->scan_rsp_data_len =up_adv->scan_rsp_data_len;
                    // copy provided scan response data
                    memcpy(&(scan_resp->data), up_adv->scan_rsp_data, GAP_SCAN_RSP_DATA_LEN);

                    hci_send_2_controller(scan_resp);
                }
                // an error occurs, terminate
                else
                {
                    completed = true;
                }
            }

            if(new_state == GAPM_OP_SET_SCAN_RSP_DATA)
            {
                update_air_state = false;
                completed = true;
            }

            // send command complete
            if(completed)
            {
                gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
            }
        }

        if(update_air_state)
        {
            // update state according to current state
            gapm_update_air_op_state(GAPM_OP_AIR, new_state, RW_ERR_HCI_TO_HL(event->status));
			//UART_PRINTF("ADV_Step6_1 : gapm_update_air_op_state\r\n");
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#endif /* #if (BLE_PERIPHERAL || BLE_BROADCASTER) */




#if (BLE_CENTRAL || BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Handles common complete event for scanning purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_scan_handler(kernel_msg_id_t const msgid, struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t new_state = GAPM_OP_ERROR;
        // retrieve air operation
        struct gapm_air_operation* air_op =
                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

        if ((event->status == COMMON_ERROR_NO_ERROR) && (air_op != NULL))
        {
            switch (msgid)
            {
                // Set Scanning parameters completed
                case HCI_LE_SET_SCAN_PARAM_CMD_OPCODE:
                {
                    new_state = GAPM_OP_SET_PARAMS;
                }
                break;
                // set scanning state completed
                case HCI_LE_SET_SCAN_EN_CMD_OPCODE:
                {
                    // check current operation state
                    new_state = ((GAPM_GET_OP_STATE(*air_op) == GAPM_OP_START)
                                ? GAPM_OP_START : GAPM_OP_STOP);
                }
                break;

                default: /* Nothing to so*/ break;
            } /* end of switch */
        }

        // update state according to current state
        gapm_update_air_op_state(GAPM_OP_AIR, new_state, RW_ERR_HCI_TO_HL(event->status));
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles advertising report event from link layer.
 * Process the advertising report event from link layer.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_adv_report_evt_handler(kernel_msg_id_t const msgid, struct hci_le_adv_report_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // sanity check
    if(state != GAPM_DEVICE_SETUP)
    {
        // Get operation
        uint8_t operation = gapm_get_operation(GAPM_OP_AIR);
        // retrieve air operation
        struct gapm_air_operation* air_op =
                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);
        // Get advertising mode
        uint8_t mode = gapm_get_adv_mode(operation, air_op);

        // parse advertising report values. if invalid mode, do nothing
        for (uint8_t cursor = 0; (cursor < event->nb_reports) && (mode != GAP_INVALID_MODE); cursor++)
        {
            bool filter = false;

            // Do not filter observer command
            if(mode != GAP_OBSERVER_MODE)
            {
                // Advertising data
                if(event->adv_rep[cursor].evt_type != LL_SCAN_RSP)
                {
                    // retrieve ad_type flag
                    uint8_t ad_type =  gapm_get_ad_type_flag((uint8_t*)event->adv_rep[cursor].data,
                            event->adv_rep[cursor].data_len);

                    /* check for limited discovery that receive limited discoverable data
                     * or
                     * for general discovery that one of Limited or general discoverable
                     * flag is present in ad_type flag
                     */
                    if((GAPM_ISBITSET(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG)
                            && (mode == GAP_LIM_DISCOVERY))
                            || (((GAPM_ISBITSET(ad_type, GAP_LE_GEN_DISCOVERABLE_FLG))
                                    || (GAPM_ISBITSET(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG)))
                                    && (mode == GAP_GEN_DISCOVERY)))

                    {
                        // add device to filter.
                        gapm_add_to_filter((bd_addr_t*) &(event->adv_rep[cursor].adv_addr),
                                event->adv_rep[cursor].adv_addr_type);
                    }
                    else
                    {
                        filter = true;
                    }
                }
                // scan response data
                else
                {
                    // check if device not filtered for scan response report
                    filter = gapm_is_filtered((bd_addr_t*) &(event->adv_rep[cursor].adv_addr),
                            event->adv_rep[cursor].adv_addr_type);
                }
            }

            // check if advertising report filtered
            if(!filter)
            {
                // send advertising report to requester
                struct gapm_adv_report_ind* report = KERNEL_MSG_ALLOC(GAPM_ADV_REPORT_IND,
                        gapm_get_requester(GAPM_OP_AIR), dest_id, gapm_adv_report_ind);

                // fill data parameters
                memcpy(&(report->report), &(event->adv_rep[cursor]), sizeof(struct adv_report));

                // send adv report indication.
                kernel_msg_send(report);
            }
        }

        // unexpected error
        if(mode == GAP_INVALID_MODE)
        {
            // update state according to current state
            gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_ERROR, GAP_ERR_PROTOCOL_PROBLEM);
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles direct advertising report event from link layer.
 * Process the advertising report event from link layer.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_dir_adv_report_evt_handler(kernel_msg_id_t const msgid, struct hci_le_dir_adv_rep_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // sanity check
    if(state != GAPM_DEVICE_SETUP)
    {
        // Get operation
        uint8_t operation = gapm_get_operation(GAPM_OP_AIR);
        // retrieve air operation
        struct gapm_air_operation* air_op =
                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);
        // Get advertising mode
        uint8_t mode = gapm_get_adv_mode(operation, air_op);

        // parse advertising report values. if invalid mode, do nothing
        for (uint8_t cursor = 0; (cursor < event->nb_reports) && (mode != GAP_INVALID_MODE); cursor++)
        {
            // send advertising report to requester
            struct gapm_adv_report_ind* report = KERNEL_MSG_ALLOC(GAPM_ADV_REPORT_IND,
                    gapm_get_requester(GAPM_OP_AIR), dest_id, gapm_adv_report_ind);

            // fill data parameters
            report->report.evt_type = event->adv_rep[cursor].evt_type;
            report->report.adv_addr_type = event->adv_rep[cursor].addr_type;
            memcpy(&(report->report.adv_addr), &(event->adv_rep[cursor].addr), BD_ADDR_LEN);

            // Note that data length is reused for directed address type
            report->report.data_len = event->adv_rep[cursor].dir_addr_type;
            // Note that data is reused for directed address
            memcpy(&(report->report.data), &(event->adv_rep[cursor].dir_addr), BD_ADDR_LEN);
            report->report.rssi = event->adv_rep[cursor].rssi;

            // send adv report indication.
            kernel_msg_send(report);
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#endif // (BLE_CENTRAL || BLE_OBSERVER)


#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles common complete event for connection purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_connect_handler(kernel_msg_id_t const msgid,
        struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t new_state = GAPM_OP_ERROR;
        // retrieve air operation
        struct gapm_air_operation* air_op =
                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

        if ((event->status == COMMON_ERROR_NO_ERROR) && (air_op != NULL))
        {
            switch (msgid)
            {
                // Receives connection status event from the link layer
                case HCI_LE_CREATE_CON_CMD_OPCODE:
                {
                    // nothing to do
                    new_state = GAPM_OP_START;
                }
                break;

                // Cancel creation of a connection
                case HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE:
                {
                    // do nothing wait for HCI_CREATE_CON_CMD_COMPLETE
                    new_state = GAPM_OP_INIT;
                }
                break;
                default: /* Nothing to so*/ break;
            } /* end of switch */
        }

        if(new_state != GAPM_OP_INIT)
        {
            // update state according to current state
            gapm_update_air_op_state(GAPM_OP_AIR, new_state, RW_ERR_HCI_TO_HL(event->status));
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#endif // (BLE_CENTRAL)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles connection complete event from the lower layer.
 * This handler is responsible for initiating the creation of L2CAP channel.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_con_cmp_evt_handler(kernel_msg_id_t const msgid, struct hci_le_con_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        struct hci_le_enh_con_cmp_evt data;

        // Copy parameters
        data.subcode        = event->subcode;
        data.status         = event->status;
        data.conhdl         = event->conhdl;
        data.role           = event->role;
        data.peer_addr_type = event->peer_addr_type;
        memcpy(&(data.peer_addr), &(event->peer_addr),
                sizeof(bd_addr_t));
        data.con_interval   = event->con_interval;
        data.con_latency    = event->con_latency;
        data.sup_to         = event->sup_to;
        data.clk_accuracy   = event->clk_accuracy;

        // Configure connection
        gapm_setup_conn(msgid, &data);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles enhanced connection complete event from the lower layer.
 * This handler is responsible for initiating the creation of L2CAP channel.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_enh_con_cmp_evt_handler(kernel_msg_id_t const msgid, struct hci_le_enh_con_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        // Configure connection
        gapm_setup_conn(msgid, event);
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Handles common complete event for address management purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_addr_set_handler(kernel_msg_id_t const msgid,
        struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t new_state = GAPM_OP_ERROR;

        // terminate set Device configuration operation
        if(gapm_get_operation(GAPM_OP_CFG) == GAPM_SET_DEV_CONFIG)
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_ADDR_MGT, RW_ERR_HCI_TO_HL(event->status));
        }
        else
        {
            // retrieve air operation
            struct gapm_air_operation* air_op =
                    (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);


            if ((event->status == COMMON_ERROR_NO_ERROR) && (air_op != NULL))
            {
                switch (msgid)
                {
                    // Set Random address command completed
                    case HCI_LE_SET_RAND_ADDR_CMD_OPCODE:
                    {
                        // indicate to host new random BD address set in lower layers
                        struct gapm_dev_bdaddr_ind *bdaddr_ind = KERNEL_MSG_ALLOC(GAPM_DEV_BDADDR_IND,
                                gapm_get_requester(GAPM_OP_AIR), TASK_GAPM, gapm_dev_bdaddr_ind);
                        /* fill up the parameters */
                        memcpy(&(bdaddr_ind->addr.addr), &(gapm_env.addr), BD_ADDR_LEN);
                        /* public bd address */
                        bdaddr_ind->addr.addr_type = ADDR_RAND;

                        /* send the message indication */
                        kernel_msg_send(bdaddr_ind);

                        new_state = GAPM_OP_ADDR_SET;
                    }
                    break;
                    default: /* Nothing to so*/ break;
                } /* end of switch */
            }

            if(new_state != GAPM_OP_INIT)
            {
                // update state according to current state
                gapm_update_air_op_state(GAPM_OP_AIR, new_state, RW_ERR_HCI_TO_HL(event->status));
            }
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

#if ((BLE_OBSERVER) || (BLE_PERIPHERAL))

/**
 ****************************************************************************************
 * @brief Handles LE read adv tx power level value.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;

    // sanity check
    if(gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_DEV_ADV_TX_POWER)
    {
        if((event->status == COMMON_ERROR_NO_ERROR))
        {
            // send event to upper layers.
            struct gapm_dev_adv_tx_power_ind *ind = KERNEL_MSG_ALLOC(GAPM_DEV_ADV_TX_POWER_IND,
                    gapm_get_requester(GAPM_OP_CFG),  dest_id, gapm_dev_adv_tx_power_ind);

            ind->power_lvl = event->adv_tx_pw_lvl;

            /* send the indication */
            kernel_msg_send(ind);
            status = GAP_ERR_NO_ERROR;
        }
        else
        {
            status = RW_ERR_HCI_TO_HL(event->status);
        }
    }

    gapm_send_complete_evt(GAPM_OP_CFG, status);

    return (KERNEL_MSG_CONSUMED);
}
#endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))


/**
 ****************************************************************************************
 * @brief HCI LE Rand command complete event handler.
 * The received random number generated by the lower layers will be sent to the saved
 * SMPC task ID.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return Received kernel message status (consumed or not).
 ****************************************************************************************
 */
static int hci_le_rand_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
                                       struct hci_le_rand_cmd_cmp_evt const *event,
                                       kernel_task_id_t const dest_id,
                                       kernel_task_id_t const src_id)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (kernel_state_get(dest_id) != GAPM_DEVICE_SETUP)
    {
        switch (gapm_get_operation(GAPM_OP_CFG))
        {
            // Generation Random Address
            case (GAPM_GEN_RAND_ADDR):
            {
                // Check the returned status
                if (event->status == GAP_ERR_NO_ERROR)
                {
                    // Get the command request
                    struct gapm_gen_rand_addr_cmd *cmd = (struct gapm_gen_rand_addr_cmd*)gapm_get_operation_ptr(GAPM_OP_CFG);
                    // Prand' =  0[0:12] || prand (MSB->LSB in prand_bis)
                    uint8_t prand_bis[GAP_KEY_LEN];

                    // Generate prand value (MSB->LSB required for encryption function)
                    memcpy(&cmd->prand[0], &event->nb.nb[0], GAP_ADDR_PRAND_LEN);

                    // Set the address type flag, clear the 2 MSBits of prand
                    cmd->prand[GAP_ADDR_PRAND_LEN - 1] &= 0x3F;
                    // Set the address type value, MSB of prand is the addr_type
                    cmd->prand[GAP_ADDR_PRAND_LEN - 1] |= cmd->rnd_type;

                    // Clear prand_bis
                    memset(&prand_bis[0], 0x00, GAP_KEY_LEN);
                    // Copy prand value in prand_bis
                    memcpy(&prand_bis[0], &cmd->prand[0], GAP_ADDR_PRAND_LEN);

                    // Encrypt the prand value using the IRK value
                    smpm_send_encrypt_req(&(gapm_get_irk()->key[0]), &prand_bis[0]);
                }
                else
                {
                    // check if requester is an air operation
                    if(gapm_get_requester(GAPM_OP_CFG) == TASK_GAPM)
                    {
                        // retrieve air operation
                        struct gapm_air_operation* air_op =
                                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

                        if(air_op != NULL)
                        {
                            // update state according to current state
                            gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_ERROR, RW_ERR_HCI_TO_HL(event->status));
                        }
                    }

                    gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
                }
            } break;

            case (GAPM_GEN_RAND_NB):
            {
                // Status
                uint8_t status = GAP_ERR_NO_ERROR;

                if (event->status == GAP_ERR_NO_ERROR)
                {
                    // Send the generated random number to the requester task.
                    struct gapm_gen_rand_nb_ind *ind = KERNEL_MSG_ALLOC(GAPM_GEN_RAND_NB_IND,
                                                                    gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                                                    gapm_gen_rand_nb_ind);

                    memcpy(&ind->randnb.nb[0], &event->nb.nb[0], GAP_RAND_NB_LEN);

                    kernel_msg_send(ind);
                }
                else
                {
                    status = RW_ERR_HCI_TO_HL(event->status);
                }

                // Send a CMP_EVT message with the request status.
                gapm_send_complete_evt(GAPM_OP_CFG, status);
            } break;

            default:
            {
                // Drop the message
                ASSERT_ERR(0);
            } break;
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief HCI LE Encrypt command complete event handler.
 * The received encrypted data is to be sent to the saved SMPC source task ID (which is the
 * origin of the request).
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 * @return Received kernel message status (consumed or not).
 ****************************************************************************************
 */
static int hci_le_enc_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
                                      struct hci_le_enc_cmd_cmp_evt const *param,
                                      kernel_task_id_t const dest_id,
                                      kernel_task_id_t const src_id)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (kernel_state_get(dest_id) != GAPM_DEVICE_SETUP)
    {
        switch (gapm_get_operation(GAPM_OP_CFG))
        {
            // Generation Random Address
            case (GAPM_GEN_RAND_ADDR):
            {
                uint8_t status = GAP_ERR_NO_ERROR;

                // Check the returned status
                if (param->status == GAP_ERR_NO_ERROR)
                {
                    // Get the command request
                    struct gapm_gen_rand_addr_cmd *cmd = (struct gapm_gen_rand_addr_cmd*)gapm_get_operation_ptr(GAPM_OP_CFG);

                    // check if requester is an air operation which request address generation
                    if(gapm_get_requester(GAPM_OP_CFG) == TASK_GAPM)
                    {
                        // retrieve air operation
                        struct gapm_air_operation* air_op =
                                (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

                        if(air_op != NULL)
                        {
                            /* fill up the parameters */// Set the prand part - Provided Address is LSB->MSB
                            memcpy(&(gapm_env.addr.addr[GAP_ADDR_PRAND_LEN]), &(cmd->prand[0]), GAP_ADDR_PRAND_LEN);

                            // Set the hash part
                            memcpy(&(gapm_env.addr.addr[0]), param->encrypted_data, GAP_ADDR_PRAND_LEN);
                        }
                    }
                    else
                    {
                        // Indication sent to the requester
                        struct gapm_dev_bdaddr_ind *ind = KERNEL_MSG_ALLOC(GAPM_DEV_BDADDR_IND, gapm_get_requester(GAPM_OP_CFG),
                                dest_id, gapm_dev_bdaddr_ind);

                        /* fill up the parameters */// Set the prand part - Provided Address is LSB->MSB
                        memcpy(&(ind->addr.addr.addr[GAP_ADDR_PRAND_LEN]), &(cmd->prand[0]), GAP_ADDR_PRAND_LEN);

                        // Set the hash part
                        memcpy(&(ind->addr.addr.addr[0]), param->encrypted_data, GAP_ADDR_PRAND_LEN);
                        ind->addr.addr_type = ADDR_RAND;

                        /* send the message indication */
                        kernel_msg_send(ind);
                    }
                }
                else
                {
                    status = RW_ERR_HCI_TO_HL(param->status);
                }

                // Send the command complete event message
                gapm_send_complete_evt(GAPM_OP_CFG, status);
            } break;

            case (GAPM_RESOLV_ADDR):
            {
                // Status of the request
                uint8_t status = GAP_ERR_NO_ERROR;
                // Get the command request
                struct gapm_resolv_addr_cmd *cmd = (struct gapm_resolv_addr_cmd *) gapm_get_operation_ptr(GAPM_OP_CFG);


                // Check the returned status
                if (param->status == GAP_ERR_NO_ERROR)
                {
                    /*
                     * Address provided from higher layers in LSB->MSB
                     *      ----------------------------------------------------------------------------------------
                     *      | hash[0:(GAP_ADDR_HASH_LEN-1)] | prand[(GAP_ADDR_HASH_LEN:(BD_ADDR_LEN-1)] |
                     *      ----------------------------------------------------------------------------------------
                     *
                     * Encrypted Data received from LL is LSB->MSB
                     *      ----------------------------------------------------------------------------------------------------
                     *      | hash[0:(GAP_KEY_LEN-GAP_ADDR_HASH_LEN)] | data[((GAP_KEY_LEN-GAP_ADDR_HASH_LEN):(GAP_KEY_LEN-1)] |
                     *      ----------------------------------------------------------------------------------------------------
                     */

                    // Compare the provided hash value and the generated hash value.
                    if (!memcmp(&(cmd->addr.addr[0]), &(param->encrypted_data[0]), GAP_ADDR_HASH_LEN))
                    {
                        // Address resolution completed
                        // Indicate which key has been used to resolve the random address.
                        struct gapm_addr_solved_ind *solved_ind = KERNEL_MSG_ALLOC(GAPM_ADDR_SOLVED_IND,
                                gapm_get_requester(GAPM_OP_CFG), dest_id,
                                gapm_addr_solved_ind);

                        // Provide Address resolved
                        memcpy(&(solved_ind->addr), &(cmd->addr.addr[0]), sizeof(bd_addr_t));

                        // Provide IRK used for address resolution
                        memcpy(&(solved_ind->irk), &(cmd->irk[cmd->nb_key]),
                                sizeof(struct gap_sec_key));

                        // Send the message
                        kernel_msg_send(solved_ind);
                    }
                    else
                    {
                        status = SMP_ERROR_ADDR_RESOLV_FAIL;
                    }
                }
                else
                {
                    status = RW_ERR_HCI_TO_HL(param->status);
                }

                if(status != SMP_ERROR_ADDR_RESOLV_FAIL)
                {
                    // Send the command complete event message
                    gapm_send_complete_evt(GAPM_OP_CFG, status);
                }
                else
                {
                    // Address not resolved, try with another key.
                    gapm_reschedule_operation(GAPM_OP_CFG);
                }
            } break;

            case (GAPM_USE_ENC_BLOCK):
            {
                uint8_t status = GAP_ERR_NO_ERROR;

                // Check the returned status
                if (param->status == GAP_ERR_NO_ERROR)
                {
                    // Indication sent to the requester
                    struct gapm_use_enc_block_ind *ind = KERNEL_MSG_ALLOC(GAPM_USE_ENC_BLOCK_IND,
                                                                      gapm_get_requester(GAPM_OP_CFG), dest_id,
                                                                      gapm_use_enc_block_ind);

                    // Copy the result
                    memcpy(&ind->result[0], &param->encrypted_data[0], GAP_KEY_LEN);

                    // Send the indication
                    kernel_msg_send(ind);
                }
                else
                {
                    status = RW_ERR_HCI_TO_HL(param->status);
                }

                // Send the command complete event message
                gapm_send_complete_evt(GAPM_OP_CFG, status);
            } break;

            default:
            {
                // Drop the message
                ASSERT_ERR(0);
            } break;
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE Read Suggested Default LE Data Lenght complete event handler.
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (kernel_state_get(dest_id) != GAPM_DEVICE_SETUP)
    {
        // Send indication to the APP
        struct gapm_sugg_dflt_data_len_ind *suggted_data = KERNEL_MSG_ALLOC(GAPM_SUGG_DFLT_DATA_LEN_IND,
                gapm_get_requester(GAPM_OP_CFG), dest_id,
                gapm_sugg_dflt_data_len_ind);

        // Fill parameters
        suggted_data->suggted_max_tx_octets = event->suggted_max_tx_octets;
        suggted_data->suggted_max_tx_time = event->suggted_max_tx_time;

        // Send the message
        kernel_msg_send(suggted_data);

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, event->status);
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE Read Maximum LE Data Lenght complete event handler.
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_max_data_len_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_le_rd_max_data_len_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (kernel_state_get(dest_id) != GAPM_DEVICE_SETUP)
    {
        // Send indication to the APP
        struct gapm_max_data_len_ind *max_data = KERNEL_MSG_ALLOC(GAPM_MAX_DATA_LEN_IND,
                gapm_get_requester(GAPM_OP_CFG), dest_id,
                gapm_max_data_len_ind);

        // Fill parameters
        max_data->suppted_max_rx_octets = event->suppted_max_rx_octets;
        max_data->suppted_max_rx_time = event->suppted_max_rx_time;
        max_data->suppted_max_tx_octets = event->suppted_max_tx_octets;
        max_data->suppted_max_tx_time = event->suppted_max_tx_time;

        // Send the message
        kernel_msg_send(max_data);

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, event->status);
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Read resolving list size complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_ral_size_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_le_rd_rslv_list_size_cmd_cmp_evt const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    if(state != GAPM_DEVICE_SETUP)
    {
        // check if there are no protocol issues.
        if(gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_RAL_SIZE)
        {
            // finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // send size indication
            if(param->status == COMMON_ERROR_NO_ERROR)
            {
                struct gapm_ral_size_ind *ral_size_ind =
                        KERNEL_MSG_ALLOC(GAPM_RAL_SIZE_IND, gapm_get_requester(GAPM_OP_CFG),
                                dest_id, gapm_ral_size_ind);
                /* fill up the parameters */
                ral_size_ind->size = param->size;
                /* send the message indication */
                kernel_msg_send(ral_size_ind);
            }

            // finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(param->status));
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Read resolving list addres complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_ral_addr_cmd_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t operation = gapm_get_operation(GAPM_OP_CFG);
        // check if there are no protocol issues.
        if((operation != GAPM_GET_RAL_PEER_ADDR) &&
                (operation != GAPM_GET_RAL_LOC_ADDR))
        {
            // finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // send white list size indication
            if(param->status == COMMON_ERROR_NO_ERROR)
            {
                struct gapm_ral_addr_ind *ral_addr_ind =
                        KERNEL_MSG_ALLOC(GAPM_RAL_ADDR_IND, gapm_get_requester(GAPM_OP_CFG),
                                dest_id, gapm_ral_addr_ind);
                /* fill up the parameters */
                ral_addr_ind->operation = operation;
                memcpy(&ral_addr_ind->addr, &param->peer_rslv_addr, sizeof(bd_addr_t));
                /* send the message indication */
                kernel_msg_send(ral_addr_ind);
            }

            // finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(param->status));
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles common complete event for resolving list management purpose.
 * Send the complete event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_basic_cmd_cmp_evt_rl_handler(kernel_msg_id_t const msgid, struct hci_basic_cmd_cmp_evt const *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;
    bool op_finished = true;

    if(state != GAPM_DEVICE_SETUP)
    {
        uint8_t operation = gapm_get_operation(GAPM_OP_CFG);

        switch (msgid)
        {
            case HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE:
            {
                gapm_op_setup_continue(GAPM_OP_SETUP_EN_CTRL_PRIV, RW_ERR_HCI_TO_HL(event->status));

                // Continue setting up device
                op_finished = false;
            }
            break;

            case HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE:
            {
                gapm_op_setup_continue(GAPM_OP_SETUP_SET_RENEW_TO, RW_ERR_HCI_TO_HL(event->status));

                // Continue setting up device
                op_finished = false;
            }
            break;

            case HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE:
            case HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE:
            {
                // check if there are no protocol issues.
                if((operation == GAPM_ADD_DEV_IN_RAL) || (operation == GAPM_RMV_DEV_FRM_RAL))
                {
                    status = RW_ERR_HCI_TO_HL(event->status);

                    if(status == GAP_ERR_NO_ERROR)
                    {
                        // retrieve operation
                        struct gapm_ral_mgt_cmd* ral_mgt_cmd =
                                (struct gapm_ral_mgt_cmd*) gapm_get_operation_ptr(GAPM_OP_CFG);

                        // decrement cursor to continue update of the list
                        if(--ral_mgt_cmd->nb > 0)
                        {
                            // continue operation execution
                            op_finished = !(gapm_reschedule_operation(GAPM_OP_CFG));
                            break; // exit from case in order to not send command complete
                        }
                    }
                }
                // else finish operation execution with an error: unexpected response received
            }
            break;

            case HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE:
            {
                // check if there are no protocol issues.
                if((operation == GAPM_CLEAR_RAL))
                {
                    // finish operation execution because operation finished.
                    status = RW_ERR_HCI_TO_HL(event->status);
                }
                // else finish operation execution with an error: unexpected response received
            }
            break;

            default:
            {
                /* Nothing to do */
            }
            break;
        } /* end of switch */
    }

    // check that operation not rescheduled
    if(op_finished)
    {
        // send command complete
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/// The message handlers for HCI command complete events
static const struct kernel_msg_handler gapm_hci_cmd_cmp_event_handler_tab[] =
{
    { HCI_RESET_CMD_OPCODE,                      (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_SET_EVT_MASK_CMD_OPCODE,               (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_SET_EVT_MASK_CMD_OPCODE,            (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_RD_BUFF_SIZE_CMD_OPCODE,            (kernel_msg_func_t) hci_le_rd_buff_size_cmd_cmp_evt_handler },
    { HCI_RD_BUFF_SIZE_CMD_OPCODE,               (kernel_msg_func_t) hci_rd_buff_size_cmd_cmp_evt_handler },
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    { HCI_NO_OPERATION_CMD_OPCODE,               (kernel_msg_func_t) hci_no_operation_cmd_cmp_evt_handler },

    { HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE,       (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },

    { HCI_RD_LOCAL_VER_INFO_CMD_OPCODE,          (kernel_msg_func_t) hci_rd_local_ver_info_cmd_cmp_evt_handler },
    { HCI_RD_BD_ADDR_CMD_OPCODE ,                (kernel_msg_func_t) hci_rd_bd_addr_cmd_cmp_evt_handler },

    { HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (kernel_msg_func_t) hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler },
    { HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE,         (kernel_msg_func_t) hci_le_rd_max_data_len_cmd_cmp_evt_handler },

    #if (BLE_2MBPS)
    { HCI_LE_SET_DFT_PHY_CMD_OPCODE,             (kernel_msg_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    #endif // (BLE_2MBPS)

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_RD_WLST_SIZE_CMD_OPCODE,            (kernel_msg_func_t) hci_le_rd_wlst_size_cmd_cmp_evt_handler },
    #endif //(BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE,         (kernel_msg_func_t) hci_basic_cmd_cmp_evt_wl_handler },
    { HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE,       (kernel_msg_func_t) hci_basic_cmd_cmp_evt_wl_handler },
    { HCI_LE_CLEAR_WLST_CMD_OPCODE,              (kernel_msg_func_t) hci_basic_cmd_cmp_evt_wl_handler },

    /* Advertising procedure */
    #if (BLE_PERIPHERAL || BLE_BROADCASTER)
    { HCI_LE_SET_ADV_PARAM_CMD_OPCODE,           (kernel_msg_func_t) hci_basic_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_ADV_DATA_CMD_OPCODE,            (kernel_msg_func_t) hci_basic_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE,       (kernel_msg_func_t) hci_basic_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_ADV_EN_CMD_OPCODE,              (kernel_msg_func_t) hci_basic_cmd_cmp_evt_adv_handler },
    #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

    /* Scan procedure */
    #if (BLE_CENTRAL || BLE_OBSERVER)
    { HCI_LE_SET_SCAN_PARAM_CMD_OPCODE,          (kernel_msg_func_t) hci_basic_cmd_cmp_evt_scan_handler },
    { HCI_LE_SET_SCAN_EN_CMD_OPCODE,             (kernel_msg_func_t) hci_basic_cmd_cmp_evt_scan_handler },
    #endif // (BLE_CENTRAL || BLE_OBSERVER)

    /* Connection procedure */
    #if (BLE_CENTRAL)
    { HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE,       (kernel_msg_func_t) hci_basic_cmd_cmp_evt_connect_handler },
    #endif // (BLE_CENTRAL)

    /* Address Management */
    { HCI_LE_SET_RAND_ADDR_CMD_OPCODE,           (kernel_msg_func_t) hci_basic_cmd_cmp_evt_addr_set_handler },

    #if ((BLE_OBSERVER) || (BLE_PERIPHERAL))
    /* retrieve adv tx power level value */
    { HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE,       (kernel_msg_func_t) hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt_handler },
    #endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))

    {HCI_LE_ENC_CMD_OPCODE,                      (kernel_msg_func_t)hci_le_enc_cmd_cmp_evt_handler},
    {HCI_LE_RAND_CMD_OPCODE,                     (kernel_msg_func_t)hci_le_rand_cmd_cmp_evt_handler},

    /* Resolving List Management */
    { HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE,       (kernel_msg_func_t) hci_le_rd_ral_size_cmd_cmp_evt_handler },
    { HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE,       (kernel_msg_func_t) hci_le_rd_ral_addr_cmd_cmp_evt_handler },
    { HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE,        (kernel_msg_func_t) hci_le_rd_ral_addr_cmd_cmp_evt_handler },
    { HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE,       (kernel_msg_func_t) hci_basic_cmd_cmp_evt_rl_handler },
    { HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE,   (kernel_msg_func_t) hci_basic_cmd_cmp_evt_rl_handler },
    { HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE,    (kernel_msg_func_t) hci_basic_cmd_cmp_evt_rl_handler },
    { HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE,  (kernel_msg_func_t) hci_basic_cmd_cmp_evt_rl_handler },
    { HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE,         (kernel_msg_func_t) hci_basic_cmd_cmp_evt_rl_handler },
};

#if (BLE_CENTRAL)
/// The message handlers for HCI status event
static const struct kernel_msg_handler  gapm_hci_cmd_stat_event_handler_tab[] =
{
    { HCI_LE_CREATE_CON_CMD_OPCODE,              (kernel_msg_func_t) hci_basic_cmd_cmp_evt_connect_handler },
};
#endif // (BLE_CENTRAL)

#if (BLE_CENTRAL || BLE_OBSERVER || BLE_PERIPHERAL)
/// The message handlers for HCI LE events
static const struct kernel_msg_handler gapm_hci_le_event_handler_tab[] =
{
    #if (BLE_CENTRAL || BLE_OBSERVER)
    { HCI_LE_ADV_REPORT_EVT_SUBCODE,    (kernel_msg_func_t) hci_le_adv_report_evt_handler },
    { HCI_LE_DIR_ADV_REP_EVT_SUBCODE,   (kernel_msg_func_t) hci_le_dir_adv_report_evt_handler },
    #endif //(BLE_CENTRAL || BLE_OBSERVER)
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_CON_CMP_EVT_SUBCODE,           (kernel_msg_func_t) hci_le_con_cmp_evt_handler },
    { HCI_LE_ENH_CON_CMP_EVT_SUBCODE,       (kernel_msg_func_t) hci_le_enh_con_cmp_evt_handler },
    #if (SECURE_CONNECTIONS)
    { HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE,   (kernel_msg_func_t)hci_le_rd_local_p256_public_key_cmp_evt_handler },
    { HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE,             (kernel_msg_func_t)hci_le_generate_dhkey_cmp_evt_handler },// hci_generate_dhkey_cmd_cmp_evt_handler },
    #endif //  (SECURE_CONNECTIONS)
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
};
#endif // (BLE_CENTRAL || BLE_OBSERVER || BLE_PERIPHERAL)



/**
 ****************************************************************************************
 * @brief Handles any HCI event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_hci_handler(kernel_msg_id_t const msgid, void const* event, kernel_task_id_t dest_id, kernel_task_id_t src_id)
{
    int return_status = KERNEL_MSG_CONSUMED;

    const struct kernel_msg_handler* handler_tab = NULL;
    uint32_t tab_size = 0;

    switch(msgid)
    {
        case HCI_CMD_CMP_EVENT:
        {
            handler_tab = gapm_hci_cmd_cmp_event_handler_tab;
            tab_size    = sizeof(gapm_hci_cmd_cmp_event_handler_tab)/sizeof(gapm_hci_cmd_cmp_event_handler_tab[0]);
        }break;

        #if (BLE_CENTRAL)
        case HCI_CMD_STAT_EVENT:
        {
            handler_tab = gapm_hci_cmd_stat_event_handler_tab;
            tab_size    = sizeof(gapm_hci_cmd_stat_event_handler_tab)/sizeof(gapm_hci_cmd_stat_event_handler_tab[0]);
        }break;
        #endif  //(BLE_CENTRAL)

        #if (BLE_CENTRAL || BLE_OBSERVER || BLE_PERIPHERAL)
        case HCI_LE_EVENT:
        {
            handler_tab = gapm_hci_le_event_handler_tab;
            tab_size    = sizeof(gapm_hci_le_event_handler_tab)/sizeof(gapm_hci_le_event_handler_tab[0]);
            // Get subcode at from message parameters (1st byte position)
            src_id      = *((uint8_t*)event);
        }break;
        #endif // (BLE_CENTRAL || BLE_OBSERVER || BLE_PERIPHERAL)
        default:   /* should not occurs, nothing to do */ break;
    }


    // Check if there is a handler corresponding to the original command opcode
    int i = 0;
    for( i = 0; i < tab_size; i++)
    {
        // Check if opcode matches
        if(handler_tab[i].id == src_id)
        {
            // Check if there is a handler function
            if(handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = handler_tab[i].func(src_id, event, dest_id, src_id);
            }
            break;
        }
    }

    return return_status;
}

/// @} GAPM_HCI
