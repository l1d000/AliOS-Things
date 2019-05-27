/**
 ****************************************************************************************
 *
 * @file gapm_task.c
 *
 * @brief Generic Access Profile Manager Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_TASK Generic Access Profile Manager Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwble_hl.h"

#include <string.h>

#include "gap.h"

#include "gattm_task.h"
#include "gapm_task.h"

#include "common_bt.h"
#include "common_error.h"
#include "common_math.h"
#include "common_version.h"
#include "kernel_mem.h"
#include "gapm_util.h"
#include "gapm_int.h"

#include "gapc.h"

#include "gattm.h"
#include "gattc_task.h"

#include "attm.h"

#include "smpm_api.h" // Access to internal API Required

#include "hci.h"

#if (BLE_PROFILES)
#include "prf.h"
#include "RomCallFlash.h"
#endif // (BLE_PROFILES)
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)

#include "kernel_timer.h"
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */


// GAP database default features
#define GAP_DB_DEFAULT_FEAT         0x001F
// GAP database features in peripheral role
#define GAP_DB_PERIPH_FEAT          0x0060
// GAP database features in central role
#define GAP_DB_CENTRAL_FEAT         0x0180

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


/**
 ****************************************************************************************
 * @brief Check if current operation can be processed or not.
 * if it can be proceed, initialize an operation request.
 * If a command complete event with error code can be triggered.
 *
 * Function returns how the message should be handled by message handler.
 *
 * @param[in] op_type       Operation type.
 * @param[in] op_msg        Requested operation message (note op_msg cannot be null)
 * @param[in] supp_ops      Supported operations array.
 *                          Latest array value shall be GAPM_NO_OP.
 *
 * @return operation can be executed if message status equals KERNEL_MSG_NO_FREE,
 * else nothing to do, just exit from the handler.
 ****************************************************************************************
 */
static int gapm_process_op(uint8_t op_type, void* op_msg, enum gapm_operation* supp_ops)
{
    ASSERT_ERR(op_type < GAPM_OP_MAX);
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED; // Reset State
    // Current process state
    uint8_t state = kernel_state_get(TASK_GAPM);
    uint8_t operation = *((uint8_t*)op_msg);

    /* no operation on going or requested operation is current on going operation. */
    if(state != GAPM_DEVICE_SETUP)
    {
        if(gapm_get_operation_ptr(op_type) != op_msg)
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // check what to do with command if an operation is ongoing.
            if((state & (1 << op_type)) != GAPM_IDLE)
            {
                // only one air operation accepted
                if(op_type == GAPM_OP_AIR)
                {
                    // request disallowed
                    status = GAP_ERR_COMMAND_DISALLOWED;
                }
                else
                {
                    // operation are queued by default
                    // save it for later.
                    msg_status = KERNEL_MSG_SAVED;
                }
            }
            else
            {
                // check if operation is suported
                while(*supp_ops != GAPM_NO_OP)
                {
                    // operation supported by command
                    if(operation == *supp_ops)
                    {
                        break;
                    }
                    // check next operation
                    else
                    {
                        supp_ops++;
                    }
                }

                // operation not supported
                if(*supp_ops == GAPM_NO_OP)
                {
                    status = GAP_ERR_INVALID_PARAM;
                }
                else
                {
                    // message memory will be managed by GAPM
                    msg_status = KERNEL_MSG_NO_FREE;

                    // store operation
                    gapm_set_operation_ptr(op_type, op_msg);
                    // set state to busy
                    gapm_update_state(op_type, true);
                }
            }

            // if an error detected, send command completed with error status
            if(status != GAP_ERR_NO_ERROR)
            {
                gapm_send_error_evt(operation, kernel_msg_src_id_get(op_msg), status);
            }
        }
        else
        {
            // message memory managed by GAPM
            msg_status = KERNEL_MSG_NO_FREE;
        }
    }

    return msg_status;
}


/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles request of host reset + Initialization of lower layers.
 *  - GAPM_RESET: software reset operation.
 *  - GAPM_PLF_RESET: Platform reset
 *
 *  Procedure:
 *   1. HCI_RESET_CMD
 *   2. HCI_SET_EVT_MASK_CMD
 *   3. HCI_LE_SET_EVT_MASK_CMD
 *   4. HCI_RD_BD_ADDR_CMD
 *   5. HCI_LE_RD_BUFF_SIZE_CMD
 *   6. HCI_RD_BUFF_SIZE_CMD
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_reset_cmd_handler(kernel_msg_id_t const msgid, struct gapm_reset_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED; // Reset State

    // check operation is a reset request
    if(param->operation == GAPM_RESET)
    {
        // perform a reset of higher layers.
        rwble_hl_reset();
				// rom_env.rwble_hl_reset();

        // Initialize operation execution.
        gapm_set_operation_ptr(GAPM_OP_CFG, param);

        msg_status = KERNEL_MSG_NO_FREE;

        // initialize reset operation
        gapm_op_reset_continue(GAPM_OP_RESET_INIT, GAP_ERR_NO_ERROR);
    }
    else if (param->operation == GAPM_PLF_RESET)
    {
        // Reset the platform
        platform_reset(RESET_AND_LOAD_FW);
			//	rom_env.platform_reset(RESET_AND_LOAD_FW);
    }
    else
    {
        // Invalid parameters set.
        gapm_send_error_evt(param->operation, src_id, GAP_ERR_INVALID_PARAM);
    }

    return msg_status;
}



/**
 ****************************************************************************************
 * @brief  Handles request of host to cancel ongoing air operation.
 *  - GAPM_CANCEL: cancel air operation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_cancel_cmd_handler(kernel_msg_id_t const msgid, struct gapm_cancel_cmd const *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t op_type = GAPM_OP_CFG;
    uint8_t operation;

    switch(param->operation)
    {
        // Cancel ongoing air operation
        case GAPM_CANCEL:
        {
            op_type = GAPM_OP_AIR;
        }
        break;
        default:
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        break;
    }

    // retrieve operation code.
    operation = gapm_get_operation(op_type);

    // check if there is no ongoing operation or if command operation is invalid
    if((status != GAP_ERR_NO_ERROR) || (operation == GAPM_NO_OP))
    {
        // no air command to cancel
        gapm_send_error_evt(param->operation, src_id, ((status != GAP_ERR_NO_ERROR)
                ? status
                : GAP_ERR_COMMAND_DISALLOWED));
    }
    else
    {
        // update state according to current state
        gapm_update_air_op_state(op_type, GAPM_OP_CANCEL, GAP_ERR_NO_ERROR);
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles modification of device GAP configuration such as role, security
 * parameters, etc:
 *  - GAPM_SET_DEV_CONFIG: Set device configuration
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_set_dev_config_cmd_handler(kernel_msg_id_t const msgid, struct gapm_set_dev_config_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_DEV_CONFIG, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // no need to check GAPM_SET_DEV_CONFIG operation, this has been done by gapm_process_op
        uint8_t status = GAP_ERR_NO_ERROR;
        uint8_t authorized_role = GAP_ROLE_NONE;

        do
        {
            uint8_t role = param->role & GAP_ROLE_ALL;

            /* Configuration can be modified only if no connection exists and no air
             * operation on-going.
             */
            if((gapm_env.connections != 0)
                    || (gapm_get_operation_ptr(GAPM_OP_AIR) != NULL) // No air operation
                )
            {
                /* send command complete event with error */
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            // create a variable used to check if role is supported or not
            #if (BLE_BROADCASTER)
            authorized_role |= GAP_ROLE_BROADCASTER;
            #endif // (BLE_BROADCASTER)

            #if (BLE_OBSERVER)
            authorized_role |= GAP_ROLE_OBSERVER;
            #endif // (BLE_OBSERVER)

            #if (BLE_CENTRAL)
            authorized_role |= GAP_ROLE_CENTRAL;
            #endif // (BLE_CENTRAL)

            #if (BLE_PERIPHERAL)
            authorized_role |= GAP_ROLE_PERIPHERAL;
            #endif // (BLE_PERIPHERAL)

            // Check that configured role is in authorized role
            if((role| authorized_role) != authorized_role)
            {
                /* Role which is configured is not supported by the Host stack */
                status = GAP_ERR_NOT_SUPPORTED;
                break;
            }

            // parameter sanity check
            if(gapm_addr_check(param->addr_type))
            {
                status = GAP_ERR_INVALID_PARAM;
                break;
            }

            // Data Length sanity check
            if(gapm_dle_val_check(param->sugg_max_tx_octets, param->sugg_max_tx_time))
            {
                status = GAP_ERR_INVALID_PARAM;
                break;
            }

            // clear role and config
            gapm_env.cfg_flags = 0;
            gapm_env.role = GAP_ROLE_NONE;
            kernel_timer_clear(GAPM_ADDR_RENEW_TO_IND , TASK_GAPM);

            #if (BLE_ATTS)
            {
                /* First Clear all the database */
                attmdb_destroy();

                // database available only for central and peripheral roles.
                if(((role & GAP_ROLE_PERIPHERAL) == GAP_ROLE_PERIPHERAL)
                        ||((role & GAP_ROLE_CENTRAL) == GAP_ROLE_CENTRAL))
                {
                    // define gap database features
                    uint32_t db_cfg = GAP_DB_DEFAULT_FEAT;

                    if(GAPM_F_GET(param->att_cfg, ATT_SLV_PREF_CON_PAR_EN))
                    {
                        db_cfg |= GAP_DB_PERIPH_FEAT;
                        GAPM_F_SET(gapm_env.cfg_flags, PREF_CON_PAR_PRES, 1);
                    }

                    if(param->addr_type & GAPM_CFG_ADDR_CTNL_PRIVACY)
                    {
                        db_cfg |= GAP_DB_CENTRAL_FEAT;
                    }

                    #if (BLE_DEBUG)
                    if(GAPM_F_GET(param->att_cfg, ATT_DBG_MODE_EN))
                    {
                        GAPM_F_SET(gapm_env.cfg_flags, DBG_MODE_EN, 1);
                    }
                    #endif // (BLE_DEBUG)

                    /* Create GAP Attribute Database */
                    status = gapm_init_attr(param->gap_start_hdl, db_cfg);
                    if(status != GAP_ERR_NO_ERROR)
                    {
                        break;
                    }

                    // define gatt database features
                    if(GAPM_F_GET(param->att_cfg, ATT_SVC_CHG_EN))
                    {
                        GAPM_F_SET(gapm_env.cfg_flags, SVC_CHG_EN, 1);
                    }

                    /* Create GATT Attribute Database */
                    status = gattm_init_attr(param->gatt_start_hdl, GAPM_F_GET(param->att_cfg, ATT_SVC_CHG_EN));
                    if(status != GAP_ERR_NO_ERROR)
                    {
                        break;
                    }
                }

                // Set appearance characteristic permissions
                if(GAPM_F_GET(param->att_cfg, ATT_APPEARENCE_PERM) != GAPM_WRITE_DISABLE)
                {
                    // Set appearance write permission
                    ATTMDB_UPDATE_PERM_VAL(gapm_get_att_handle(GAP_IDX_ICON), WP, (GAPM_F_GET(param->att_cfg, ATT_APPEARENCE_PERM) - 1));
                    ATTMDB_UPDATE_PERM(gapm_get_att_handle(GAP_IDX_ICON), WRITE_REQ, ENABLE);
                }

                // Set device name characteristic permissions
                if(GAPM_F_GET(param->att_cfg, ATT_NAME_PERM) != GAPM_WRITE_DISABLE)
                {
                    // Set device name write permission
                    ATTMDB_UPDATE_PERM_VAL(gapm_get_att_handle(GAP_IDX_DEVNAME), WP, (GAPM_F_GET(param->att_cfg, ATT_NAME_PERM) - 1));
                    ATTMDB_UPDATE_PERM(gapm_get_att_handle(GAP_IDX_DEVNAME), WRITE_REQ, ENABLE);
                }
            }
            #endif //(BLE_ATTS)

            #if (BLE_PROFILES)
            /* Then remove all profile tasks */
            prf_init(true);
					//	rom_env.prf_init(true);
            #endif // (BLE_PROFILES)

            // set role
            gapm_env.role = role;

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            // Set maximal MTU
            gapm_set_max_mtu(param->max_mtu);
            // Set maximal MPS
            gapm_set_max_mps(param->max_mps);

            #if (BLE_LECB)
            // remove all registered LE_PSM
            gapm_le_psm_cleanup();
            gapm_env.nb_lecb     = 0;
            gapm_env.max_nb_lecb = param->max_nb_lecb;
            #endif // (BLE_LECB)

            // security configuration only needed if device is peripheral or central
            if((gapm_env.role & ~(GAP_ROLE_OBSERVER | GAP_ROLE_BROADCASTER)) != 0)
            {
                // Store pairing mode
                gapm_env.pairing_mode = param->pairing_mode;

                #if (SECURE_CONNECTIONS)
                // check if secure connection is allowed or not
                if((gapm_env.pairing_mode & GAPM_PAIRING_SEC_CON) != 0)
                {
                    // Try to load p256 Public Key
                    if((gapm_env.pairing_mode & GAPM_PAIRING_FORCE_P256_KEY_GEN) == 0)
                    {
                        #if (NVDS_SUPPORT)
                        uint8_t p256_pk_array[64];
                        uint8_t pk_len = NVDS_LEN_LE_PUBLIC_KEY_P256;

                        if (nvds_get(NVDS_TAG_LE_PUBLIC_KEY_P256, &pk_len, (uint8_t *)&p256_pk_array) == NVDS_OK)
                        {
                            // Copy the Public Key read from NVRAM to GAP.
                            memcpy(&gapm_env.public_key.x[0],&p256_pk_array[0],32);
                            memcpy(&gapm_env.public_key.y[0],&p256_pk_array[32],32);
                        }
                        else
                        #endif // (NVDS_SUPPORT)
                        {
                            // Mark that p256 public key has to be generated
                            gapm_env.pairing_mode |= GAPM_PAIRING_FORCE_P256_KEY_GEN;
                        }
                    }
                }
                else
                #endif // (SECURE_CONNECTIONS)
                {
                    // ensure that p256 public key will not be generated
                    gapm_env.pairing_mode &= ~GAPM_PAIRING_FORCE_P256_KEY_GEN;
                    gapm_env.pairing_mode &= ~GAPM_PAIRING_SEC_CON;
                }
            }
            else
            #endif //(BLE_CENTRAL || BLE_PERIPHERAL)
            {
                gapm_env.pairing_mode = 0;
            }

            // Set device Identity key (IRK)
            memcpy(&(gapm_env.irk), &(param->irk), sizeof(struct gap_sec_key));

            #if(BLE_AUDIO)
            // copy audio configuration
            gapm_env.audio_cfg = param->audio_cfg;
            #endif // (BLE_AUDIO)
        } while (0);

        if(status == GAP_ERR_NO_ERROR)
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_INIT, GAP_ERR_NO_ERROR);
        }
        else
        {
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of modifying local channel map:
 *  - GAPM_SET_CHANNEL_MAP:  Set device channel map
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_set_channel_map_cmd_handler(kernel_msg_id_t const msgid, struct gapm_set_channel_map_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_CHANNEL_MAP, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // no need to check GAPM_SET_CHANNEL_MAP operation, this has been done by gapm_process_op

        // this is only for central role
         if ((gapm_get_role() & GAP_ROLE_CENTRAL) == GAP_ROLE_CENTRAL)
         {
             // allocate set host channel classification message
             struct hci_le_set_host_ch_class_cmd *ch_class = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE, hci_le_set_host_ch_class_cmd);

             // update channel map value
             memcpy(&ch_class->chmap.map[0], &param->chmap.map[0], LE_CHNL_MAP_LEN);

             // message send */
             hci_send_2_controller(ch_class);
         }
         else
         {
             // send command complete event with error
             gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_COMMAND_DISALLOWED);
         }
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of getting information about local device such as:
 * - GAPM_GET_DEV_NAME: Get Local device name indication event
 * - GAPM_GET_DEV_VERSION: Get Local device version indication event
 * - GAPM_GET_DEV_BDADDR: Get Local device BD Address indication event
 * - GAPM_GET_DEV_ADV_TX_POWER: Get device advertising power level
 * - GAPM_DBG_GET_MEM_INFO: Get memory usage (debug only)
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_get_dev_info_cmd_handler(kernel_msg_id_t const msgid, struct gapm_get_dev_info_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_GET_DEV_VERSION,
                                       GAPM_GET_DEV_BDADDR, GAPM_GET_DEV_ADV_TX_POWER,
                                       GAPM_DBG_GET_MEM_INFO,
                                       GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN,
                                       GAPM_GET_MAX_LE_DATA_LEN,
                                       GAPM_NO_OP };
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {

        // check operation
        switch(param->operation)
        {
            // Get Local device version
            case GAPM_GET_DEV_VERSION:
            {
                /* Get the device local version */
                gapm_basic_hci_cmd_send(HCI_RD_LOCAL_VER_INFO_CMD_OPCODE);
            }
            break;
            // Get Local device BD Address
            case GAPM_GET_DEV_BDADDR:
            {
                struct gapm_dev_bdaddr_ind *bdaddr_ind =
                        KERNEL_MSG_ALLOC(GAPM_DEV_BDADDR_IND, src_id, dest_id, gapm_dev_bdaddr_ind);
                /* fill up the parameters */
                memcpy(&(bdaddr_ind->addr.addr), &(gapm_env.addr), BD_ADDR_LEN);
                bdaddr_ind->addr.addr_type = ADDR_PUBLIC;

                /* send the message indication */
                kernel_msg_send(bdaddr_ind);

                /* send command complete event with error */
                gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
            }
            break;
            // Get device advertising Tx Power level
            case GAPM_GET_DEV_ADV_TX_POWER:
            {
                #if ((BLE_OBSERVER) || (BLE_PERIPHERAL))
                /* send read adv tx power level */
                gapm_basic_hci_cmd_send(HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE);
                #else // ((BLE_OBSERVER) || (BLE_PERIPHERAL))
                /* send command complete event with error */
                gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NOT_SUPPORTED);
                #endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))
            }
            break;
            // Get device memory usage
            case GAPM_DBG_GET_MEM_INFO:
            {
                #if (KERNEL_PROFILING)
                uint8_t cursor;
                struct gapm_dbg_mem_info_ind meminfo;
                struct gapm_dbg_mem_info_ind * meminfo_msg;

                // First remove command message in order to be sure it's not taken in account.
                kernel_msg_free(kernel_param2msg(param));
                gapm_set_operation_ptr(GAPM_OP_CFG, NULL);

                // Then retrieve memory information from kernel
                meminfo.max_mem_used = kernel_get_max_mem_usage();
                for(cursor = 0; cursor < KERNEL_MEM_BLOCK_MAX ; cursor++)
                {
                    meminfo.mem_used[cursor] = kernel_get_mem_usage(cursor);
                }

                // Finally send indication to application that request memory information
                meminfo_msg = KERNEL_MSG_ALLOC(GAPM_DBG_MEM_INFO_IND, src_id, dest_id, gapm_dbg_mem_info_ind);
                memcpy(meminfo_msg, &meminfo, sizeof(struct gapm_dbg_mem_info_ind));
                kernel_msg_send(meminfo_msg);

                /* send command complete event with no error */
                gapm_send_error_evt(GAPM_DBG_GET_MEM_INFO, src_id, GAP_ERR_NO_ERROR);
                // restore GAPM state to idle
                gapm_update_state(GAPM_OP_CFG, false);

                #else
                /* send command complete event with error */
                gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NOT_SUPPORTED);
                #endif /* (KERNEL_PROFILING) */
            }
            break;
            // Get Suggested Default Data Length
            case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN:
            {
                // Allocate message
                struct gapm_operation_cmd *get_sugg_data = KERNEL_MSG_ALLOC(HCI_COMMAND, 0,
                        HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, gapm_operation_cmd);

                get_sugg_data->operation = GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN;
                /* send the message */
                hci_send_2_controller(get_sugg_data);
            }
            break;
            // Get Maximum LE Data Length
            case GAPM_GET_MAX_LE_DATA_LEN:
            {
                /* send the message */
                gapm_basic_hci_cmd_send(HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE);
            }
            break;
            default:
            {
                // no need to check operation, this has been done by gapm_process_op
                /* nothing to do */
            }
            break;
        };
    }

    return msg_status;
}





/**
 ****************************************************************************************
 * @brief Handles management of white list command.
 *
 * This command can be used to :
 *  - GAPM_GET_WLIST_SIZE:    Get White List Size.
 *  - GAPM_ADD_DEV_IN_WLIST:  Add devices in white list.
 *  - GAPM_RMV_DEV_FRM_WLIST: Remove devices form white list.
 *  - GAPM_CLEAR_WLIST:       Clear all devices from white list.
 *   *
 * This command can be used at any time except when:
 *  1. The advertising filter policy uses the white list and advertising is enabled.
 *  2. The scanning filter policy uses the white list and scanning is enabled.
 *  3. The initiator filter policy uses the white list and a create connection command
 *     is outstanding.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_white_list_mgt_cmd_handler(kernel_msg_id_t const msgid,
        struct gapm_white_list_mgt_cmd *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GET_WLIST_SIZE, GAPM_ADD_DEV_IN_WLIST,
                                      GAPM_RMV_DEV_FRM_WLIST, GAPM_CLEAR_WLIST,
                                      GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        switch(param->operation)
        {
            // Get White List Size.
            case GAPM_GET_WLIST_SIZE:
            {
                gapm_basic_hci_cmd_send(HCI_LE_RD_WLST_SIZE_CMD_OPCODE);
            }
            break;
            // Add devices in white list.
            case GAPM_ADD_DEV_IN_WLIST:
            {
                // Still some devices to add in white list.
                if(param->nb > 0)
                {
                    /* add specific address in the list */
                    struct hci_le_add_dev_to_wlst_cmd *add_dev = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE, hci_le_add_dev_to_wlst_cmd);

                    /* fill up the parameters */
                    memcpy(&(add_dev->dev_addr), &(param->devices[param->nb-1].addr), BD_ADDR_LEN);
                    add_dev->dev_addr_type = param->devices[param->nb-1].addr_type;

                    /* send the message */
                    hci_send_2_controller(add_dev);

                }
                // operation is finished
                else
                {
                    /* send command complete succeed */
                    gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
                }
            }
            break;
            // Remove devices form white list.
            case GAPM_RMV_DEV_FRM_WLIST:
            {
                // Still some devices to add in white list.
                if(param->nb > 0)
                {
                    /* remove specific address in the list */
                    struct hci_le_rmv_dev_from_wlst_cmd *rem_dev = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE, hci_le_rmv_dev_from_wlst_cmd);

                    /* fill up the parameters */
                    memcpy(&(rem_dev->dev_addr), &(param->devices[param->nb-1].addr), BD_ADDR_LEN);
                    rem_dev->dev_addr_type = param->devices[param->nb-1].addr_type;

                    /* send the message */
                    hci_send_2_controller(rem_dev);

                }
                // operation is finished
                else
                {
                    /* send command complete succeed */
                    gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
                }
            }
            break;
            // Clear all devices from white list.
            case GAPM_CLEAR_WLIST:
            {
                // clear white list
                gapm_basic_hci_cmd_send(HCI_LE_CLEAR_WLST_CMD_OPCODE);
            }
            break;
            default:
            {
                /* Nothing to do since operation has been checked by gapm_process_op */
            }
            break;
        };
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of solving a resolvable random address.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_resolv_addr_cmd_handler(kernel_msg_id_t const msgid, struct gapm_resolv_addr_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_RESOLV_ADDR, GAPM_NO_OP};
    // use to know if it's first operation loop
    bool first_loop = (gapm_get_operation_ptr(GAPM_OP_CFG) == NULL);
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // no need to check GAPM_RESOLV_ADDR operation, this has been done by gapm_process_op

        uint8_t status = GAP_ERR_NO_ERROR;

        // Perform sanity check when receiving command.
        if(first_loop)
        {
            // check if key are provided and if address is resolvable
            if((param->nb_key == 0)
                    || ((param->addr.addr[BD_ADDR_LEN-1] & 0xC0) != GAP_RSLV_ADDR))
            {
                /* Invalid parameter */
                status = GAP_ERR_INVALID_PARAM;
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Not able to solve random address.
            if(param->nb_key == 0)
            {
                status = GAP_ERR_NOT_FOUND;
            }
            else
            {
                // Check with latest key.
                param->nb_key--;
                // request to resolve first device address
                smpm_resolv_addr( &(param->addr), &(param->irk[param->nb_key]));
            }
        }
        if(status != GAP_ERR_NO_ERROR)
        {
            // send command completed with provided status code.
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }

    return msg_status;
}

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handles request of starting advertising command:
 *
 *  - GAPM_ADV_NON_CONN: Start non connectable advertising
 *  - GAPM_ADV_UNDIRECT: Start undirected connectable advertising
 *  - GAPM_ADV_DIRECT: Start directed connectable advertising
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_start_advertise_cmd_handler(kernel_msg_id_t const msgid, struct gapm_start_advertise_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_ADV_NON_CONN,GAPM_ADV_UNDIRECT,
            GAPM_ADV_DIRECT, GAPM_ADV_DIRECT_LDC, GAPM_NO_OP};
    // use to know if it's first operation loop
    bool first_loop = (gapm_get_operation_ptr(GAPM_OP_AIR) == NULL);
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // execution status
        uint8_t status = GAP_ERR_NO_ERROR;

        /* if destination and source id are different it means that operation
         * execution starts
         */
        if(first_loop)
        {
            // operation initial state
            param->op.state = 0;

            // perform a sanity check
            status = gapm_adv_op_sanity();

            if(status == GAP_ERR_NO_ERROR)
            {
                // initialize state machine
                gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_INIT, GAP_ERR_NO_ERROR);
								
            }
            else
            {
                // stop operation execution
                gapm_send_complete_evt(GAPM_OP_AIR, status);
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Execute the operation
            gapm_execute_adv_op();
        }
    }

    return msg_status;
}
/**
 ****************************************************************************************
 * @brief Handles request of starting advertising command:
 *
 *  - GAPM_UPDATE_ADVERTISE_DATA: Update on the fly advertising data
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_update_advertise_data_cmd_handler(kernel_msg_id_t const msgid, struct gapm_update_advertise_data_cmd *param,
                                                  kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_UPDATE_ADVERTISE_DATA, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        switch(gapm_get_operation(GAPM_OP_AIR))
        {
            // Non connected or undirect advertising
            case GAPM_ADV_NON_CONN:
            case GAPM_ADV_UNDIRECT:
            {
                // retrieve advertising parameters
                struct gapm_start_advertise_cmd *adv =
                        (struct gapm_start_advertise_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

                // execute current operation state.
                if(GAPM_GET_OP_STATE(adv->op) > GAPM_OP_SET_SCAN_RSP_DATA)
                {
                    // Set advertising data
                    gapm_set_adv_data(param->adv_data_len, param->adv_data);
                    break;
                }
            }
            // no break

            default:
            {
                // stop operation execution
                gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_COMMAND_DISALLOWED);
            }
            break;
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles limited discoverable timeout indication
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_lim_disc_to_ind_handler(kernel_msg_id_t const msgid, void const *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        // update state according to current state
        gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_TIMEOUT, GAP_ERR_NO_ERROR);
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif /* #if (BLE_PERIPHERAL || BLE_BROADCASTER) */




#if (BLE_CENTRAL || BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Handles request of starting scan operation
 *  - GAPM_SCAN_ACTIVE: Start active scan operation
 *  - GAPM_SCAN_PASSIVE: Start passive scan operation
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_start_scan_cmd_handler(kernel_msg_id_t const msgid, struct gapm_start_scan_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_SCAN_ACTIVE, GAPM_SCAN_PASSIVE, GAPM_NO_OP};
    // use to know if it's first operation loop
    bool first_loop = (gapm_get_operation_ptr(GAPM_OP_AIR) == NULL);
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // execution status
        uint8_t status = GAP_ERR_NO_ERROR;
        /* if destination and source id are different it means that operation
         * execution starts
         */
        if(first_loop)
        {
            // operation initial state
            param->op.state = 0;

            // perform a sanity check
            status = gapm_scan_op_sanity();

            if(status == GAP_ERR_NO_ERROR)
            {
                // initialize state machine
                gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_INIT, GAP_ERR_NO_ERROR);
            }
            else
            {
                // stop operation execution
                gapm_send_complete_evt(GAPM_OP_AIR, status);
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Execute the operation
            gapm_execute_scan_op();
        }
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles timeout of scan procedure
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_scan_to_ind_handler(kernel_msg_id_t const msgid, void const *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
        // update state according to current state
        gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_TIMEOUT, GAP_ERR_NO_ERROR);
    }

    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_CENTRAL || BLE_OBSERVER)


#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles request of starting connection establishment command:
 *  - GAPM_CONNECTION_DIRECT: Direct connection operation
 *  - GAPM_CONNECTION_AUTO: Automatic connection operation
 *  - GAPM_CONNECTION_SELECTIVE: Selective connection operation
 *  - GAPM_CONNECTION_NAME_REQUEST: Name Request operation (requires to start a direct connection)
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_start_connection_cmd_handler(kernel_msg_id_t const msgid, struct gapm_start_connection_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_CONNECTION_DIRECT, GAPM_CONNECTION_AUTO,
            GAPM_CONNECTION_SELECTIVE, GAPM_CONNECTION_NAME_REQUEST, GAPM_CONNECTION_GENERAL, GAPM_NO_OP};
    // use to know if it's first operation loop
    bool first_loop = (gapm_get_operation_ptr(GAPM_OP_AIR) == NULL);
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // execution status
        uint8_t status = GAP_ERR_NO_ERROR;
        /* if destination and source id are different it means that operation
         * execution starts
         */
        if(first_loop)
        {
            // operation initial state
            param->op.state = 0;

            // perform a sanity check
            status = gapm_connect_op_sanity();

            if(status == GAP_ERR_NO_ERROR)
            {
                // initialize state machine
                gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_INIT, GAP_ERR_NO_ERROR);
            }
            else
            {
                // stop operation execution
                gapm_send_complete_evt(GAPM_OP_AIR, status);
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Execute the operation
            gapm_execute_connect_op();
        }
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles Connection confirmation message to start selective connection to
 * specified device
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_connection_cfm_handler(kernel_msg_id_t const msgid, struct gapm_connection_cfm const *cfm,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    uint8_t operation = gapm_get_operation(GAPM_OP_AIR);

    if((state != GAPM_DEVICE_SETUP) && (   (operation == GAPM_CONNECTION_SELECTIVE)
                                        || (operation == GAPM_CONNECTION_GENERAL)))
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        struct gapm_start_connection_cmd* connect_cmd =
                (struct gapm_start_connection_cmd*) gapm_get_operation_ptr(GAPM_OP_AIR);

        if(operation == GAPM_CONNECTION_SELECTIVE)
        {
            int8_t cursor;

            for(cursor = connect_cmd->nb_peers - 1 ; cursor >= 0 ; cursor--)
            {
                // Force value in controller based privacy
                connect_cmd->peers[cursor].addr_type =
                        (gapm_get_address_type() & GAPM_CFG_ADDR_CTNL_PRIVACY) ?
                    ADDR_RPA_OR_PUBLIC : cfm->addr_type;

                if((connect_cmd->peers[cursor].addr_type == cfm->addr_type)
                        && (memcmp(&(connect_cmd->peers[cursor].addr), &(cfm->addr),
                                sizeof(bd_addr_t)) == 0))
                {
                    break;
                }
            }

            // device to connect present in peers array
            if(cursor < 0)
            {
                status = GAP_ERR_INVALID_PARAM;
            }
        }

        if(status == GAP_ERR_NO_ERROR)
        {
            // Copy report address into first peer list element to connect to.
            connect_cmd->peers[0].addr_type = cfm->addr_type;
            memcpy(&(connect_cmd->peers[0].addr), &(cfm->addr),
                    sizeof(bd_addr_t));

            // set selected connection parameters ****
            // Minimum of connection interval
            connect_cmd->con_intv_min = cfm->con_intv_min;
            // Maximum of connection interval
            connect_cmd->con_intv_max = cfm->con_intv_max;
            // Connection latency
            connect_cmd->con_latency  = cfm->con_latency;
            // Link supervision timeout
            connect_cmd->superv_to    = cfm->superv_to;
            // Minimum CE length
            connect_cmd->ce_len_min   = cfm->ce_len_min;
            // Maximum CE length
            connect_cmd->ce_len_max   = cfm->ce_len_max;
        }

        // start connection operation or stop with an error.
        gapm_update_air_op_state(GAPM_OP_AIR,
                ((status == GAP_ERR_NO_ERROR) ? GAPM_OP_CONNECT_REQ : GAPM_OP_ERROR), GAP_ERR_INVALID_PARAM);
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of GAP controller command completed event.
 *  - For Name Request
 *  - Disconnection part of name request operation
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct gapc_cmp_evt const *cmp_evt,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);


    if((state != GAPM_DEVICE_SETUP)
            && (gapm_get_operation(GAPM_OP_AIR) != GAPM_NO_OP))
    {
        uint8_t new_state = GAPM_OP_ERROR;

        // check reason of command complete.
        switch(cmp_evt->operation)
        {
            // Disconnection
            case GAPC_DISCONNECT:
            {
                // update state according to current state
                new_state = ((cmp_evt->status ==  GAP_ERR_NO_ERROR)
                                ? GAPM_OP_DISCONNECT : GAPM_OP_ERROR);
            }
            break;
            // name request
            case GAPC_GET_PEER_NAME:
            {
                // update state according to current state (if name request failed,
                // mark operation as canceled
                new_state =((cmp_evt->status ==  GAP_ERR_NO_ERROR)
                                ? GAPM_OP_NAME_REQ : GAPM_OP_CANCEL);
            }
            break;
            default: /* Nothing to do. */ break;
        }
        gapm_update_air_op_state(GAPM_OP_AIR, new_state, cmp_evt->status);
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of name indication. Convey message to name requester.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_peer_att_info_ind_handler(kernel_msg_id_t const msgid,
        struct gapc_peer_att_info_ind const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if((state != GAPM_DEVICE_SETUP)
            && (gapm_get_operation(GAPM_OP_AIR) == GAPM_CONNECTION_NAME_REQUEST))
    {
        uint8_t conidx = KERNEL_IDX_GET(src_id);

        // create name information indication message.
        struct gapm_peer_name_ind * name_ind =
                KERNEL_MSG_ALLOC_DYN(GAPM_PEER_NAME_IND, gapm_get_requester(GAPM_OP_AIR),
                        dest_id, gapm_peer_name_ind, param->info.name.length);

        // fill parameters
        memcpy(&(name_ind->addr), &(gapc_get_bdaddr(conidx,SMPC_INFO_PEER)->addr),
                sizeof(bd_addr_t));
        name_ind->addr_type = gapc_get_bdaddr(conidx,SMPC_INFO_PEER)->addr_type;
        name_ind->name_len = param->info.name.length;
        memcpy(name_ind->name, param->info.name.value, name_ind->name_len);

        // send indication
        kernel_msg_send(name_ind);
    }

    return (KERNEL_MSG_CONSUMED);
}

#endif // (BLE_CENTRAL)


#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles reception of WL operation completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct gapm_cmp_evt const *cmp_evt,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    // sanity check
    if((state != GAPM_DEVICE_SETUP)
            && (gapm_get_operation(GAPM_OP_AIR) != GAPM_NO_OP))
    {
        uint8_t new_state = GAPM_OP_ERROR;

        // check reason of command complete.
        switch(cmp_evt->operation)
        {
            // Disconnection
            case GAPM_CLEAR_WLIST:
            {
                // update state according to current state
                new_state = ((cmp_evt->status == GAP_ERR_NO_ERROR)
                                ? GAPM_OP_CLEAR_WL : GAPM_OP_ERROR);
            }
            break;
            // name request
            case GAPM_ADD_DEV_IN_WLIST:
            {
                // update state according to current state (if name request failed,
                // mark operation as canceled
                new_state =((cmp_evt->status == GAP_ERR_NO_ERROR)
                                ? GAPM_OP_SET_WL : GAPM_OP_CLEAR_WL);
            }
            break;

            case GAPM_GEN_RAND_ADDR:
            {
                // status of generate random address
                new_state =((cmp_evt->status == GAP_ERR_NO_ERROR)
                                                ? GAPM_OP_ADDR_GEN : GAPM_OP_ERROR);
            }
            break;
            default: /* Nothing to do. */ break;
        }

        gapm_update_air_op_state(GAPM_OP_AIR, new_state, cmp_evt->status);
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handles timeout of renew address timer.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_addr_renew_to_ind_handler(kernel_msg_id_t const msgid, void const *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPM_DEVICE_SETUP)
    {
       // Set Address renew flag
       GAPM_F_SET(gapm_env.cfg_flags, ADDR_RENEW, 1);

       if(gapm_get_operation(GAPM_OP_AIR) != GAPM_NO_OP)
       {
           // update state according to current state
           gapm_update_air_op_state(GAPM_OP_AIR, GAPM_OP_ADDR_RENEW, GAP_ERR_NO_ERROR);
       }
    }

    return (KERNEL_MSG_CONSUMED);
}

#if (BLE_PROFILES)
/**
 ****************************************************************************************
 * @brief Handles request to add a new profile task.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_profile_task_add_cmd_handler(kernel_msg_id_t const msgid,
                                          struct gapm_profile_task_add_cmd * param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_PROFILE_TASK_ADD, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        kernel_task_id_t prf_task;
        uint8_t status;
        #if(AHI_TL_SUPPORT)
        // for external application which AHI, update task Number from task ID
        if(KERNEL_TYPE_GET(param->app_task) == TASK_ID_AHI)
        {
            param->app_task = gapm_get_task_from_id(param->app_task);
        }
        #endif // (AHI_TL_SUPPORT)

        // request to add the profile
        status = prf_add_profile(param, &prf_task);
			//	status = rom_env.prf_add_profile(param, &prf_task);

        if(status == GAP_ERR_NO_ERROR)
        {
            struct gapm_profile_added_ind *ind;
            #if(AHI_TL_SUPPORT)
            // for external application which AHI, update task Number from task ID
            if(KERNEL_TYPE_GET(param->app_task) == TASK_AHI)
            {
                prf_task = gapm_get_id_from_task(prf_task);
            }
            #endif // (AHI_TL_SUPPORT)

            // send an indication to inform that profile has been added
            ind = KERNEL_MSG_ALLOC(GAPM_PROFILE_ADDED_IND, src_id, dest_id, gapm_profile_added_ind);
            ind->prf_task_id = param->prf_task_id;
            ind->prf_task_nb = prf_task;
            ind->start_hdl   = param->start_hdl;
            kernel_msg_send(ind);
        }

        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}
#endif // (BLE_PROFILES)


/**
 ****************************************************************************************
 * @brief Default message handler
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_default_msg_handler(kernel_msg_id_t const msgid, void *event,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    return (KERNEL_MSG_CONSUMED);
}





/**
 ****************************************************************************************
 * @brief Handles request of generating a random address command
 * - GAPM_GEN_RAND_ADDR:  Generate a random address
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_gen_rand_addr_cmd_handler(kernel_msg_id_t const msgid, struct gapm_gen_rand_addr_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GEN_RAND_ADDR, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // no need to check GAPM_GEN_RAND_ADDR operation, this has been done by gapm_process_op

        uint8_t status = smpm_gen_rand_addr(param->rnd_type);

        if(status != GAP_ERR_NO_ERROR)
        {
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Command requested to SMP is completed.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_use_enc_block_cmd_handler(kernel_msg_id_t const msgid,
                                          struct gapm_use_enc_block_cmd * param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_USE_ENC_BLOCK, GAPM_GEN_RAND_NB, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // no need to check GAPM_USE_ENC_BLOCK, GAPM_GEN_RAND_NB operation, this has been done by gapm_process_op

        switch(msgid)
        {
            case GAPM_USE_ENC_BLOCK_CMD:
            {
                // request aes usage
                smpm_use_enc_block((uint8_t*)&(param->operand_1[0]), (uint8_t*)&(param->operand_2[0]));
            }
            break;
            case GAPM_GEN_RAND_NB_CMD:
            {
                // request to generate a random number
                smpm_gen_rand_nb();
            }
            break;
            default: /* nothing to do */ break;
        }
    }

    return msg_status;
}

#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles request of generating a DH Key command
 * - GAPM_GEN_DH_KEY :  Generate a DH Key
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 
static int gapm_gen_dh_key_cmd_handler(kernel_msg_id_t const msgid,
                                          struct gapm_gen_dh_key_cmd * param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GEN_DH_KEY, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_DHKEY, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        smpm_send_generate_dh_key((uint8_t*)&(param->operand_1[0]), (uint8_t*)&(param->operand_2[0]));
    }

    return msg_status;
}

#endif // (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Message to an Unknown task received
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_unknown_task_msg_handler(kernel_msg_id_t const msgid,  void * param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    struct kernel_msg* msg = kernel_param2msg(param);

    // inform main application that a message to an unknown task has been requested
    struct gapm_unknown_task_ind * ind = KERNEL_MSG_ALLOC(GAPM_UNKNOWN_TASK_IND, APP_MAIN_TASK, dest_id, gapm_unknown_task_ind);
    ind->msg_id = msg->param_len;
    ind->task_id = src_id;
    kernel_msg_send(ind);

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles management of resolving list command.
 *
 * This command can be used to :
 *  - GAPM_GET_RAL_SIZE:    Get resolving list size.
 *  - GAPM_GET_RAL_ADDR:    Get resolving address (nb = 0 local, else peer)
 *  - GAPM_ADD_DEV_IN_RAL:  Add devices in resolving list.
 *  - GAPM_RMV_DEV_FRM_RAL: Remove devices form resolving list.
 *  - GAPM_CLEAR_RAL:       Clear all devices from resolving list.
 *
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_ral_mgt_cmd_handler(kernel_msg_id_t const msgid,
        struct gapm_ral_mgt_cmd *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GET_RAL_SIZE, GAPM_CLEAR_RAL,
                                      GAPM_GET_RAL_LOC_ADDR, GAPM_GET_RAL_PEER_ADDR,
                                      GAPM_ADD_DEV_IN_RAL, GAPM_RMV_DEV_FRM_RAL,
                                      GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        switch(param->operation)
        {
            // Get resolving List Size.
            case GAPM_GET_RAL_SIZE:
            {
                gapm_basic_hci_cmd_send(HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE);
            }
            break;

            // Get address.
            case GAPM_GET_RAL_LOC_ADDR:
            case GAPM_GET_RAL_PEER_ADDR:
            {
                if(param->nb == 1)
                {
                    /* add specific address in the list */
                    struct hci_le_rd_loc_rslv_addr_cmd *rd_addr = KERNEL_MSG_ALLOC(
                            HCI_COMMAND, 0, (param->operation == GAPM_GET_RAL_LOC_ADDR) ?
                                    HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE : HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE,
                                    hci_le_rd_loc_rslv_addr_cmd);

                    /* fill up the parameters */
                    rd_addr->peer_id_addr_type = param->devices[0].addr_type;
                    memcpy(&rd_addr->peer_id_addr, &(param->devices[0].addr), BD_ADDR_LEN);

                    /* send the message */
                    hci_send_2_controller(rd_addr);
                }
                else
                {
                    /* send command complete succeed */
                    gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_INVALID_PARAM);
                }
            }
            break;

            // Clear all devices from resolving list.
            case GAPM_CLEAR_RAL:
            {
                // clear resolving list
                gapm_basic_hci_cmd_send(HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE);
            }
            break;

            // Add devices in resolving list.
            case GAPM_ADD_DEV_IN_RAL:
            {
                // Still some devices to add in resolving list.
                if(param->nb > 0)
                {
                    /* add specific address in the list */
                    struct hci_le_add_dev_to_rslv_list_cmd *add_dev = KERNEL_MSG_ALLOC(
                            HCI_COMMAND, 0, HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE, hci_le_add_dev_to_rslv_list_cmd);

                    /* fill up the parameters */
                    memcpy(add_dev, &(param->devices[param->nb-1]), sizeof(struct hci_le_add_dev_to_rslv_list_cmd));

                    /* send the message */
                    hci_send_2_controller(add_dev);
                }
                // operation is finished
                else
                {
                    /* send command complete succeed */
                    gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
                }
            }
            break;
            // Remove devices form resolving list.
            case GAPM_RMV_DEV_FRM_RAL:
            {
                // Still some devices to remove in resolving list.
                if(param->nb > 0)
                {
                    /* remove specific address in the list */
                    struct hci_le_rmv_dev_from_rslv_list_cmd *rem_dev = KERNEL_MSG_ALLOC(
                            HCI_COMMAND, 0, HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE, hci_le_rmv_dev_from_rslv_list_cmd);

                    /* fill up the parameters */
                    memcpy(&(rem_dev->peer_id_addr), &(param->devices[param->nb-1].addr), BD_ADDR_LEN);
                    rem_dev->peer_id_addr_type = param->devices[param->nb-1].addr_type;

                    /* send the message */
                    hci_send_2_controller(rem_dev);
                }
                // operation is finished
                else
                {
                    /* send command complete succeed */
                    gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
                }
            }
            break;

            default:
            {
                /* Nothing to do since operation has been checked by gapm_process_op */
            }
            break;
        };
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of changing the IRK
 * - GAPM_SET_IRK:  Set IRK
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_set_irk_cmd_handler(kernel_msg_id_t const msgid, struct gapm_set_irk_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_IRK, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // Set device Identity key (IRK)
        memcpy(&(gapm_env.irk), &(param->irk), sizeof(struct gap_sec_key));

        // Set Address renew flag
        GAPM_F_SET(gapm_env.cfg_flags, ADDR_RENEW, 1);
        // Clear timer
        kernel_timer_clear(GAPM_ADDR_RENEW_TO_IND , TASK_GAPM);

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of registering LE Protocol multiplexer
 * - GAPM_LEPSM_REG: Register a LE Protocol/Service Multiplexer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_lepsm_register_cmd_handler(kernel_msg_id_t const msgid, struct gapm_lepsm_register_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_LEPSM_REG, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_INVALID_PARAM;
        #if (BLE_LECB)

        // 1. search if LE_PSM is present.
        struct gapm_le_psm_info* info = gapm_le_psm_find(param->le_psm);

        // check that LE_PSM found
        if((info == NULL) && (KERNEL_TYPE_GET(param->app_task) != TASK_BLE_NONE))
        {
            #if(AHI_TL_SUPPORT)
            // for external application which AHI, update task Number from task ID
            if(KERNEL_TYPE_GET(param->app_task) == TASK_ID_AHI)
            {
                param->app_task = gapm_get_task_from_id(param->app_task);
            }
            #endif // (AHI_TL_SUPPORT)

            // 2. allocate a data structure for new LE_PSM registered.
            info = (struct gapm_le_psm_info*)kernel_malloc(sizeof(struct gapm_le_psm_info), KERNEL_MEM_ATT_DB);

            // 3. fill structure and put it in LE_PSM registered list.
            info->le_psm    = param->le_psm;
            info->sec_lvl   = param->sec_lvl;
            info->task_id   = KERNEL_TYPE_GET(param->app_task);
            info->nb_est_lk = 0;

            // 4. put LE_PSM info in registered list
            common_list_push_back(&(gapm_env.reg_le_psm), &(info->hdr));

            status = GAP_ERR_NO_ERROR;
        }

        #else // !(BLE_LECB)
        status = GAP_ERR_COMMAND_DISALLOWED;
        #endif // (BLE_LECB)
        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of unregistering LE Protocol multiplexer
 * - GAPM_LEPSM_UNREG: Unregister a LE Protocol/Service Multiplexer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapm_lepsm_unregister_cmd_handler(kernel_msg_id_t const msgid, struct gapm_lepsm_unregister_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_LEPSM_UNREG, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        #if (BLE_LECB)

        // 1. search if LE_PSM is present.
        struct gapm_le_psm_info* info = gapm_le_psm_find(param->le_psm);

        if(info == NULL)
        {
            status = L2C_ERR_LEPSM_NOT_SUPP;
        }
        // 2. If present remove it from list
        else if(info->nb_est_lk == 0)
        {
            // remove registered LE_PSM from list
            common_list_extract(&(gapm_env.reg_le_psm), &(info->hdr), 0);
            kernel_free(info);
        }
        else
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }

        #else // !(BLE_LECB)
        status = GAP_ERR_NOT_SUPPORTED;
        #endif // (BLE_LECB)
        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}

// HCI handler is present into a dedicated module
extern int gapm_hci_handler(kernel_msg_id_t const msgid, void const* event, kernel_task_id_t dest_id, kernel_task_id_t src_id);

/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
const struct kernel_msg_handler gapm_default_state[] =
{
    // note: first message is latest message checked by kernel so default is put on top.
    { KERNEL_MSG_DEFAULT_HANDLER,                 (kernel_msg_func_t) gapm_default_msg_handler },

    /* Reset command */
    { GAPM_RESET_CMD,                         (kernel_msg_func_t) gapm_reset_cmd_handler },

    /* Cancel operation command */
    { GAPM_CANCEL_CMD,                        (kernel_msg_func_t) gapm_cancel_cmd_handler },

    /* Device set configuration */
    { GAPM_SET_DEV_CONFIG_CMD,                (kernel_msg_func_t) gapm_set_dev_config_cmd_handler },
    { GAPM_SET_CHANNEL_MAP_CMD,               (kernel_msg_func_t) gapm_set_channel_map_cmd_handler },

    /* Device get configuration */
    { GAPM_GET_DEV_INFO_CMD ,                 (kernel_msg_func_t) gapm_get_dev_info_cmd_handler },

    /* White list management */
    { GAPM_WHITE_LIST_MGT_CMD,                (kernel_msg_func_t) gapm_white_list_mgt_cmd_handler },

    /* Address resolution */
    { GAPM_RESOLV_ADDR_CMD,                   (kernel_msg_func_t) gapm_resolv_addr_cmd_handler },
    { GAPM_GEN_RAND_ADDR_CMD,                 (kernel_msg_func_t) gapm_gen_rand_addr_cmd_handler },
    { GAPM_USE_ENC_BLOCK_CMD,                 (kernel_msg_func_t) gapm_use_enc_block_cmd_handler },
    #if (SECURE_CONNECTIONS)
    { GAPM_GEN_DH_KEY_CMD,                    (kernel_msg_func_t) gapm_gen_dh_key_cmd_handler   },
    #endif // (SECURE_CONNECTIONS)
    { GAPM_GEN_RAND_NB_CMD,                   (kernel_msg_func_t) gapm_use_enc_block_cmd_handler },

    /* Advertising procedure */
    #if (BLE_PERIPHERAL || BLE_BROADCASTER)
    { GAPM_START_ADVERTISE_CMD,               (kernel_msg_func_t) gapm_start_advertise_cmd_handler },
    { GAPM_UPDATE_ADVERTISE_DATA_CMD,         (kernel_msg_func_t) gapm_update_advertise_data_cmd_handler },
    { GAPM_LIM_DISC_TO_IND,                   (kernel_msg_func_t) gapm_lim_disc_to_ind_handler },
    #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

    /* Scan procedure */
    #if (BLE_CENTRAL || BLE_OBSERVER)
    { GAPM_START_SCAN_CMD,                    (kernel_msg_func_t) gapm_start_scan_cmd_handler },
    { GAPM_SCAN_TO_IND,                       (kernel_msg_func_t) gapm_scan_to_ind_handler },
    #endif // (BLE_CENTRAL || BLE_OBSERVER)

    /* Connection procedure */
    #if (BLE_CENTRAL)
    { GAPM_START_CONNECTION_CMD,              (kernel_msg_func_t) gapm_start_connection_cmd_handler },
    { GAPC_CMP_EVT,                           (kernel_msg_func_t) gapc_cmp_evt_handler },
    { GAPC_PEER_ATT_INFO_IND,                 (kernel_msg_func_t) gapc_peer_att_info_ind_handler },
    { GAPM_CONNECTION_CFM,                    (kernel_msg_func_t) gapm_connection_cfm_handler },
    #endif // (BLE_CENTRAL)
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    { GAPM_CMP_EVT,                           (kernel_msg_func_t) gapm_cmp_evt_handler },
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    /* Address Management */
    { GAPM_ADDR_RENEW_TO_IND,                 (kernel_msg_func_t) gapm_addr_renew_to_ind_handler },

    /* Profile Management */
    #if BLE_PROFILES
    { GAPM_PROFILE_TASK_ADD_CMD,              (kernel_msg_func_t) gapm_profile_task_add_cmd_handler },
    #endif // (BLE_PROFILES)

    { GAPM_UNKNOWN_TASK_MSG,                  (kernel_msg_func_t) gapm_unknown_task_msg_handler },

    /* HCI management */
    { HCI_CMD_CMP_EVENT,                      (kernel_msg_func_t) gapm_hci_handler },
    { HCI_CMD_STAT_EVENT,                     (kernel_msg_func_t) gapm_hci_handler },
    { HCI_LE_EVENT,                           (kernel_msg_func_t) gapm_hci_handler },

    /* Resolving list management */
    { GAPM_RAL_MGT_CMD,                       (kernel_msg_func_t) gapm_ral_mgt_cmd_handler },

    /* Set IRK */
    { GAPM_SET_IRK_CMD,                       (kernel_msg_func_t) gapm_set_irk_cmd_handler },

    /* LE Credit Based Channel Management */
    { GAPM_LEPSM_REGISTER_CMD,                (kernel_msg_func_t) gapm_lepsm_register_cmd_handler },
    { GAPM_LEPSM_UNREGISTER_CMD,              (kernel_msg_func_t) gapm_lepsm_unregister_cmd_handler },
};


/// Message handlers that are common to all states.
const struct kernel_state_handler gapm_default_handler = KERNEL_STATE_HANDLER(gapm_default_state);

/// GATT task instance.
kernel_state_t gapm_state[GAPM_IDX_MAX];


/// @} GAPM_TASK
