/**
 ****************************************************************************************
 *
 * @file gapc_task.c
 *
 * @brief Generic Access Profile Controller Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC_TASK Generic Access Profile Controller Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)

#include "common_math.h"

#include "gap.h"
#include "gapc_int.h"
#include "gapc_sig.h"

#include "gapm.h"
#include "gapm_task.h"

#include "gattc_task.h"
#include "gattc.h"
#include "gattm.h"

#include "smpc_api.h" // Access to internal API Required

#include "l2cc.h"

#include "hci.h"
#include "attm.h"
#include "kernel_timer.h"

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


/**
 ****************************************************************************************
 * @brief Check if current operation can be processed or not.
 * if it can be proceed, initialize an operation request.
 * If a command complete event with error code can be triggered.
 *
 * Function returns how the message should be handled by message handler.
 *
 * @param[in] conidx        Connection Index
 * @param[in] op_type       Operation type.
 * @param[in] op_msg        Requested operation message (note op_msg cannot be null)
 * @param[in] supp_ops      Supported operations array.
 *                          Latest array value shall be GAPC_NO_OP.
 *
 * @return operation can be executed if message status equals KERNEL_MSG_NO_FREE,
 * else nothing to do, just exit from the handler.
 ****************************************************************************************
 */
int gapc_process_op(uint8_t conidx, uint8_t op_type, void* op_msg, enum gapc_operation* supp_ops)
{
    ASSERT_ERR(op_type < GAPC_OP_MAX);
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED; // Reset State
    // Current process state
    uint8_t state = kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx));
    uint8_t operation = *((uint8_t*)op_msg);

    uint8_t status = GAP_ERR_COMMAND_DISALLOWED;

    /* no operation on going or requested operation is current on going operation. */
    // in a disconnect state, reject all commands
    if((state != GAPC_FREE) && (state != GAPC_DISC_BUSY))
    {
        if(gapc_get_operation_ptr(conidx, op_type) != op_msg)
        {
            status = GAP_ERR_NO_ERROR;

            // check what to do with command if an operation is ongoing.
            if((state & (1 << op_type)) != GAPC_READY)
            {
                // operation are queued by default
                // save it for later.
                msg_status = KERNEL_MSG_SAVED;
            }
            else
            {
                // check if operation is supported
                while(*supp_ops != GAPC_NO_OP)
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
                if(*supp_ops == GAPC_NO_OP)
                {
                    status = GAP_ERR_INVALID_PARAM;
                }
                else
                {
                    // message memory will be managed by GAPM
                    msg_status = KERNEL_MSG_NO_FREE;

                    // store operation
                    gapc_set_operation_ptr(conidx, op_type, op_msg);

                    // set state to busy
                    gapc_update_state(conidx, (1 << op_type), true);
                }
            }
        }
        else
        {
            // message memory managed by GAPC
            msg_status = KERNEL_MSG_NO_FREE;
            status = GAP_ERR_NO_ERROR;
        }
    }

    // if an error detected, send command completed with error status
    if(status != GAP_ERR_NO_ERROR)
    {
        gapc_send_error_evt(conidx, operation, kernel_msg_src_id_get(op_msg), status);
    }

    return msg_status;
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles disconnection of BLE link request.
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
static int gapc_disconnect_cmd_handler(kernel_msg_id_t const msgid, struct gapc_disconnect_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check operation
    switch(param->operation)
    {
        case GAPC_DISCONNECT:
        {
            // Current process state
            uint8_t state = kernel_state_get(dest_id);

            // Operation can be handled - not an operation because shall be handled immediately
            if((state != GAPC_FREE) && gapc_env[idx]->disc_requester == 0)
            {
                // set state to busy
                kernel_state_set(KERNEL_BUILD_ID(TASK_GAPC, idx), GAPC_DISC_BUSY);
                gapc_env[idx]->disc_requester = src_id;

                /* allocate disconnect command message */
                struct hci_disconnect_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_DISCONNECT_CMD_OPCODE, hci_disconnect_cmd);
                cmd->conhdl = gapc_get_conhdl(idx);
                cmd->reason = param->reason;
                /* send the request */
                hci_send_2_controller(cmd);
            }
            else
            {
                // Send an error to the application
                gapc_send_error_evt(idx, param->operation, src_id, GAP_ERR_COMMAND_DISALLOWED);
            }
        }
        break;

        default:
        {
            // Invalid operation code, reject the request
            gapc_send_error_evt(idx, param->operation, src_id, GAP_ERR_INVALID_PARAM);
        }
        break;
    }

    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles reception of connection information retrieve from a precedent connection.
 * It mainly contains some bond data.
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
static int gapc_connection_cfm_handler(kernel_msg_id_t const msgid, struct gapc_connection_cfm *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);

    // Operation can be handled
    if(state != GAPC_FREE)
    {
        // Current connection index
        uint8_t idx = KERNEL_IDX_GET(dest_id);

        // check if connection contains bonding data.
        if((param->auth & GAP_AUTH_BOND) == GAP_AUTH_BOND)
        {
            // ***** fill bonded data. ***** //
            // authentication level
            gapc_auth_set(idx, param->auth, param->ltk_present);

            // Service change indication configuration
            GAPC_SET_FIELD(idx, SVC_CHG_CCC, param->svc_changed_ind_enable);

            // Sign counters.
            gapc_env[idx]->smpc.sign_counter[SMPC_INFO_LOCAL] = param->lsign_counter;
            gapc_env[idx]->smpc.sign_counter[SMPC_INFO_PEER]  = param->rsign_counter;

            // CSRKs values
            memcpy(&(gapc_env[idx]->smpc.csrk[SMPC_INFO_LOCAL]), &(param->lcsrk),
                    sizeof(struct gap_sec_key));
            memcpy(&(gapc_env[idx]->smpc.csrk[SMPC_INFO_PEER]), &(param->rcsrk),
                    sizeof(struct gap_sec_key));
        }
        // no bond data present, nothing to restore.
        else
        {
            // Not paired because no bonding data available.
            gapc_auth_set(idx, GAP_AUTH_NONE, false);
        }

        // Inform lower layers that link is ready to be used.
        gapm_con_enable(idx);
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles request to retrieve connection informations.
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
static int gapc_get_info_cmd_handler(kernel_msg_id_t const msgid, struct gapc_get_info_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_GET_PEER_NAME, GAPC_GET_PEER_APPEARANCE,
            GAPC_GET_PEER_SLV_PREF_PARAMS, GAPC_GET_ADDR_RESOL_SUPP, GAPC_GET_PEER_VERSION,
            GAPC_GET_PEER_FEATURES, GAPC_GET_CON_RSSI, GAPC_GET_CON_CHANNEL_MAP,
            GAPC_GET_LE_PING_TO, GAPC_GET_PHY, GAPC_NO_OP};
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_LINK_INFO, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        /// check operation
        switch(param->operation)
        {
            case GAPC_GET_PEER_NAME:
            case GAPC_GET_PEER_APPEARANCE:
            case GAPC_GET_PEER_SLV_PREF_PARAMS:
            case GAPC_GET_ADDR_RESOL_SUPP:
            {
                uint16_t uuid = ATT_CHAR_DEVICE_NAME;
                /* allocate read name req message */
                struct gattc_read_cmd *read_cmd = KERNEL_MSG_ALLOC_DYN(GATTC_READ_CMD,
                        KERNEL_BUILD_ID(TASK_GATTC,idx), dest_id, gattc_read_cmd,
                        ATT_UUID_16_LEN);

                /* fill up parameters */
                read_cmd->operation = GATTC_READ_BY_UUID;
                read_cmd->nb = 1;
                read_cmd->req.by_uuid.start_hdl = ATT_1ST_REQ_START_HDL;
                read_cmd->req.by_uuid.end_hdl   = ATT_1ST_REQ_END_HDL;
                read_cmd->req.by_uuid.uuid_len  = ATT_UUID_16_LEN;

                // set value to read
                if(param->operation == GAPC_GET_PEER_APPEARANCE)
                {
                    uuid = ATT_CHAR_APPEARANCE;
                }
                else if (param->operation == GAPC_GET_PEER_SLV_PREF_PARAMS)
                {
                    uuid = ATT_CHAR_PERIPH_PREF_CON_PARAM;
                }
                else if (param->operation == GAPC_GET_ADDR_RESOL_SUPP)
                {
                    uuid = ATT_CHAR_CTL_ADDR_RESOL_SUPP;
                }

                common_write16p(read_cmd->req.by_uuid.uuid, uuid);

                /* send the message to GATTC */
                kernel_msg_send(read_cmd);
            }
            break;

            case GAPC_GET_PEER_VERSION:
            {
                /* allocate read version message */
                struct hci_rd_rem_ver_info_cmd *read_ver = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_RD_REM_VER_INFO_CMD_OPCODE, hci_rd_rem_ver_info_cmd);

                read_ver->conhdl = gapc_get_conhdl(idx);

                /* send the message */
                hci_send_2_controller(read_ver);
            }
            break;

            case GAPC_GET_PEER_FEATURES:
            {
                /* allocate read feature message */
                struct hci_le_rd_rem_used_feats_cmd *read_feat = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_RD_REM_USED_FEATS_CMD_OPCODE, hci_le_rd_rem_used_feats_cmd);

                read_feat->conhdl = gapc_get_conhdl(idx);

                /* send the message */
                hci_send_2_controller(read_feat);
            }
            break;

            case GAPC_GET_CON_RSSI:
            {
                /* allocate read rssi message */
                struct hci_basic_conhdl_cmd *rd_rssi = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_RD_RSSI_CMD_OPCODE, hci_basic_conhdl_cmd);

                rd_rssi->conhdl = gapc_get_conhdl(idx);

                /* message send */
                hci_send_2_controller(rd_rssi);
            }
            break;
            // retrieve connection channel map
            case GAPC_GET_CON_CHANNEL_MAP:
            {
                /* allocate read rssi message */
                struct hci_basic_conhdl_cmd* rd_ch_map = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_RD_CHNL_MAP_CMD_OPCODE, hci_basic_conhdl_cmd);

                rd_ch_map->conhdl = gapc_get_conhdl(idx);

                /* message send */
                hci_send_2_controller(rd_ch_map);
            }
            break;
            case GAPC_GET_LE_PING_TO:
            {
               /* allocate get timeout message */
               struct hci_rd_auth_payl_to_cmd* rd_ping_to = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, hci_rd_auth_payl_to_cmd);

               rd_ping_to->conhdl = gapc_get_conhdl(idx);

               /* message send */
               hci_send_2_controller(rd_ping_to);
            }
            break;
            case GAPC_GET_PHY:
            {
                #if (BLE_2MBPS)
                struct hci_le_rd_phy_cmd* rd_phy = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_RD_PHY_CMD_OPCODE, hci_le_rd_phy_cmd);
                rd_phy->conhdl = gapc_get_conhdl(idx);
                hci_send_2_controller(rd_phy);
                #else
                gapc_send_complete_evt(idx, GAPC_OP_LINK_INFO, GAP_ERR_NOT_SUPPORTED);
                #endif // (BLE_2MBPS)
            } break;
            default: /*  Could not enter here  */ break;
        };
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles gatt read indication event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_read_ind_handler(kernel_msg_id_t const msgid,
        struct gattc_read_ind const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    if(state != GAPC_FREE)
    {
        switch(gapc_get_operation(conidx, GAPC_OP_LINK_INFO))
        {
            case GAPC_GET_PEER_NAME:
            {
                // create name information indication message.
                struct gapc_peer_att_info_ind * name_ind =
                        KERNEL_MSG_ALLOC_DYN(GAPC_PEER_ATT_INFO_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_peer_att_info_ind, param->length);

                name_ind->req                = GAPC_DEV_NAME;
                name_ind->handle             = param->handle;
                name_ind->info.name.length   = param->length;
                memcpy(name_ind->info.name.value, param->value, param->length);

                // send indication
                kernel_msg_send(name_ind);
            }
            break;

            case GAPC_GET_PEER_APPEARANCE:
            {
                // create name information indication message.
                struct gapc_peer_att_info_ind * icon_ind =
                        KERNEL_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_peer_att_info_ind);

                icon_ind->req                = GAPC_DEV_APPEARANCE;
                icon_ind->handle             = param->handle;
                icon_ind->info.appearance    = common_read16p(param->value);

                // send indication
                kernel_msg_send(icon_ind);
            }
            break;

            case GAPC_GET_PEER_SLV_PREF_PARAMS:
            {
                // create name information indication message.
                struct gapc_peer_att_info_ind * param_ind =
                        KERNEL_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_peer_att_info_ind);

                param_ind->req                           = GAPC_DEV_SLV_PREF_PARAMS;
                param_ind->handle                        = param->handle;
                param_ind->info.slv_params.con_intv_min  = common_read16p(&(param->value[0]));
                param_ind->info.slv_params.con_intv_max  = common_read16p(&(param->value[2]));
                param_ind->info.slv_params.slave_latency = common_read16p(&(param->value[4]));
                param_ind->info.slv_params.conn_timeout  = common_read16p(&(param->value[6]));

                // send indication
                kernel_msg_send(param_ind);
            }
            break;

            case GAPC_GET_ADDR_RESOL_SUPP:
            {
                // create address resolution indication message.
                struct gapc_peer_att_info_ind * param_ind =
                        KERNEL_MSG_ALLOC(GAPC_PEER_ATT_INFO_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_peer_att_info_ind);

                param_ind->req                           = GAPC_DEV_CTL_ADDR_RESOL;
                param_ind->handle                        = param->handle;
                param_ind->info.cnt_addr_resol           = param->value[0];

                // send indication
                kernel_msg_send(param_ind);
            }
            break;

            default: /* Do Nothing */ break;
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles gatt command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_cmp_evt_handler(kernel_msg_id_t const msgid,
        struct gattc_cmp_evt const *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    if(state != GAPC_FREE)
    {
        switch(param->operation)
        {
            case GATTC_READ_BY_UUID:
            #if (BLE_CENTRAL)
            case GATTC_WRITE:
            #endif // (BLE_CENTRAL)
            {
                // Disconnect ongoing due to name request timeout
                if((gapc_get_operation(conidx, GAPC_OP_LINK_INFO) == GAPC_GET_PEER_NAME)
                        && (gapc_get_requester(conidx, GAPC_OP_LINK_INFO) == TASK_GAPM)
                        && (state == GAPC_DISC_BUSY))
                {
                    // mark task disconnection requested by GAPM
                    gapc_env[conidx]->disc_requester = TASK_GAPM;
                }

                /* send command complete event */
                gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, param->status);
            }
            break;
            #if (BLE_CENTRAL)
            case GATTC_DISC_BY_UUID_CHAR:
            {
                uint16_t handle = ATT_INVALID_HANDLE;

                switch(gapc_get_operation(conidx, GAPC_OP_LINK_INFO))
                {
                    default: /* Do Nothing */ break;
                }

                // attribute not found
                if(handle == ATT_INVALID_HANDLE)
                {
                    gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, ATT_ERR_INVALID_HANDLE);
                }
                else
                {
                    // perform write request.
                    gapc_reschedule_operation(conidx, GAPC_OP_LINK_INFO);
                }
            }
            break;
            #endif // (BLE_CENTRAL)
            default:
            {
                /* send command complete event */
                gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, GAP_ERR_PROTOCOL_PROBLEM);
            }
            break;
        }
    }
    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request to Update connection parameters.
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
static int gapc_param_update_cmd_handler(kernel_msg_id_t const msgid, struct  gapc_param_update_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_UPDATE_PARAMS, GAPC_NO_OP};

    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_LINK_UPD, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        /* perform a sanity check of connection parameters */
        if (!gapc_param_update_sanity(param->intv_max,
                param->intv_min,
                param->latency,
                param->time_out))
        {
            /* send command complete event with error */
            gapc_send_complete_evt(idx, GAPC_OP_LINK_UPD, GAP_ERR_INVALID_PARAM);
        }
        else
        {
            // check if parameter request feature is supported
            if(GAPM_ISBITSET(gapc_env[idx]->features,GAPC_CONN_PARAM_REQ_FEAT_MASK)
                    || (GAPC_GET_FIELD(idx, ROLE) == ROLE_MASTER))
            {
                if(KERNEL_TYPE_GET(src_id) == TASK_GAPC)
                {
                    struct gapc_param_update_req_ind *req_ind = KERNEL_MSG_ALLOC(GAPC_PARAM_UPDATE_REQ_IND,
                                                APP_MAIN_TASK, dest_id, gapc_param_update_req_ind);

                    /* fill up parameters */
                    req_ind->intv_max = param->intv_max;
                    req_ind->intv_min = param->intv_min;
                    req_ind->latency  = param->latency;
                    req_ind->time_out = param->time_out;

                    /* send the indication */
                    kernel_msg_send(req_ind);
                }
                else
                {
                    struct hci_le_con_update_cmd* up_con_req =
                            KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_CON_UPDATE_CMD_OPCODE,
                                    hci_le_con_update_cmd);

                    /* fill up the parameters */
                    up_con_req->conhdl       = gapc_get_conhdl(idx);
                    up_con_req->con_latency  = param->latency;
                    up_con_req->superv_to    = param->time_out;
                    up_con_req->con_intv_max = param->intv_max;
                    up_con_req->con_intv_min = param->intv_min;
                    up_con_req->ce_len_max   = param->ce_len_max;
                    up_con_req->ce_len_min   = param->ce_len_min;

                    /* send the message */
                    hci_send_2_controller(up_con_req);
                }
            }
            #if (BLE_PERIPHERAL)
            else
            {
                struct l2cc_update_param_req *req = L2CC_SIG_PDU_ALLOC(idx, L2C_CODE_CONN_PARAM_UPD_REQ,
                                                                   KERNEL_BUILD_ID(TASK_GAPC, idx), l2cc_update_param_req);

                // generate packet identifier
                param->pkt_id = common_rand_word() & 0xFF;
                if(param->pkt_id == 0)
                {
                    param->pkt_id = 1;
                }

                /* fill up the parameters */
                req->intv_max = param->intv_max;
                req->intv_min = param->intv_min;
                req->latency  = param->latency;
                req->timeout  = param->time_out;
                req->pkt_id   = param->pkt_id;

                l2cc_pdu_send(req);

                /* Start timer */
                kernel_timer_set(GAPC_PARAM_UPDATE_TO_IND, dest_id, GAP_TMR_CONN_PARAM_TIMEOUT);
            }
            #endif // (BLE_PERIPHERAL)

        }
    }
    return msg_status;
}



/**
 ****************************************************************************************
 * @brief Receives a pdu that should be handled by GAP. This PDU is a signaling packet
 * such as Update connection parameters or pdu error.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_pdu_recv_ind_handler(kernel_msg_id_t const msgid, struct l2cc_pdu_recv_ind *ind,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;

    if(state != GAPC_FREE)
    {
        switch (ind->pdu.chan_id)
        {
            // Signaling PDU
            case L2C_CID_LE_SIGNALING:
            {
                // Handles signaling message received
                msg_status = gapc_sig_pdu_recv_handler(conidx, &(ind->pdu));
            } break;
            // Security
            case L2C_CID_SECURITY:
            {
                #if (BLE_SMPC)
                // In encrypt state, postpone message process
                if ((state & GAPC_ENCRYPT_BUSY)== GAPC_ENCRYPT_BUSY)
                {
                    msg_status = KERNEL_MSG_SAVED;
                }
                // Allow PDU reception in any state but FREE
                else
                {
                    if(ind->status == GAP_ERR_NO_ERROR)
                    {
                        // Decide next action depending on received PDU
                        smpc_pdu_recv(KERNEL_IDX_GET(dest_id), &(ind->pdu));
                    }
                    else if(ind->status == L2C_ERR_INVALID_PDU)
                    {
                        struct l2cc_pairing_failed *err_msg = L2CC_SMP_PDU_ALLOC(conidx, L2C_CODE_PAIRING_FAILED,
                                                                         KERNEL_BUILD_ID(TASK_GAPC, conidx), l2cc_pairing_failed);
                        /* fill up the parameters */
                        err_msg->reason   = SMP_ERROR_CMD_NOT_SUPPORTED;
                        // Send message to L2CAP
                        l2cc_pdu_send(err_msg);
                    }
                }
                #else // (BLE_SMPC)
                struct l2cc_pairing_failed *err_msg = L2CC_SMP_PDU_ALLOC(conidx, L2C_CODE_PAIRING_FAILED,
                                                                 KERNEL_BUILD_ID(TASK_GAPC, conidx), l2cc_pairing_failed);
                /* fill up the parameters */
                err_msg->reason   = SMP_ERROR_PAIRING_NOT_SUPP;
                // Send message to L2CAP
                l2cc_pdu_send(err_msg);
                #endif // (BLE_SMPC)
            } break;
            // Invalid CID
            default:
            {
                ASSERT_INFO(0, ind->pdu.chan_id, conidx);
            } break;
        }
    }
    /* message is consumed */
    return (msg_status);
}

#if (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Response time out indication. Peer did not respond to signaling request.
 * This will cause the link to be disconnected.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_update_conn_param_to_ind_handler(kernel_msg_id_t const msgid, void const *param,
                 kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);

    if((state != GAPC_FREE) && (gapc_get_operation(conidx, GAPC_OP_LINK_UPD) == GAPC_UPDATE_PARAMS))
    {
        /// disconnect the link
        struct gapc_disconnect_cmd *discon_req = KERNEL_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                dest_id, dest_id, gapc_disconnect_cmd);

        discon_req->operation = GAPC_DISCONNECT;
        discon_req->reason = (uint8_t) COMMON_ERROR_UNACCEPTABLE_CONN_INT;

        kernel_msg_send(discon_req);

        // inform that request has timeout
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, GAP_ERR_TIMEOUT);
    }

    return (KERNEL_MSG_CONSUMED);
}

#endif // (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Confirmation or not of Slave connection parameters
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_cfm_handler(kernel_msg_id_t const msgid, struct gapc_param_update_cfm const *cfm,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        struct gapc_param_update_cmd* cmd =
                            (struct gapc_param_update_cmd*) gapc_get_operation_ptr(idx, GAPC_OP_LINK_UPD);

        // check if message can be proceed
        if((gapc_get_operation(idx, GAPC_OP_LINK_UPD) == GAPC_UPDATE_PARAMS)
                && (GAPM_ISBITSET(gapc_env[idx]->features,GAPC_CONN_PARAM_REQ_FEAT_MASK)))
        {
            // if parameters are accepted, update them.
            if(cfm->accept)
            {
                struct hci_le_rem_con_param_req_reply_cmd* con_upd_reply =
                        KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE,
                                    hci_le_rem_con_param_req_reply_cmd);

                /* fill up the parameters */
                con_upd_reply->conhdl       = gapc_get_conhdl(idx);
                con_upd_reply->interval_min = cmd->intv_min;
                con_upd_reply->interval_max = cmd->intv_max;
                con_upd_reply->latency      = cmd->latency;
                con_upd_reply->min_ce_len   = cmd->ce_len_min;
                con_upd_reply->max_ce_len   = cmd->ce_len_max;
                con_upd_reply->timeout      = cmd->time_out;
                /* send the message */
                hci_send_2_controller(con_upd_reply);
            }
            else
            {
                struct hci_le_rem_con_param_req_neg_reply_cmd* con_upd_reply =
                        KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE,
                                    hci_le_rem_con_param_req_neg_reply_cmd);

                /* fill up the parameters */
                con_upd_reply->conhdl = gapc_get_conhdl(idx);
                // transform HL error to HCI error code.
                con_upd_reply->reason = COMMON_ERROR_UNACCEPTABLE_CONN_INT;

                /* send the message */
                hci_send_2_controller(con_upd_reply);

                // command execution finished
                gapc_send_complete_evt(idx, GAPC_OP_LINK_UPD, GAP_ERR_REJECTED);
            }
        }
        else if(gapc_get_operation(idx, GAPC_OP_LINK_UPD) == GAPC_UPDATE_PARAMS)
        {
            #if (BLE_CENTRAL)
            // send response to peer device
            gapc_sig_send_param_resp(idx, (cfm->accept ? L2C_CONN_PARAM_ACCEPT : L2C_CONN_PARAM_REJECT), cmd->pkt_id);

            // if parameters are accepted, update them.
            if(cfm->accept)
            {
                struct hci_le_con_update_cmd* up_con_req = KERNEL_MSG_ALLOC(HCI_COMMAND, idx,
                        HCI_LE_CON_UPDATE_CMD_OPCODE, hci_le_con_update_cmd);

                /* fill up the parameters */
                up_con_req->conhdl       = gapc_get_conhdl(idx);
                up_con_req->con_latency  = cmd->latency;
                up_con_req->superv_to    = cmd->time_out;
                up_con_req->con_intv_max = cmd->intv_max;
                up_con_req->con_intv_min = cmd->intv_min;
                up_con_req->ce_len_max   = cfm->ce_len_max;
                up_con_req->ce_len_min   = cfm->ce_len_min;

                /* send the message */
                hci_send_2_controller(up_con_req);
            }
            else
            {
                // command execution finished
                gapc_send_complete_evt(idx, GAPC_OP_LINK_UPD, GAP_ERR_REJECTED);
            }
            #endif // (BLE_CENTRAL)
        }
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles request to Start a bonding procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapc_bond_cmd_handler(kernel_msg_id_t const msgid, struct  gapc_bond_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_BOND, GAPC_NO_OP};
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_SMP, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;

        #if (BLE_CENTRAL)
        // check connection role.
        if(gapc_get_role(idx) == ROLE_MASTER)
        {
            // request SMPC to start pairing
            status = smpc_pairing_start(idx, &(param->pairing));
        }
        else
        #endif // (BLE_CENTRAL)
        {
            #if (BLE_PERIPHERAL)
            // internal
            if(dest_id == src_id)
            {
                // internally handle the pairing request as a BOND command
                smpc_pairing_req_handler(idx, &(param->pairing));
            }
            else
            {
                /* command supported only by master of the connection. */
                status = GAP_ERR_NOT_SUPPORTED;
            }
            #endif // (BLE_PERIPHERAL)
        }

        // check if pairing has been aborted
        if(status != GAP_ERR_NO_ERROR)
        {
            gapc_send_complete_evt(idx, GAPC_OP_SMP, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handle reception of application confirmation for bonding procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_bond_cfm_handler(kernel_msg_id_t const msgid, struct gapc_bond_cfm *cfm,
                                 kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED;
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if message can be proceed
    if(state != GAPC_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;

        switch(cfm->request)
        {
            case GAPC_PAIRING_RSP:
            {
                status = smpc_pairing_rsp(idx, cfm->accept, &(cfm->data.pairing_feat));
            }
            break;
            // Connection signature resolving key provided
            case GAPC_CSRK_EXCH:
            {
                status = smpc_pairing_csrk_exch(idx, &(cfm->data.csrk));
            }
            break;

            // Temporary Key exchange
            case GAPC_TK_EXCH:
            {
                status = smpc_pairing_tk_exch(idx, cfm->accept, &(cfm->data.tk));
            }
            break;

            // Long Term Key exchange
            case GAPC_LTK_EXCH:
            {
                status = smpc_pairing_ltk_exch(idx, &(cfm->data.ltk));
            }
            break;

            // Identity Resolving Key exchange
            case GAPC_IRK_EXCH:
            {
                status = smpc_pairing_irk_exch(idx, &(cfm->data.irk.irk), &(cfm->data.irk.addr));
            }
            break;
            #if (SECURE_CONNECTIONS)
            case GAPC_OOB_EXCH:
            {
                status = smpc_pairing_oob_exch(idx, cfm->accept, &(cfm->data.oob));
            }
            break;

            case GAPC_NC_EXCH :
            {
                status = smpc_pairing_nc_exch(idx,cfm->accept);
            }
            break;
            #endif // (SECURE_CONNECTIONS)

            // no break;
            default: /* Do nothing and ignore message */ break;
        }

        if ((status != GAP_ERR_NO_ERROR) && (gapc_env[idx]->smpc.state != SMPC_STATE_RESERVED))
        {
            // Send the Pairing Failed PDU to the peer device
            smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, (void *)&status);

            // Apply the error mask to know who triggered the error
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status);

            // Inform the HL that an error has occurred.
            smpc_pairing_end(idx, gapc_get_role(idx), status, true);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request to Start an Encryption procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapc_encrypt_cmd_handler(kernel_msg_id_t const msgid, struct  gapc_encrypt_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_ENCRYPT, GAPC_NO_OP};
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_SMP, param, supp_ops);

    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;

        #if (BLE_CENTRAL)
        // check connection role.
        if(gapc_get_role(idx) == ROLE_MASTER)
        {
            // use SMPC to start encryption
            status = smpc_encrypt_start(idx, &(param->ltk));
        }
        else
        #endif // (BLE_CENTRAL)
        {
            #if (BLE_PERIPHERAL)
            // internal
            if(dest_id == src_id)
            {
                // internally handle the encryption request as an ENCRYPT command
                smpc_encrypt_start_handler(idx, &(param->ltk));
            }
            else
            {
                /* command supported only by master of the connection. */
                status = GAP_ERR_NOT_SUPPORTED;
            }
            #endif // (BLE_PERIPHERAL)
        }

        // check if pairing has been aborted
        if(status != GAP_ERR_NO_ERROR)
        {
            gapc_send_complete_evt(idx, GAPC_OP_SMP, status);
        }
    }

    return msg_status;
}




#if (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handle reception of application confirmation for encryption procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_encrypt_cfm_handler(kernel_msg_id_t const msgid, struct gapc_encrypt_cfm *cfm,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Returned message status
    int msg_status = KERNEL_MSG_CONSUMED;
    // Current process state
    uint8_t state = kernel_state_get(dest_id);
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if message can be proceed
    if((state != GAPC_FREE) && (gapc_get_role(idx) == ROLE_SLAVE))
    {
        smpc_encrypt_cfm(idx, cfm->found, &(cfm->ltk), cfm->key_size);
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request to start security request procedure.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
static int gapc_security_cmd_handler(kernel_msg_id_t const msgid, struct  gapc_security_cmd *param,
                   kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_SECURITY_REQ, GAPC_NO_OP};
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_SMP, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status;

        // check connection role.
        if(gapc_get_role(idx) == ROLE_SLAVE)
        {
            // use SMPC to send security request
            status = smpc_security_req_send(idx, param->auth);
        }
        else
        {
            /* command supported only by slave of the connection. */
            status = GAP_ERR_NOT_SUPPORTED;
        }

        // check if request should be aborted
        if(status != GAP_ERR_NO_ERROR)
        {
            gapc_send_complete_evt(idx, GAPC_OP_SMP, status);
        }
    }

    return msg_status;
}
#endif // (BLE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Command requested to Sign a packet or resolve signature.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_sign_cmd_handler(kernel_msg_id_t const msgid,
                                 struct gapc_sign_cmd *param,
                                 kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_SIGN_PACKET, GAPC_SIGN_CHECK, GAPC_NO_OP};
    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);
    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_SMP, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        uint8_t status =  smpc_sign_command(idx, param);

        // check if request should be aborted
        if(status != GAP_ERR_NO_ERROR)
        {
            gapc_send_complete_evt(idx, GAPC_OP_SMP, status);
        }
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles data send response from L2CAP.
 *  this is used to know when an indication has been correctly sent
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] evt       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GATTC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int l2cc_cmp_evt_handler(kernel_msg_id_t const msgid, struct l2cc_cmp_evt *evt,
                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    #if (BLE_PERIPHERAL)
    // Recover connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        if (gapc_get_operation(idx, GAPC_OP_SMP) == GAPC_SECURITY_REQ)
        {
            // Security request performed
            gapc_send_complete_evt(idx, GAPC_OP_SMP, GAP_ERR_NO_ERROR);
        }
        #if (SECURE_CONNECTIONS)
        else if (gapc_get_operation(idx, GAPC_OP_LINK_INFO) == GAPC_KEY_PRESS_NOTIFICATION)
        {
            // Keypress notification
            gapc_send_complete_evt(idx, GAPC_OP_LINK_INFO, GAP_ERR_NO_ERROR);
        }
        #endif // (SECURE_CONNECTIONS)
    }
    #endif //(BLE_PERIPHERAL)

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Encryption Start request from HL (when LTK is known).
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_gen_rand_nb_ind_handler(kernel_msg_id_t const msgid,
                                        struct gapm_gen_rand_nb_ind *param,
                                        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Index
    uint8_t idx     = KERNEL_IDX_GET(dest_id);

    if ((kernel_state_get(dest_id) != GAPC_FREE)
            && (gapc_get_operation(idx, GAPC_OP_SMP) == GAPC_BOND))
    {
        smpc_confirm_gen_rand(idx, &(param->randnb));
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles Encryption Start request from HL (when LTK is known).
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_use_enc_block_ind_handler(kernel_msg_id_t const msgid,
                                          struct gapm_use_enc_block_ind *param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Index
    uint8_t idx     = KERNEL_IDX_GET(dest_id);

    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        switch(gapc_get_operation(idx, GAPC_OP_SMP))
        {
            case GAPC_BOND:
            {
                smpc_calc_confirm_cont(idx, param->result);
            } break;

            case GAPC_SIGN_PACKET:
            case GAPC_SIGN_CHECK:
            {
                smpc_sign_cont(idx, param->result);
            } break;

            default: /* Nothing to do */ break;
        }
    }
    return (KERNEL_MSG_CONSUMED);

}

#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles DH_KEY request
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_gen_dh_key_ind_handler(kernel_msg_id_t const msgid,
                                          struct gapm_gen_dh_key_ind *param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        smpc_handle_dh_key_check_complete(idx,param->result);
    }
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Keypress notification request
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_key_press_notification_cmd_handler(kernel_msg_id_t const msgid,
        struct gapc_key_press_notif_cmd *param,
        kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Index
    uint8_t conidx = KERNEL_IDX_GET(dest_id);
    int msg_status = KERNEL_MSG_CONSUMED;
    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        // list of handler supported operations
        enum gapc_operation supp_ops[] = {GAPC_KEY_PRESS_NOTIFICATION, GAPC_NO_OP};

        // check if operation can be executed
        msg_status = gapc_process_op(conidx, GAPC_OP_LINK_INFO, param, supp_ops);

        // Operation can be handled
        if(msg_status == KERNEL_MSG_NO_FREE)
        {
            smpc_pdu_send(conidx, L2C_CODE_KEYPRESS_NOTIFICATION, &param->notification_type);
            // Check if we are waiting the TK (Pairing procedure on going)
            if ((gapc_env[conidx]->smpc.state > SMPC_STATE_RESERVED) && (gapc_env[conidx]->smpc.state < SMPC_SIGN_L_GEN))
            {
                if( kernel_timer_active(GAPC_SMP_TIMEOUT_TIMER_IND, KERNEL_BUILD_ID(TASK_GAPC, conidx)))
                {
                    kernel_timer_set(GAPC_SMP_TIMEOUT_TIMER_IND, KERNEL_BUILD_ID(TASK_GAPC, conidx), SMPC_TIMEOUT_TIMER_DURATION);
                }
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
    }
    return msg_status;
}

#endif // (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles Encryption Start request from HL (when LTK is known).
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(kernel_msg_id_t const msgid,
                                struct gapm_cmp_evt *param,
                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        if (param->status != GAP_ERR_NO_ERROR)
        {
            // Index
            uint8_t idx         = KERNEL_IDX_GET(dest_id);
            switch(gapc_get_operation(idx, GAPC_OP_SMP))
            {
                case GAPC_BOND:
                {
                    smpc_pairing_end(idx, gapc_get_role(idx), param->status, false);
                } break;

                case GAPC_SIGN_PACKET:
                case GAPC_SIGN_CHECK:
                {
                    gapc_send_complete_evt(idx, GAPC_OP_SMP, param->status);
                } break;

                default: /* Nothing to do */ break;
            }
        }
    }

    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles Encryption Start request from HL (when LTK is known).
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_smp_timeout_timer_ind_handler(kernel_msg_id_t const msgid,
                                          void const *param,
                                          kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Recover connection index
    uint8_t idx    = KERNEL_IDX_GET(dest_id);

    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        // Disable the timer state in the environment
        SMPC_TIMER_UNSET_FLAG(idx, SMPC_TIMER_TIMEOUT_FLAG);
        // No more SM procedure may occur
        SMPC_TIMER_SET_FLAG(idx, SMPC_TIMER_TIMEOUT_BLOCKED_FLAG);

        // Inform the HL that an error has occurred.
        smpc_pairing_end(idx, gapc_get_role(idx), GAP_ERR_TIMEOUT, false);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Encryption Start request from HL (when LTK is known).
 * @param[in] msgid     Id of the message received.
 * @param[in] req       Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_smp_rep_attempts_timer_handler(kernel_msg_id_t const msgid,
                                           void const *param,
                                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    if (kernel_state_get(dest_id) != GAPC_FREE)
    {
        // Recover connection index
        uint8_t idx = KERNEL_IDX_GET(dest_id);

        ASSERT_ERR(SMPC_IS_FLAG_SET(idx, SMPC_TIMER_REP_ATT_FLAG));

        // Reset the timer value
        gapc_env[idx]->smpc.rep_att_timer_val = SMPC_REP_ATTEMPTS_TIMER_DEF_VAL;
        // Update the timer flag
        SMPC_TIMER_UNSET_FLAG(idx, SMPC_TIMER_REP_ATT_FLAG);
    }

    //message is consumed
    return (KERNEL_MSG_CONSUMED);
}


#if (BLE_ATTS)
/**
 ****************************************************************************************
 * @brief Receives indication from ATTS that reconnection address has been
 * modified by the central, and this non-resolvable address in the reconnection
 * address will be saved in the upper layer for reference in the future. This
 * address can be used to connect to a peer device (reconnection characteristic
 * is only one instance, so former central's reconnection address needs to be
 * saved somewhere for future usage.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_write_req_ind_handler(kernel_msg_id_t const msgid, struct gattc_write_req_ind const *param,
                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current state
    uint8_t state = kernel_state_get(dest_id);
    // Status
    uint8_t status;

    if(state != GAPC_FREE)
    {
        // Device name modified
        if ((param->handle == gapm_get_att_handle(GAP_IDX_DEVNAME)))
        {
            // only accept name request with a full value
            if(param->offset == 0)
            {
                // send an indication message to application to modify device name
                struct gapc_set_dev_info_req_ind *req_ind = KERNEL_MSG_ALLOC_DYN(GAPC_SET_DEV_INFO_REQ_IND,
                        APP_MAIN_TASK, dest_id, gapc_set_dev_info_req_ind, param->length);
                req_ind->req = GAPC_DEV_NAME;
                req_ind->info.name.length = param->length;
                memcpy(&(req_ind->info.name.value), param->value, param->length);
                /* send the indication */
                kernel_msg_send(req_ind);
            }
            // answer with insufficient authorization error
            else
            {
                // send back response to ATTS
                struct gattc_write_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
                cfm->handle = param->handle;
                cfm->status = ATT_ERR_INSUFF_AUTHOR;
                kernel_msg_send(cfm);
            }
        }
        // Appearance modified
        else if (param->handle == gapm_get_att_handle(GAP_IDX_ICON))
        {
            // sanity check
            if((param->offset == 0) && (param->length == sizeof(uint16_t)))
            {
                // send an indication message to application to modify device appearance
                struct gapc_set_dev_info_req_ind *req_ind = KERNEL_MSG_ALLOC(GAPC_SET_DEV_INFO_REQ_IND,
                        APP_MAIN_TASK, dest_id, gapc_set_dev_info_req_ind);
                req_ind->req = GAPC_DEV_APPEARANCE;
                req_ind->info.appearance = common_read16p(param->value) ;
                /* send the indication */
                kernel_msg_send(req_ind);
            }
            // answer with error
            else
            {
                // send back response to ATTS
                struct gattc_write_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
                cfm->handle = param->handle;
                cfm->status = ((param->length == sizeof(uint16_t))
                                        ? ATT_ERR_INSUFF_AUTHOR : ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN);
                kernel_msg_send(cfm);
            }
        }
        else
        {
            status = ATT_ERR_APP_ERROR;

            // send back response to ATTS
            struct gattc_write_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
            cfm->handle = param->handle;
            cfm->status = status;
            kernel_msg_send(cfm);
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Receive a request to get information about an attribute handle to write
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_att_info_req_ind_handler(kernel_msg_id_t const msgid, struct gattc_att_info_req_ind const *param,
                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPC_FREE)
    {
        uint16_t length = 0;
        uint8_t status = ATT_ERR_NO_ERROR;

        // Device name modified
        if (param->handle == gapm_get_att_handle(GAP_IDX_DEVNAME))
        {
            // Force value to zero in order to prevent peer device to write a name without starting from 0
            length = 0;
        }
        // Appearance modified
        else if (param->handle == gapm_get_att_handle(GAP_IDX_ICON))
        {
            length = sizeof(uint16_t);
        }
        // Address Resolution
        else if (param->handle == gapm_get_att_handle(GAP_IDX_CNT_ADDR_RESOL))
        {
            length = sizeof(uint8_t);
        }
        else
        {
            // Not supported
            status = ATT_ERR_APP_ERROR;
        }

        // send back response to ATTS
        struct gattc_att_info_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
        cfm->handle = param->handle;
        cfm->status = status;
        cfm->length = length;

        kernel_msg_send(cfm);
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Receive a request to get attribute value
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_read_req_ind_handler(kernel_msg_id_t const msgid, struct gattc_att_info_req_ind const *param,
                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPC_FREE)
    {
        uint8_t req = GAPC_DEV_INFO_MAX;

        #if (BLE_PERIPHERAL)
        // Slave preferred parameters
        if (gapm_is_pref_con_param_pres() && (param->handle == gapm_get_att_handle(GAP_IDX_SLAVE_PREF_PARAM)))
        {
            req = GAPC_DEV_SLV_PREF_PARAMS;
        }
        else
        #endif /* (BLE_PERIPHERAL) */
        // Device name
        if (param->handle == gapm_get_att_handle(GAP_IDX_DEVNAME))
        {
            req = GAPC_DEV_NAME;
        }
        // Appearance modified
        else if (param->handle == gapm_get_att_handle(GAP_IDX_ICON))
        {
            req = GAPC_DEV_APPEARANCE;
        }

        // check if application can provide value
        if(req != GAPC_DEV_INFO_MAX)
        {
            // send value request to application.
            struct gapc_get_dev_info_req_ind * req_ind = KERNEL_MSG_ALLOC(GAPC_GET_DEV_INFO_REQ_IND, APP_MAIN_TASK, dest_id, gapc_get_dev_info_req_ind);
            req_ind->req = req;
            kernel_msg_send(req_ind);
        }
        else
        {
            // Central Address Resolution supported
            if (param->handle == gapm_get_att_handle(GAP_IDX_CNT_ADDR_RESOL))
            {
                // send back address resolution to ATTS
                struct gattc_read_cfm * cnt_addr_resol_cfm = KERNEL_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                                    KERNEL_BUILD_ID(TASK_GATTC, KERNEL_IDX_GET(dest_id)), dest_id,
                                                    gattc_read_cfm, sizeof(uint8_t));
                cnt_addr_resol_cfm->handle   = gapm_get_att_handle(GAP_IDX_CNT_ADDR_RESOL);
                cnt_addr_resol_cfm->status   = ATT_ERR_NO_ERROR;
                cnt_addr_resol_cfm->length   = sizeof(uint8_t);
                cnt_addr_resol_cfm->value[0] = (gapm_get_address_type() == GAPM_CFG_ADDR_CTNL_PRIVACY) ? 1 : 0;

                kernel_msg_send(cnt_addr_resol_cfm);
            }
            else
            {
                // Inform peer device that an error occurs.
                struct gattc_read_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm);
                cfm->handle = param->handle;
                cfm->status = ATT_ERR_APP_ERROR;
                kernel_msg_send(cfm);
            }
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Receive device information from application
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_get_dev_info_cfm_handler(kernel_msg_id_t const msgid, struct gapc_get_dev_info_cfm const *param,
                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPC_FREE)
    {
        uint8_t conidx = KERNEL_IDX_GET(dest_id);
        // check provided value
        switch(param->req)
        {
            // device name provided
            case GAPC_DEV_NAME:
            {
                // send back device name to ATTS
                struct gattc_read_cfm * name_cfm = KERNEL_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                                    KERNEL_BUILD_ID(TASK_GATTC, conidx), dest_id,
                                                    gattc_read_cfm, param->info.name.length);
                name_cfm->handle = gapm_get_att_handle(GAP_IDX_DEVNAME);
                name_cfm->status = ATT_ERR_NO_ERROR;
                name_cfm->length = param->info.name.length;
                memcpy(name_cfm->value, param->info.name.value, name_cfm->length);
                kernel_msg_send(name_cfm);
            }
            break;
            case GAPC_DEV_APPEARANCE:
            {
                // send back device appearance to ATTS
                struct gattc_read_cfm * appearance_cfm = KERNEL_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                                    KERNEL_BUILD_ID(TASK_GATTC, conidx), dest_id,
                                                    gattc_read_cfm, sizeof(uint16_t));
                appearance_cfm->handle = gapm_get_att_handle(GAP_IDX_ICON);
                appearance_cfm->status = ATT_ERR_NO_ERROR;
                appearance_cfm->length = sizeof(uint16_t);
                common_write16p(appearance_cfm->value, param->info.appearance);

                kernel_msg_send(appearance_cfm);
            }
            break;

            #if (BLE_PERIPHERAL)
            case GAPC_DEV_SLV_PREF_PARAMS:
            {
                uint8_t cursor = 0;
                // send back device slave parameters to ATTS
                struct gattc_read_cfm * param_cfm = KERNEL_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                                    KERNEL_BUILD_ID(TASK_GATTC, conidx), dest_id,
                                                    gattc_read_cfm, sizeof(struct gap_slv_pref));
                param_cfm->handle = gapm_get_att_handle(GAP_IDX_SLAVE_PREF_PARAM);
                param_cfm->status = ATT_ERR_NO_ERROR;
                param_cfm->length = sizeof(struct gap_slv_pref);

                common_write16p(&param_cfm->value[cursor], param->info.slv_params.con_intv_min);
                cursor+= sizeof(uint16_t);
                common_write16p(&param_cfm->value[cursor], param->info.slv_params.con_intv_max);
                cursor+= sizeof(uint16_t);
                common_write16p(&param_cfm->value[cursor], param->info.slv_params.slave_latency);
                cursor+= sizeof(uint16_t);
                common_write16p(&param_cfm->value[cursor], param->info.slv_params.conn_timeout);

                kernel_msg_send(param_cfm);
            }
            break;
            #endif /* (BLE_PERIPHERAL) */
            default: /* Ignore Message */ break;
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Receive device information modification confirmation from application
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_cfm_handler(kernel_msg_id_t const msgid, struct gapc_set_dev_info_cfm const *param,
                           kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // Current state
    uint8_t state = kernel_state_get(dest_id);

    if(state != GAPC_FREE)
    {
        uint8_t conidx = KERNEL_IDX_GET(dest_id);
        uint16_t handle =  ATT_INVALID_HDL;

        // check provided value
        switch(param->req)
        {
            case GAPC_DEV_NAME:
            {
                handle = gapm_get_att_handle(GAP_IDX_DEVNAME);
            }
            break;
            case GAPC_DEV_APPEARANCE:
            {
                handle = gapm_get_att_handle(GAP_IDX_ICON);
            }
            break;
            default: /* Ignore Message */ break;
        }

        if(handle != ATT_INVALID_HDL)
        {
            // send back response to ATTS
            struct gattc_write_cfm * cfm = KERNEL_MSG_ALLOC(GATTC_WRITE_CFM, KERNEL_BUILD_ID(TASK_GATTC, conidx), dest_id, gattc_write_cfm);
            cfm->handle = handle;
            cfm->status = param->status;
            kernel_msg_send(cfm);
        }
    }

    /* message is consumed */
    return (KERNEL_MSG_CONSUMED);
}
#endif // (BLE_ATTS)


/**
 ****************************************************************************************
 * @brief Handles the reception of GAPC_LE_PING_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_le_ping_to_handler(kernel_msg_id_t const msgid,
                                struct gapc_set_le_ping_to_cmd *param,
                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_SET_LE_PING_TO, GAPC_NO_OP};

    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_LINK_INFO, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // send HCI command
        /* allocate read version message */
        struct hci_wr_auth_payl_to_cmd *write_to = KERNEL_MSG_ALLOC(HCI_COMMAND, idx, HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, hci_wr_auth_payl_to_cmd);

        write_to->conhdl       = gapc_get_conhdl(idx);
        write_to->auth_payl_to = param->timeout;

        /* send the message */
        hci_send_2_controller(write_to);
    }

    return msg_status;
}


static int gapc_cmp_evt_handler(kernel_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                kernel_task_id_t const dest_id,
                                kernel_task_id_t const src_id)
{
	// UART_PRINTF("gapctask gapc_cmp_evt_handler\r\n");
	 struct gapc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(GAPC_CMP_EVT, APP_MAIN_TASK,
                TASK_GAPC, gapc_cmp_evt);
        /* fill up the parameters */
        cmp_evt->operation = param->operation;
        cmp_evt->status = param->status;
	 
	 kernel_msg_send(cmp_evt);
	
	 return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of GAPC_SET_LE_PKT_SIZE_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_le_pkt_size_handler(kernel_msg_id_t const msgid,
                                struct gapc_set_le_pkt_size_cmd *param,
                                kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_SET_LE_PKT_SIZE, GAPC_NO_OP};

    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_LINK_INFO, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        // send HCI command
        struct hci_le_set_data_len_cmd *set_data = KERNEL_MSG_ALLOC(HCI_COMMAND, idx,
         HCI_LE_SET_DATA_LEN_CMD_OPCODE, hci_le_set_data_len_cmd);

        set_data->conhdl    = gapc_get_conhdl(idx);
        set_data->tx_octets = param->tx_octets;
        set_data->tx_time   = param->tx_time;

        /* send the message */
        hci_send_2_controller(set_data);
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles the reception of GAPC_SET_PHY_CMD message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_phy_cmd_handler(kernel_msg_id_t const msgid,
                                    struct  gapc_set_phy_cmd *param,
                                    kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapc_operation supp_ops[] = {GAPC_SET_PHY, GAPC_NO_OP};

    // Current connection index
    uint8_t idx = KERNEL_IDX_GET(dest_id);

    // check if operation can be executed
    int msg_status = gapc_process_op(idx, GAPC_OP_LINK_UPD, param, supp_ops);

    // Operation can be handled
    if(msg_status == KERNEL_MSG_NO_FREE)
    {
        #if (BLE_2MBPS)
        // send HCI command
        struct hci_le_set_phy_cmd *set_phy = KERNEL_MSG_ALLOC(HCI_COMMAND, idx,
                HCI_LE_SET_PHY_CMD_OPCODE, hci_le_set_phy_cmd);

        set_phy->conhdl    = gapc_get_conhdl(idx);

        // Fill data
        set_phy->all_phys  = (param->tx_rates == GAP_RATE_ANY) ? ALL_PHYS_TX_NO_PREF : ALL_PHYS_TX_PREF;
        set_phy->all_phys |= (param->rx_rates == GAP_RATE_ANY) ? ALL_PHYS_RX_NO_PREF : ALL_PHYS_RX_PREF;

        set_phy->rx_phys  = param->rx_rates;
        set_phy->tx_phys  = param->tx_rates;

        /* send the message */
        hci_send_2_controller(set_phy);

        #else
        gapc_send_complete_evt(idx, GAPC_OP_LINK_UPD, GAP_ERR_NOT_SUPPORTED);
        #endif // (BLE_2MBPS)
    }

    return msg_status;
}

// HCI handler is present into a dedicated module
extern int gapc_hci_handler(kernel_msg_id_t const msgid, void const* event, kernel_task_id_t dest_id, kernel_task_id_t src_id);


/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
const struct kernel_msg_handler gapc_default_state[] =
{
    /* Link management command */
    { GAPC_DISCONNECT_CMD,                         (kernel_msg_func_t) gapc_disconnect_cmd_handler },

    /* Set Bonding information */
    { GAPC_CONNECTION_CFM,                         (kernel_msg_func_t) gapc_connection_cfm_handler},
	{GAPC_CMP_EVT,									(kernel_msg_func_t) gapc_cmp_evt_handler }, // add by sean 2017.04.26 refer 3433
    /* Peer device info */
    { GAPC_GET_INFO_CMD,                           (kernel_msg_func_t) gapc_get_info_cmd_handler },
    // dev name
    { GATTC_READ_IND,                              (kernel_msg_func_t) gattc_read_ind_handler },
    { GATTC_CMP_EVT,                               (kernel_msg_func_t) gattc_cmp_evt_handler },

    // Update connection parameters
    { GAPC_PARAM_UPDATE_CMD,                       (kernel_msg_func_t) gapc_param_update_cmd_handler },
    { GAPC_PARAM_UPDATE_CFM,                       (kernel_msg_func_t) gapc_param_update_cfm_handler},
    { L2CC_PDU_RECV_IND,                           (kernel_msg_func_t) l2cc_pdu_recv_ind_handler },
    #if (BLE_PERIPHERAL)
    { GAPC_PARAM_UPDATE_TO_IND,                    (kernel_msg_func_t) gapc_update_conn_param_to_ind_handler },
    #endif // (BLE_PERIPHERAL)

    // Bonding procedure
    { GAPC_BOND_CMD,                               (kernel_msg_func_t) gapc_bond_cmd_handler},

    { GAPC_BOND_CFM,                               (kernel_msg_func_t) gapc_bond_cfm_handler},

    // Encryption procedure
    { GAPC_ENCRYPT_CMD,                            (kernel_msg_func_t) gapc_encrypt_cmd_handler },
    #if (BLE_PERIPHERAL)
    { GAPC_ENCRYPT_CFM,                            (kernel_msg_func_t) gapc_encrypt_cfm_handler },
    // Security Request procedure
    { GAPC_SECURITY_CMD,                           (kernel_msg_func_t) gapc_security_cmd_handler },
    #endif // (BLE_PERIPHERAL)

    { GAPC_SIGN_CMD,                               (kernel_msg_func_t) gapc_sign_cmd_handler},
    // Signature SignCounter Indication handler
    { L2CC_CMP_EVT,                                (kernel_msg_func_t) l2cc_cmp_evt_handler},
    { GAPM_GEN_RAND_NB_IND,                        (kernel_msg_func_t) gapm_gen_rand_nb_ind_handler},
    { GAPM_USE_ENC_BLOCK_IND,                      (kernel_msg_func_t) gapm_use_enc_block_ind_handler},
    { GAPM_CMP_EVT,                                (kernel_msg_func_t) gapm_cmp_evt_handler},

    // Pairing timers handler
    { GAPC_SMP_TIMEOUT_TIMER_IND,                  (kernel_msg_func_t) gapc_smp_timeout_timer_ind_handler},
    { GAPC_SMP_REP_ATTEMPTS_TIMER_IND,             (kernel_msg_func_t) gapc_smp_rep_attempts_timer_handler},
    #if (SECURE_CONNECTIONS)
    { GAPM_GEN_DH_KEY_IND,                         (kernel_msg_func_t) gapc_gen_dh_key_ind_handler},
    { GAPC_KEY_PRESS_NOTIFICATION_CMD,             (kernel_msg_func_t) gapc_key_press_notification_cmd_handler},
    #endif // (SECURE_CONNECTIONS)
    #if (BLE_ATTS)
    /* Generic Access DB Management */
    { GATTC_WRITE_REQ_IND,                         (kernel_msg_func_t) gattc_write_req_ind_handler },
    { GATTC_ATT_INFO_REQ_IND,                      (kernel_msg_func_t) gattc_att_info_req_ind_handler },
    { GATTC_READ_REQ_IND,                          (kernel_msg_func_t) gattc_read_req_ind_handler },
    { GAPC_GET_DEV_INFO_CFM,                       (kernel_msg_func_t) gapc_get_dev_info_cfm_handler },
    { GAPC_SET_DEV_INFO_CFM,                       (kernel_msg_func_t) gapc_set_dev_info_cfm_handler },
    #endif /* (BLE_ATTS) */

    { HCI_CMD_CMP_EVENT,                           (kernel_msg_func_t) gapc_hci_handler },
    { HCI_CMD_STAT_EVENT,                          (kernel_msg_func_t) gapc_hci_handler },
    { HCI_LE_EVENT,                                (kernel_msg_func_t) gapc_hci_handler },
    { HCI_EVENT,                                   (kernel_msg_func_t) gapc_hci_handler },

    // LE Ping
    { GAPC_SET_LE_PING_TO_CMD,                     (kernel_msg_func_t) gapc_set_le_ping_to_handler},

    // LE Data Length Extension
    { GAPC_SET_LE_PKT_SIZE_CMD,                    (kernel_msg_func_t) gapc_set_le_pkt_size_handler},

    // LE Phy configuration
    { GAPC_SET_PHY_CMD,                            (kernel_msg_func_t) gapc_set_phy_cmd_handler},
};


/// Message handlers that are common to all states.
const struct kernel_state_handler gapc_default_handler = KERNEL_STATE_HANDLER(gapc_default_state);

/// GAPC task instance.
kernel_state_t gapc_state[GAPC_IDX_MAX];


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} GAPC_TASK
