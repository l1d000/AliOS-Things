/**
 ****************************************************************************************
 *
 * @file gapm_util.c
 *
 * @brief Generic Access Profile Manager Tool Box Implementation
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_UTIL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#include "gap.h"

#include "gapm_int.h"
#include "gapm_task.h"
#include "gapm_util.h"

#include "common_math.h"
#include "kernel_mem.h"
#include "kernel_timer.h"
#include "gapc.h"

#include "attm.h"

#include "smpm_api.h" // Access to internal API required

#include "hci.h"

#include "l2cm.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */
/// Low energy mask
#define GAP_EVT_MASK                                       {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x00, 0x20}
#if (BLE_2MBPS)
#define GAP_LE_EVT_MASK                                    {0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#else
#define GAP_LE_EVT_MASK                                    {0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#endif // (BLE_2MBPS)
#define GAP_LE_EVT_4_0_MASK                                (0x1F)


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

static void gapm_update_address_op_state(struct gapm_air_operation* op, uint8_t state)
{
    // execute current operation state.
    switch(state)
    {
        case GAPM_OP_ADDR_GEN: // Address generation state
        {
            // Set generated random address
            GAPM_SET_OP_STATE(*op, GAPM_OP_ADDR_SET);
        }
        break;

        case GAPM_OP_ADDR_SET: // Set device random address
        {
            // next step, go to wait state
            GAPM_SET_OP_STATE(*op, GAPM_OP_START);
        }
        break;
        default:
        {
            if(op->addr_src == GAPM_STATIC_ADDR)
            {
                // next step, go to start operation state
                GAPM_SET_OP_STATE(*op, GAPM_OP_START);
                break;
            }
            // Generate address
            else
            {
                // address has to be renewed
                if((gapm_get_address_type() == GAPM_CFG_ADDR_HOST_PRIVACY)
                    && (GAPM_F_GET(gapm_env.cfg_flags, ADDR_RENEW)
                        // or address type need to be changed
                        || ((op->addr_src == GAPM_GEN_RSLV_ADDR)     && !GAPM_F_GET(gapm_env.cfg_flags, RESOLV_ADDR))
                        || ((op->addr_src == GAPM_GEN_NON_RSLV_ADDR) &&  GAPM_F_GET(gapm_env.cfg_flags, RESOLV_ADDR))))
                {
                    // clear address renewal field
                    GAPM_CLEAR_OP_FIELD(*op, ADDR_RENEW);
                    GAPM_F_SET(gapm_env.cfg_flags, ADDR_RENEW, 0);
                    // go to update device address state
                    GAPM_SET_OP_STATE(*op, GAPM_OP_ADDR_GEN);
                    GAPM_F_SET(gapm_env.cfg_flags, RESOLV_ADDR, (op->addr_src == GAPM_GEN_RSLV_ADDR));
                }
                else
                {
                    // next step, go to start operation state
                    GAPM_SET_OP_STATE(*op, GAPM_OP_START);
                }
            }
        }
        break;
    }
}

/**
 ****************************************************************************************
 * @brief Set a random address operation.
 *
 * This function is used by air operations to modify random address in lower layer before
 * starting any air operations.
 *
 *  - If a public address is configured, operation state is set to @ref GAPM_OP_WAIT.
 *
 *  - If a random address should be generated, @ref GAPM_GEN_RAND_ADDR_CMD will be sent to
 *    @ref TASK_GAPM to request random address generation.
 *
 *  - When random address is generated, or provided by operation parameters, address is
 *    set to lower layers.
 *
 * @param op Operation parameters
 *
 ****************************************************************************************
 */
static void gapm_set_address_op(struct gapm_air_operation* op)
{
    // execute current operation state.
    switch(GAPM_GET_OP_STATE(*op))
    {
        case GAPM_OP_ADDR_GEN: // Address generation state
        {
            struct gapm_gen_rand_addr_cmd * req = KERNEL_MSG_ALLOC(GAPM_GEN_RAND_ADDR_CMD,
                    TASK_GAPM, TASK_GAPM, gapm_gen_rand_addr_cmd);

            req->operation = GAPM_GEN_RAND_ADDR;

            // address type to generate
            switch(op->addr_src)
            {
                // Private resolvable address
                case GAPM_GEN_RSLV_ADDR:        req->rnd_type = GAP_RSLV_ADDR; break;
                // Private non resolvable address
                case GAPM_GEN_NON_RSLV_ADDR:    req->rnd_type = GAP_NON_RSLV_ADDR; break;
                // cannot append
                default: break;
            }

            // send request
            kernel_msg_send(req);
        }
        break;

        case GAPM_OP_ADDR_SET: // Set device random address
        {
            // update lower layer random address.
            struct hci_le_set_rand_addr_cmd *rand_addr = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_RAND_ADDR_CMD_OPCODE, hci_le_set_rand_addr_cmd);

            /* random address is in operation address (put during address generation
             * or provided by host application
             */
            memcpy(&(rand_addr->rand_addr), &(gapm_env.addr), sizeof(bd_addr_t));

            // send command
            hci_send_2_controller(rand_addr);

            // start resolvable/ non resolvable address timer to regenerate address if timeout occurs.
            // minimum duration: GAP_TMR_PRIV_ADDR_INT
            kernel_timer_set(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM, common_max(gapm_env.renew_dur, GAP_TMR_PRIV_ADDR_INT));
            // clear address renew flag
            GAPM_F_SET(gapm_env.cfg_flags, ADDR_RENEW, 0);
        }
        break;
        default:
        {
            // should never happen
            ASSERT_ERR(0);
        }
        break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_PERIPHERAL || BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Verify if advertising data type is unique
 *
 * @param[in] adv_type  Type of advertising data
 *
 * @return True if unique, False else
 ****************************************************************************************
 */
static bool gapm_is_advtype_unique(uint8_t type)
{
    // advertising type check which shall be unique
    switch(type)
    {
        case GAP_AD_TYPE_MORE_16_BIT_UUID:           case GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID:
        case GAP_AD_TYPE_MORE_32_BIT_UUID:           case GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID:
        case GAP_AD_TYPE_MORE_128_BIT_UUID:          case GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID:
        case GAP_AD_TYPE_SHORTENED_NAME:             case GAP_AD_TYPE_COMPLETE_NAME:
        case GAP_AD_TYPE_APPEARANCE:                 case GAP_AD_TYPE_ADV_INTV:
        case GAP_AD_TYPE_PUB_TGT_ADDR:               case GAP_AD_TYPE_RAND_TGT_ADDR:
        case GAP_AD_TYPE_LE_BT_ADDR:                 case GAP_AD_TYPE_LE_ROLE:
        case GAP_AD_TYPE_FLAGS:

        return true;

        default: return false;
    }
}


/**
 ****************************************************************************************
 * @brief Perform an advertising data sanity check
 *
 * @param[in] adv_data           Advertising data
 * @param[in] adv_data_len       Advertising data length
 * @param[in] scan_rsp_data      Scan response data
 * @param[in] scan_rsp_data_len  Scan response data length
 *
 * @return GAP_ERR_NO_ERROR if valid, GAP_ERR_ADV_DATA_INVALID if not valid
 ****************************************************************************************
 */
static uint8_t gapm_adv_sanity(uint8_t *adv_data, uint8_t adv_data_len,
                               uint8_t *scan_rsp_data, uint8_t scan_rsp_data_len)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t data_type_cursor = 0;

    // check for duplicate information in advertising or scan response data.
//    uint8_t dup_filter[(GAP_ADV_DATA_LEN * 2) / 3];
//    dup_filter[0] = GAP_AD_TYPE_FLAGS;
//    uint8_t dup_filt_cursor = 1;

    while((data_type_cursor < 2) && (status == GAP_ERR_NO_ERROR))
    {
       // uint8_t cursor = 0;
        //uint8_t* data;
        uint8_t length;
        uint8_t max_length;

        // check adv_data
        if(data_type_cursor == 0)
        {
            //data =       adv_data;
            length =     adv_data_len;
            max_length = GAP_ADV_DATA_LEN;
        }
        // check scan_rsp_data
        else
        {
            //data =       scan_rsp_data;
            length =     scan_rsp_data_len;
            max_length = GAP_SCAN_RSP_DATA_LEN;
        }

        if(length > max_length)
        {
            status = GAP_ERR_INVALID_PARAM;
        }

        // check next data
        data_type_cursor++;
    }
    return (status);
}

uint8_t gapm_adv_op_sanity(void)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    // retrieve advertising parameters
    struct gapm_start_advertise_cmd *adv =
            (struct gapm_start_advertise_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    do
    {
        uint8_t supp_role = ((adv->op.code != GAPM_ADV_NON_CONN)
                ? GAP_ROLE_PERIPHERAL: GAP_ROLE_BROADCASTER);

        // can advertise only if there's no ongoing connection
        if(
                #if (BLE_PERIPHERAL)
                (gapm_env.connections >= BLE_CONNECTION_MAX) &&
                #endif
                (adv->op.code != GAPM_ADV_NON_CONN)
                && (adv->info.host.mode != GAP_BROADCASTER_MODE))
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // check if this operation supported by current role.
        if(!GAPM_IS_ROLE_SUPPORTED(supp_role))
        {
            // role not supported
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        //check operation
        switch(adv->op.code)
        {
            #if (BLE_PERIPHERAL)
            // only advertising is allowed
            case GAPM_ADV_UNDIRECT:
            {
                if(adv->info.host.mode >= GAP_BROADCASTER_MODE)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

            }
            // no break
            #endif // (BLE_PERIPHERAL)
            case GAPM_ADV_NON_CONN:
            {
                // ADV mode sanity check
                if(adv->info.host.mode > GAP_BROADCASTER_MODE)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Filter policy sanity check
                if(adv->info.host.adv_filt_policy >= ADV_ALLOW_SCAN_END)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Filter policy sanity check - cont
                //
                // While a device is in limited/general discoverable mode the Host configures the Controller as follows:
                // - The Host shall set the advertising filter policy to "process scan and
                //   connection requests from all devices".
                if((adv->info.host.adv_filt_policy  != ADV_ALLOW_SCAN_ANY_CON_ANY)
                        && ((adv->info.host.mode == GAP_GEN_DISCOVERABLE)
                                || (adv->info.host.mode == GAP_LIM_DISCOVERABLE)))
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // check advertising mode
                #if (BLE_PERIPHERAL)
                if(!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_PERIPHERAL))
                #endif // (BLE_PERIPHERAL)
                {
                    // While a device is in the Broadcaster, Observer or Central role
                    // the device shall not support the general or limited discoverable mode
                    if((adv->info.host.mode == GAP_GEN_DISCOVERABLE)
                            || (adv->info.host.mode == GAP_LIM_DISCOVERABLE))
                    {
                        status = GAP_ERR_NOT_SUPPORTED;
                        break;
                    }
                }

                // perform sanity check of advertising data and scan response data.
                status = gapm_adv_sanity(adv->info.host.adv_data,
                                         adv->info.host.adv_data_len,
                                         adv->info.host.scan_rsp_data,
                                         adv->info.host.scan_rsp_data_len);
            }
            // no break

            #if (BLE_PERIPHERAL)
            case GAPM_ADV_DIRECT:
            case GAPM_ADV_DIRECT_LDC:
            #endif // (BLE_PERIPHERAL)
            {
                // Check Privacy
                if(GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) == GAPM_CFG_ADDR_HOST_PRIVACY)
                {
                    if(((adv->op.code != GAPM_ADV_NON_CONN) && (adv->op.addr_src == GAPM_GEN_NON_RSLV_ADDR))
                        ||  (adv->op.addr_src == GAPM_STATIC_ADDR))
                    {
                        status = GAP_ERR_PRIVACY_CFG_PB;
                    }
                }
                else if(adv->op.addr_src != GAPM_STATIC_ADDR)
                {
                    status = GAP_ERR_PRIVACY_CFG_PB;
                }
            }
            break;

            default:
            {
                /* send command complete event with error */
                status = GAP_ERR_INVALID_PARAM;
            }
            break;
        };
    } while(0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Manage state of advertising operation
 *
 * @param[in|out] adv  Advertising operation message pointer
 * @param[in] state    Status event that trigger modification of operation state
 ****************************************************************************************
 */
static void gapm_update_adv_op_state(struct gapm_start_advertise_cmd *adv, uint8_t state)
{
    // Update operation states with state
    switch(state)
    {
        // Initialize operation
        case GAPM_OP_INIT:
        {
            // First set scanning parameters
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_SET_PARAMS);
        }
        break;

        // entry point of scan operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            if((adv->op.code == GAPM_ADV_NON_CONN) || (adv->op.code == GAPM_ADV_UNDIRECT))
            {
                // change operation state to set adv data
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_SET_ADV_DATA);
            }
            else
            {
                // let address management to update state
                gapm_update_address_op_state(&(adv->op), state);
            }
        }
        break;

        // Set operation advertise data
        case GAPM_OP_SET_ADV_DATA:
        {
            // change operation state to set scan data
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_SET_SCAN_RSP_DATA);
        }
        break;
        // Set operation scan response data
        case GAPM_OP_SET_SCAN_RSP_DATA:
        {
            // let address management to update state
            gapm_update_address_op_state(&(adv->op), state);
        }
        break;

        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // let address management to update state
            gapm_update_address_op_state(&(adv->op), state);
        }
        break;

        // no break
        case GAPM_OP_START:
        {
            if((GAPM_IS_OP_FIELD_SET(adv->op, ADDR_RENEW))
                    || (GAPM_IS_OP_FIELD_SET(adv->op, CANCELED))
                    || (GAPM_IS_OP_FIELD_SET(adv->op, TIMEOUT)))
            {
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_STOP);
            }
            else
            {
                // next step is wait state
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_WAIT);
            }
        }
        break;
        case GAPM_OP_STOP:
        {
            if((GAPM_IS_OP_FIELD_SET(adv->op, CANCELED))
                || (GAPM_IS_OP_FIELD_SET(adv->op, TIMEOUT)))
            {
                // operation is finished
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_FINISH);
            }
            else if(GAPM_IS_OP_FIELD_SET(adv->op, ADDR_RENEW))
            {
                // let address management to update state
                gapm_update_address_op_state(&(adv->op), state);
            }
            else
            {
                // else return to start state
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_START);
            }
        }
        break;
        // Operation is in canceled state and shall be terminated.
        case GAPM_OP_CANCEL:
        {
            // Cancel operation
            GAPM_SET_OP_FIELD(adv->op, CANCELED);

            if(GAPM_GET_OP_STATE(adv->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_STOP);
            }
        }
        break;
        // Operation is in canceled state and shall be terminated.
        case GAPM_OP_TIMEOUT:
        {
            // Cancel operation
            GAPM_SET_OP_FIELD(adv->op, TIMEOUT);

            if(GAPM_GET_OP_STATE(adv->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_STOP);
            }
        }
        break;
        // Operation timeout and already terminated.
        case GAPM_OP_TERM_TIMEOUT:
        {
            // operation timeout
            GAPM_SET_OP_FIELD(adv->op, TIMEOUT);
            // nothing more to do.
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_FINISH);
        }
        break;

        // Renew address generation
        case GAPM_OP_ADDR_RENEW:
        {
            // set Connecting state
            GAPM_SET_OP_FIELD(adv->op, ADDR_RENEW);

            // if waiting state
            if(GAPM_GET_OP_STATE(adv->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(adv->op, GAPM_OP_STOP);
            }
        }
        break;
        case GAPM_OP_CONNECT:
        {
            // operation is finished
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_FINISH);
        }
        break;
        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_ERROR);
        }
        break;
    }
}

void gapm_execute_adv_op(void)
{
    // retrieve advertising parameters
    struct gapm_start_advertise_cmd *adv =
            (struct gapm_start_advertise_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    // clear message in kernel queue
    GAPM_CLEAR_OP_FIELD(adv->op, QUEUED);

    // execute current operation state.
    switch(GAPM_GET_OP_STATE(adv->op))
    {
        // entry point of advertising operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            // First, Advertising parameters shall be set in lower layers.
            struct hci_le_set_adv_param_cmd *adv_par = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_ADV_PARAM_CMD_OPCODE, hci_le_set_adv_param_cmd);

            // Get own address type
            uint8_t addr_type = gapm_get_local_addrtype();

            // set provided parameters
            adv_par->own_addr_type = addr_type;
            adv_par->adv_chnl_map  = adv->channel_map;
            adv_par->adv_intv_max  = adv->intv_max;
            adv_par->adv_intv_min  = adv->intv_min;

            switch(adv->op.code)
            {
                case GAPM_ADV_NON_CONN:
                {
                    if(adv->info.host.scan_rsp_data_len == 0)
                    {
                        // Advertising without scan response data (ADV_NON_CONN_IND)
                        adv_par->adv_type = ADV_NONCONN_UNDIR;
                    }
                    else
                    {
                        // Advertising with scan response (ADV_SCAN_IND)
                        adv_par->adv_type = ADV_DISC_UNDIR;
                    }
                    // set filter policy
                    adv_par->adv_filt_policy = adv->info.host.adv_filt_policy;
                    // Copy address
                    if(addr_type >= ADDR_RPA_OR_PUBLIC)
                    {
                        // set direct address to value
                        adv_par->peer_addr_type = adv->info.host.peer_addr.addr_type;
                        memcpy(&adv_par->peer_addr, &adv->info.host.peer_addr.addr, BD_ADDR_LEN);
                    }
                    else
                    {
                        // set direct address to default value
                        adv_par->peer_addr_type = ADDR_RAND;
                        memset(&adv_par->peer_addr, 0, BD_ADDR_LEN);
                    }
                }
                break;
                case GAPM_ADV_UNDIRECT:
                {
                    // Advertising connectable (ADV_IND)
                    adv_par->adv_type = ADV_CONN_UNDIR;
                    // set filter policy
                    adv_par->adv_filt_policy = adv->info.host.adv_filt_policy;
                    // Copy address
                    if(addr_type >= ADDR_RPA_OR_PUBLIC)
                    {
                        // set direct address to value
                        adv_par->peer_addr_type = adv->info.host.peer_addr.addr_type;
                        memcpy(&adv_par->peer_addr, &adv->info.host.peer_addr.addr, BD_ADDR_LEN);
                    }
                    else
                    {
                        // set direct address to default value
                        adv_par->peer_addr_type = ADDR_RAND;
                        memset(&adv_par->peer_addr, 0, BD_ADDR_LEN);
                    }
                }
                break;
                case GAPM_ADV_DIRECT:
                case GAPM_ADV_DIRECT_LDC:
                {
                    // Direct connectable advertising (ADV_DIRECT_IND)
                    adv_par->adv_type = ((adv->op.code == GAPM_ADV_DIRECT_LDC)
                            ? ADV_CONN_DIR_LDC : ADV_CONN_DIR);

                    // Set advertising policy
                    adv_par->adv_filt_policy  = ADV_ALLOW_SCAN_ANY_CON_ANY;

                    // set initiator address
                    adv_par->peer_addr_type = adv->info.direct.addr_type;
                    memcpy(&adv_par->peer_addr, &adv->info.direct.addr, BD_ADDR_LEN);
                }
                break;
                default: ASSERT_ERR(0); break; // not allowed
            }

            /* send the message */
            hci_send_2_controller(adv_par);
        }
        break;
        case GAPM_OP_SET_ADV_DATA:
        {
            // Set advertising data
            gapm_set_adv_data(adv->info.host.adv_data_len,  adv->info.host.adv_data);
        }
        break;
        case GAPM_OP_SET_SCAN_RSP_DATA:
        {
            struct hci_le_set_scan_rsp_data_cmd *scan_resp = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE, hci_le_set_scan_rsp_data_cmd);
            // retrieve scan response data length
            scan_resp->scan_rsp_data_len =adv->info.host.scan_rsp_data_len;
            // copy provided scan response data
            memcpy(&(scan_resp->data), adv->info.host.scan_rsp_data, GAP_SCAN_RSP_DATA_LEN);

            hci_send_2_controller(scan_resp);
        }
        break;
        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // Use address management toolbox to set address
            gapm_set_address_op(&adv->op);
        }
        break;
        case GAPM_OP_START: // Start advertising
        {
            // start a timer in limited discoverable mode
            if(adv->info.host.mode== GAP_LIM_DISCOVERABLE)
            {
                kernel_timer_set(GAPM_LIM_DISC_TO_IND, TASK_GAPM, GAP_TMR_LIM_ADV_TIMEOUT);
            }

            // start advertising mode
            gapm_set_adv_mode(true);
        }
        break;

        case GAPM_OP_STOP: // Stop
        {
            gapm_set_adv_mode(false);
        }
        break;

        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(adv->op, GAPM_OP_ERROR);
        }
        break;
    }

}

void gapm_set_adv_data(uint8_t length,  uint8_t* data)
{
    // retrieve command
    struct gapm_start_advertise_cmd *adv =
            (struct gapm_start_advertise_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    // Set advertising data
    struct hci_le_set_adv_data_cmd *adv_data = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_ADV_DATA_CMD_OPCODE, hci_le_set_adv_data_cmd);

#if 0
    // Set data lengh (with added AD_TYPE Flag)
    adv_data->adv_data_len = length + 3;
    // copy provided advertising data
    memcpy(&(adv_data->data.data[3]), data, GAP_ADV_DATA_LEN-3);
    // Set ad type flags.
    adv_data->data.data[0] = 2;// Length of ad type flags
    adv_data->data.data[1] = GAP_AD_TYPE_FLAGS;
    adv_data->data.data[2] = GAP_BR_EDR_NOT_SUPPORTED;
    // set mode in ad_type
    switch(adv->info.host.mode)
    {
        // General discoverable mode
        case GAP_GEN_DISCOVERABLE:
        {
            adv_data->data.data[2] |= GAP_LE_GEN_DISCOVERABLE_FLG;
        }
        break;
        // Limited discoverable mode
        case GAP_LIM_DISCOVERABLE:
        {
            adv_data->data.data[2] |= GAP_LE_LIM_DISCOVERABLE_FLG;
        }
        break;
        default: break; // do nothing
    }
#else
 	adv_data->adv_data_len = length ;
    // copy provided advertising data
      memcpy(&(adv_data->data.data[0]), data, GAP_ADV_DATA_LEN);


#endif //

    /* send the data */
    hci_send_2_controller(adv_data);
}

#endif // (BLE_PERIPHERAL || BLE_BROADCASTER)


#if (BLE_OBSERVER || BLE_CENTRAL)

uint8_t gapm_scan_op_sanity(void)
{
    // retrieve scan parameters
    struct gapm_start_scan_cmd *scan =
            (struct gapm_start_scan_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    uint8_t status = GAP_ERR_NO_ERROR;

    do
    {
        // check if this operation supported by current role.
        if(!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_OBSERVER))
        {
            // role not supported
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }

        //check operation
        switch(scan->op.code)
        {
            // only active and passive scan are allowed
            case GAPM_SCAN_ACTIVE:
                if(((GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) == GAPM_CFG_ADDR_HOST_PRIVACY) && (scan->op.addr_src == GAPM_STATIC_ADDR))
                    || ((GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) != GAPM_CFG_ADDR_HOST_PRIVACY) && (scan->op.addr_src > GAPM_STATIC_ADDR)))
                {
                    status = GAP_ERR_PRIVACY_CFG_PB;
                    break;
                }
                // no break
            case GAPM_SCAN_PASSIVE:
            {
                // sanity check address (reconnection address supported only for ADV_DIRECT)
                if(scan->op.addr_src > GAPM_GEN_NON_RSLV_ADDR)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Scan mode sanity check
                if(scan->mode >= GAP_INVALID_MODE)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Filter policy sanity check
                if(scan->filt_policy >= SCAN_ALLOW_ADV_END)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Filter policy sanity check - cont
                //
                // While a device is in limited/general discoverable mode the Host configures the Controller as follows:
                // - The Host shall set the advertising filter policy to "process scan and
                //   connection requests from all devices".
                if((scan->filt_policy == SCAN_ALLOW_ADV_WLST)
                        && ((scan->mode == GAP_GEN_DISCOVERY)
                                || (scan->mode == GAP_LIM_DISCOVERY)))
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Filter duplicate sanity check
                if(scan->filter_duplic >= SCAN_FILT_DUPLIC_END)
                {
                    status = GAP_ERR_INVALID_PARAM;
                    break;
                }
            }
            break;

            default:
            {
                /* send command complete event with error */
                status = GAP_ERR_INVALID_PARAM;
            }
            break;
        }
    } while(0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Manage state of scanning operation
 *
 * @param[in|out] scan  Scanning operation message pointer
 * @param[in] state     Status event that trigger modification of operation state
 ****************************************************************************************
 */
static void gapm_update_scan_op_state(struct gapm_start_scan_cmd *scan, uint8_t state)
{
    // Update operation states with state
    switch(state)
    {
        // Initialize operation
        case GAPM_OP_INIT:
        {
            // First set scanning parameters
            GAPM_SET_OP_STATE(scan->op, GAPM_OP_SET_PARAMS);
        }
        break;

        // entry point of scan operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            // let address management to update state
            gapm_update_address_op_state(&(scan->op), state);
        }
        break;
        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // let address management to update state
            gapm_update_address_op_state(&(scan->op), state);
        }
        break;

        // no break
        case GAPM_OP_START:
        {
            if((GAPM_IS_OP_FIELD_SET(scan->op, ADDR_RENEW))
                    || (GAPM_IS_OP_FIELD_SET(scan->op, CANCELED))
                    || (GAPM_IS_OP_FIELD_SET(scan->op, TIMEOUT)))
            {
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_STOP);
            }
            else
            {
                // next step is wait state
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_WAIT);
            }
        }
        break;
        case GAPM_OP_STOP:
        {
            if((GAPM_IS_OP_FIELD_SET(scan->op, CANCELED))
                || (GAPM_IS_OP_FIELD_SET(scan->op, TIMEOUT)))
            {
                // operation is finished
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_FINISH);
            }
            else if(GAPM_IS_OP_FIELD_SET(scan->op, ADDR_RENEW))
            {
                // let address management to update state
                gapm_update_address_op_state(&(scan->op), state);
            }
            else
            {
                // else return to start state
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_START);
            }
        }
        break;
        // Operation is in canceled state and shall be terminated.
        case GAPM_OP_CANCEL:
        {
            // Cancel operation
            GAPM_SET_OP_FIELD(scan->op, CANCELED);

            if(GAPM_GET_OP_STATE(scan->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_STOP);
            }
        }
        break;
        // Operation is in canceled state and shall be terminated.
        case GAPM_OP_TIMEOUT:
        {
            // Cancel operation
            GAPM_SET_OP_FIELD(scan->op, TIMEOUT);

            if(GAPM_GET_OP_STATE(scan->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_STOP);
            }
        }
        break;
        // Renew address generation
        case GAPM_OP_ADDR_RENEW:
        {
            // set Connecting state
            GAPM_SET_OP_FIELD(scan->op, ADDR_RENEW);

            // if waiting state
            if(GAPM_GET_OP_STATE(scan->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(scan->op, GAPM_OP_STOP);
            }
        }
        break;
        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(scan->op, GAPM_OP_ERROR);
        }
        break;
    }
}

void gapm_execute_scan_op(void)
{
    // retrieve scan parameters
    struct gapm_start_scan_cmd *scan =
            (struct gapm_start_scan_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    // clear message in kernel queue
    GAPM_CLEAR_OP_FIELD(scan->op, QUEUED);

    // execute current operation state.
    switch(GAPM_GET_OP_STATE(scan->op))
    {
        // entry point of scan operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            struct hci_le_set_scan_param_cmd *scan_param = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_SCAN_PARAM_CMD_OPCODE, hci_le_set_scan_param_cmd);

            /* fill up the parameters */
            scan_param->own_addr_type = gapm_get_local_addrtype();
            scan_param->scan_filt_policy = scan->filt_policy;

            scan_param->scan_intv = scan->interval;
            scan_param->scan_window = scan->window;
            scan_param->scan_type = ((scan->op.code == GAPM_SCAN_ACTIVE)
                    ? (SCAN_BLE_ACTIVE) : (SCAN_BLE_PASSIVE));

            scan_param->channel_map = scan->channel_map;

            /* send the message */
            hci_send_2_controller(scan_param);
        }
        break;
        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // Use address management toolbox to set address
            gapm_set_address_op(&scan->op);
        }
        break;
        case GAPM_OP_START: // start scanning
        {
            switch (scan->mode) {
                case GAP_GEN_DISCOVERY:
                {
                    // start a timer in general discovery mode
                    kernel_timer_set(GAPM_SCAN_TO_IND, TASK_GAPM, GAP_TMR_GEN_DISC_SCAN);
                }
                break;
                case GAP_LIM_DISCOVERY:
                {
                    // start a timer in limited discovery mode
                    kernel_timer_set(GAPM_SCAN_TO_IND, TASK_GAPM, GAP_TMR_LIM_DISC_SCAN);
                }
                break;
                default: /* Nothing to do */ break;
            }

            // start scanning mode
            gapm_set_scan_mode(true, scan->filter_duplic);
        }
        break;
        case GAPM_OP_STOP: // wait state
        {
            // stop scanning mode
            gapm_set_scan_mode(false, scan->filter_duplic);
        }
        break;
        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(scan->op, GAPM_OP_ERROR);
        }
        break;
    }

}
#endif // (BLE_OBSERVER || BLE_CENTRAL)


#if (BLE_CENTRAL)


uint8_t gapm_connect_op_sanity(void)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    struct gapm_start_connection_cmd *connect =
            (struct gapm_start_connection_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    do
    {
        // Not possible to establish a connection if max number of connection already established
        if(gapm_env.connections >= BLE_CONNECTION_MAX)
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // check if this operation supported by current role.
        if(!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
        {
            // role not supported
            status = GAP_ERR_NOT_SUPPORTED;
            break;
        }
        
        // sanity check address (reconnection address supported only for ADV_DIRECT)
        if(connect->op.addr_src > GAPM_GEN_NON_RSLV_ADDR)
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Check that some peer devices are set.
        if((connect->nb_peers == 0) && (connect->op.code != GAPM_CONNECTION_GENERAL))
        {
            status = GAP_ERR_INVALID_PARAM;
            break;
        }

        
        // sanity check address type according to privacy feature
        if(((GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) == GAPM_CFG_ADDR_HOST_PRIVACY) && (connect->op.addr_src != GAPM_GEN_RSLV_ADDR))
                || ((GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) != GAPM_CFG_ADDR_HOST_PRIVACY) && (connect->op.addr_src != GAPM_STATIC_ADDR)))
        {
            status = GAP_ERR_PRIVACY_CFG_PB;
            break;
        }

        //check operation
        switch(connect->op.code)
        {
            // only active and passive scan are allowed
            case GAPM_CONNECTION_GENERAL:
            case GAPM_CONNECTION_AUTO:
            case GAPM_CONNECTION_DIRECT:
            case GAPM_CONNECTION_SELECTIVE:
            case GAPM_CONNECTION_NAME_REQUEST:
            {
                // Nothing to check
            }
            break;

            default:
            {
                /* send command complete event with error */
                status = GAP_ERR_INVALID_PARAM;
            }
            break;
        }
    } while(0);


    // check if commands does not contains any peer (possible in general connection mode)
    if((status == GAP_ERR_NO_ERROR) && (connect->nb_peers == 0))
    {
        struct kernel_msg* src_msg = kernel_param2msg(connect);
        // ensure that at least one bd address is present in memory (will be used later)
        struct gapm_start_connection_cmd* new_cmd = KERNEL_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD,
                                                                     src_msg->dest_id, src_msg->src_id,
                                                                     gapm_start_connection_cmd, sizeof(struct gap_bdaddr));

        // copy parameter length
        memcpy(new_cmd, connect, src_msg->param_len);

        // force new command pointer
        gapm_set_operation_ptr(GAPM_OP_AIR, new_cmd);
        kernel_msg_free(src_msg);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Manage state of connection establishment operation
 *
 * @param[in|out] scan  connection operation message pointer
 * @param[in] state     Status event that trigger modification of operation state
 ****************************************************************************************
 */
static void gapm_update_connect_op_state(struct gapm_start_connection_cmd *connect, uint8_t state)
{
    // Update operation states with state
    switch(state)
    {
        // Initialize operation
        case GAPM_OP_INIT:
        {
            //check operation
            switch(connect->op.code)
            {
                // only active and passive scan are allowed
                case GAPM_CONNECTION_AUTO:
                case GAPM_CONNECTION_SELECTIVE:
                {
                    // First clear current white list
                    GAPM_SET_OP_STATE(connect->op, GAPM_OP_CLEAR_WL);
                }
                break;
                case GAPM_CONNECTION_GENERAL:
                {
                    // First, set scan parameters
                    GAPM_SET_OP_STATE(connect->op, GAPM_OP_SET_PARAMS);
                }
                break;
                case GAPM_CONNECTION_DIRECT:
                case GAPM_CONNECTION_NAME_REQUEST:
                {
                    // let address management to update state
                    gapm_update_address_op_state(&(connect->op), state);
                    // start a connection
                    GAPM_SET_OP_FIELD(connect->op, CONNECTING);
                }
                break;
                default: /* nothing to do */ break;
            } /* end of switch */
        }
        break;

        // request to clear white list.
        case GAPM_OP_CLEAR_WL:
        {
            // next step is set white list
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_SET_WL);
        }
        break;

        // set white list
        case GAPM_OP_SET_WL:
        {
            if(connect->op.code == GAPM_CONNECTION_AUTO)
            {
                // let address management to update state
                gapm_update_address_op_state(&(connect->op), state);
                // start a connection
                GAPM_SET_OP_FIELD(connect->op, CONNECTING);
            }
            else
            {
                // next step is set scan parameters
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_SET_PARAMS);
            }
        }
        break;

        // entry point of scan operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            // let address management to update state
            gapm_update_address_op_state(&(connect->op), state);
        }
        break;
        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // let address management to update state
            gapm_update_address_op_state(&(connect->op), state);
        }
        break;

        // no break
        case GAPM_OP_START:
        {
            if((GAPM_IS_OP_FIELD_SET(connect->op, ADDR_RENEW))
                    || (GAPM_IS_OP_FIELD_SET(connect->op, CANCELED))
                    || ((GAPM_IS_OP_FIELD_SET(connect->op, CONNECTING))
                            && (GAPM_IS_OP_FIELD_SET(connect->op, SCANNING))))
            {
                // first stop current mode
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_STOP);
            }
            else
            {
                // next step is wait state
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_WAIT);
            }
        }
        break;
        case GAPM_OP_CONNECT:
        {
            if(connect->op.code == GAPM_CONNECTION_NAME_REQUEST)
            {
                // next step is name request
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_NAME_REQ);
            }
            else
            {
                // operation is finished
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_FINISH);
            }
        }
        break;
        case GAPM_OP_STOP:
        {
            if(GAPM_IS_OP_FIELD_SET(connect->op, CANCELED))
            {
                // operation is finished
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_FINISH);
            }
            else if(GAPM_IS_OP_FIELD_SET(connect->op, ADDR_RENEW))
            {
                // let address management to update state
                gapm_update_address_op_state(&(connect->op), state);
            }
            else
            {
                // else return to start state
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_START);
            }
        }
        break;
        // Perform peer device name request
        case GAPM_OP_NAME_REQ:
        {
            // next step is disconnect
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_DISCONNECT);
        }
        break;

        // Perform peer device disconnection (after name request)
        case GAPM_OP_DISCONNECT:
        {
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_FINISH);
        }
        break;

        // Operation is in canceled state and shall be terminated.
        case GAPM_OP_CANCEL:
        {
            // Cancel operation
            GAPM_SET_OP_FIELD(connect->op, CANCELED);

            if(GAPM_GET_OP_STATE(connect->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_STOP);
            }
            else if(GAPM_GET_OP_STATE(connect->op) == GAPM_OP_NAME_REQ)
            {
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_DISCONNECT);
            }
        }
        break;
        // Renew address generation
        case GAPM_OP_ADDR_RENEW:
        {
            // set Connecting state
            GAPM_SET_OP_FIELD(connect->op, ADDR_RENEW);

            // if waiting state
            if(GAPM_GET_OP_STATE(connect->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_STOP);
            }
        }
        break;

        // Start connection request
        case GAPM_OP_CONNECT_REQ:
        {
            // set Connecting state
            GAPM_SET_OP_FIELD(connect->op, CONNECTING);
            // if waiting state
            if(GAPM_GET_OP_STATE(connect->op) == GAPM_OP_WAIT)
            {
                GAPM_SET_OP_STATE(connect->op, GAPM_OP_STOP);
            }
        }
        break;

        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_ERROR);
        }
        break;
    }
}


void gapm_execute_connect_op(void)
{
    // retrieve connect operation parameters
    struct gapm_start_connection_cmd *connect =
            (struct gapm_start_connection_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

    // clear message in kernel queue
    GAPM_CLEAR_OP_FIELD(connect->op, QUEUED);

    // execute current operation state.
    switch(GAPM_GET_OP_STATE(connect->op))
    {
        // request to clear white list.
        case GAPM_OP_CLEAR_WL:
        {
            struct gapm_white_list_mgt_cmd* clear_wl = KERNEL_MSG_ALLOC(GAPM_WHITE_LIST_MGT_CMD,
                    TASK_GAPM, TASK_GAPM, gapm_white_list_mgt_cmd);

            // fill parameters
            clear_wl->operation = GAPM_CLEAR_WLIST;

            kernel_msg_send(clear_wl);
        }
        break;

        // set white list
        case GAPM_OP_SET_WL:
        {
            struct gapm_white_list_mgt_cmd* set_wl = KERNEL_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD,
                    TASK_GAPM, TASK_GAPM, gapm_white_list_mgt_cmd,
                    connect->nb_peers * sizeof(struct gap_bdaddr));

            // fill parameters
            set_wl->operation = GAPM_ADD_DEV_IN_WLIST;
            set_wl->nb = connect->nb_peers;
            memcpy(set_wl->devices, connect->peers,
                    connect->nb_peers * sizeof(struct gap_bdaddr));
            kernel_msg_send(set_wl);
        }
        break;

        // entry point of scan operation execution.
        case GAPM_OP_SET_PARAMS:
        {
            struct hci_le_set_scan_param_cmd *scan_param = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_SCAN_PARAM_CMD_OPCODE, hci_le_set_scan_param_cmd);

            /* fill up the parameters */

            scan_param->own_addr_type = gapm_get_local_addrtype();

            // in controller privacy mode, use the resolving list
            if(GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) & GAPM_CFG_ADDR_CTNL_PRIVACY)
            {
                scan_param->scan_filt_policy = ((connect->op.code == GAPM_CONNECTION_SELECTIVE)
                        ? SCAN_ALLOW_ADV_WLST_AND_INIT_RPA : SCAN_ALLOW_ADV_ALL_AND_INIT_RPA);
            }
            else
            {
                scan_param->scan_filt_policy = ((connect->op.code == GAPM_CONNECTION_SELECTIVE)
                        ? SCAN_ALLOW_ADV_WLST : SCAN_ALLOW_ADV_ALL);
            }
            scan_param->scan_intv = connect->scan_interval;
            scan_param->scan_window = connect->scan_window;
            scan_param->scan_type = SCAN_BLE_PASSIVE;

            /* send the message */
            hci_send_2_controller(scan_param);
        }
        break;
        // address management
        case GAPM_OP_ADDR_GEN:
        case GAPM_OP_ADDR_SET:
        {
            // Use address management toolbox to set address
            gapm_set_address_op(&connect->op);
        }
        break;
        // no break
        case GAPM_OP_START:
        {
            // should we initiate a connection ?
            if(GAPM_IS_OP_FIELD_SET(connect->op, CONNECTING))
            {
                // initiate a connection
                struct hci_le_create_con_cmd *conn_par = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_CREATE_CON_CMD_OPCODE, hci_le_create_con_cmd);

                // fill up the parameters for connection
                conn_par->init_filt_policy = ((connect->op.code == GAPM_CONNECTION_AUTO)
                                                ? INIT_FILT_USE_WLST : INIT_FILT_IGNORE_WLST);
                conn_par->scan_intv        = connect->scan_interval;
                conn_par->scan_window      = connect->scan_window;
                conn_par->con_intv_max     = connect->con_intv_max;
                conn_par->con_intv_min     = connect->con_intv_min;
                conn_par->ce_len_min       = connect->ce_len_min;
                conn_par->ce_len_max       = connect->ce_len_max;
                conn_par->con_latency      = connect->con_latency;
                conn_par->superv_to        = connect->superv_to;
                conn_par->peer_addr_type   = connect->peers[0].addr_type;
                conn_par->own_addr_type    = gapm_get_local_addrtype();
                memcpy(&(conn_par->peer_addr), &(connect->peers[0].addr), BD_ADDR_LEN);

                /* send connection request */
                hci_send_2_controller(conn_par);
            }
            else
            {
                // start scanning operation
                gapm_set_scan_mode(true, true);
                GAPM_SET_OP_FIELD(connect->op, SCANNING);
            }
        }
        break;
        case GAPM_OP_STOP: // Stop state
        {
            // next step is address generation
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_ADDR_GEN);

            // should we stop scanning
            if(GAPM_IS_OP_FIELD_SET(connect->op, SCANNING))
            {
                // stop scanning operation
                gapm_set_scan_mode(false, true);
                GAPM_CLEAR_OP_FIELD(connect->op, SCANNING);
            }
            else
            {
                // cancel connection request
                gapm_basic_hci_cmd_send(HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE);
            }
        }
        break;

        // Perform peer device name request
        case GAPM_OP_NAME_REQ:
        {
            // connection index has been put in addr_src
            struct gapc_get_info_cmd* info_cmd = KERNEL_MSG_ALLOC(GAPC_GET_INFO_CMD,
                    KERNEL_BUILD_ID(TASK_GAPC, connect->op.addr_src), TASK_GAPM,
                    gapc_get_info_cmd);

            // request peer device name.
            info_cmd->operation = GAPC_GET_PEER_NAME;

            // send command
            kernel_msg_send(info_cmd);
        }
        break;

        // Perform peer device disconnection (after name request)
        case  GAPM_OP_DISCONNECT:
        {
            // connection index has been put in addr_src
            struct gapc_disconnect_cmd* disconnect_cmd = KERNEL_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                    KERNEL_BUILD_ID(TASK_GAPC, connect->op.addr_src), TASK_GAPM,
                    gapc_disconnect_cmd);

            // request peer device name.
            disconnect_cmd->operation = GAPC_DISCONNECT;
            disconnect_cmd->reason    = COMMON_ERROR_REMOTE_USER_TERM_CON;

            // send command
            kernel_msg_send(disconnect_cmd);
        }
        break;

        default:
        {
            // error state, trigger an error message.
            GAPM_SET_OP_STATE(connect->op, GAPM_OP_ERROR);
        }
        break;

    }
}
#endif // (BLE_CENTRAL)

uint8_t  gapm_get_role(void)
{
    // return current role.
    return gapm_env.role;
}


#if (BLE_BROADCASTER || BLE_PERIPHERAL)
void gapm_set_adv_mode(uint8_t en_flag)
{
    struct hci_le_set_adv_en_cmd *adv_en = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_ADV_EN_CMD_OPCODE, hci_le_set_adv_en_cmd);

    adv_en->adv_en = en_flag;

    /* send the message */
    hci_send_2_controller(adv_en);
}
#endif // (BLE_BROADCASTER || BLE_PERIPHERAL)


#if (BLE_OBSERVER || BLE_CENTRAL)
void gapm_set_scan_mode(uint8_t en_flag, uint8_t filter_duplic_en)
{
    struct hci_le_set_scan_en_cmd *scan_en = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_SCAN_EN_CMD_OPCODE, hci_le_set_scan_en_cmd);

    scan_en->scan_en = en_flag;
    scan_en->filter_duplic_en = filter_duplic_en;

    /* send the message */
    hci_send_2_controller(scan_en);
}


uint8_t gapm_get_ad_type_flag(uint8_t *data, uint8_t length)
{
    uint8_t cursor = 0;
    uint8_t add_flag = 0;

    // parse advertising data to find ad_type
    while (cursor < length)
    {
        // check if it's AD Type flag data
        if (data[cursor+1] == GAP_AD_TYPE_FLAGS)
        {
            /* move to the value of the AD flag */
            add_flag = data[cursor+2];
            break;
        }

        /* go to next advertising info */
        cursor += data[cursor] + 1;
    }

    return add_flag;
}

void gapm_add_to_filter(bd_addr_t * addr, uint8_t addr_type)
{
    bd_addr_t default_addr = {{0,0,0,0,0,0}};
    uint8_t cursor = 0;

    // allocate scan filtered device list if needed.
    if(gapm_env.scan_filter == NULL)
    {
        gapm_env.scan_filter =
                (struct gap_bdaddr*) kernel_malloc(sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE, KERNEL_MEM_KERNEL_MSG);

        memset(gapm_env.scan_filter, 0, sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE);
    }

    // find first available space in array
    while((memcmp(&(gapm_env.scan_filter[cursor].addr), &default_addr,
            sizeof(bd_addr_t)) != 0)
           && (cursor <  GAPM_SCAN_FILTER_SIZE))
    {
        cursor++;
    }

    // copy provided device address in array
    if (cursor < GAPM_SCAN_FILTER_SIZE)
    {
        memcpy(&(gapm_env.scan_filter[cursor].addr), addr,
                sizeof(bd_addr_t));
        gapm_env.scan_filter[cursor].addr_type = addr_type;
    }
    // else don't put device into filter.
}

bool gapm_is_filtered(bd_addr_t * addr, uint8_t addr_type)
{
    bool ret = true;
    uint8_t cursor = 0;

    // check that filter array exists
    if(gapm_env.scan_filter != NULL)
    {
        // Find provided address in filter
        while(((memcmp(&(gapm_env.scan_filter[cursor].addr), addr,
                sizeof(bd_addr_t)) != 0 )
                || (gapm_env.scan_filter[cursor].addr_type != addr_type))
                && (cursor <  GAPM_SCAN_FILTER_SIZE))
        {
            cursor++;
        }
        // copy provided device address in array
        if (cursor < GAPM_SCAN_FILTER_SIZE)
        {
            ret = false;
            // remove device from filter.
            memset(&(gapm_env.scan_filter[cursor].addr), 0,
                    sizeof(bd_addr_t));
        }
    }
    return ret;
}

#endif // (BLE_OBSERVER || BLE_CENTRAL)


void gapm_update_air_op_state(uint8_t op_type, uint8_t state, uint8_t status)
{
    // retrieve air operation
    struct gapm_air_operation *op =
            (struct gapm_air_operation *) gapm_get_operation_ptr(op_type);

    // sanity check
    ASSERT_WARN(op != NULL, 0, 0);
    if(op != NULL)
    {
        uint8_t prev_state = GAPM_GET_OP_STATE(*op);
        if(state != GAPM_OP_ERROR)
        {
            // Update state only for specific event if message is in kernel queue
            if(!GAPM_IS_OP_FIELD_SET(*op, QUEUED)
                    || (state == GAPM_OP_CANCEL) || (state == GAPM_OP_TIMEOUT)
                    || (state == GAPM_OP_ADDR_RENEW) || (state == GAPM_OP_CONNECT_REQ))
            {
                switch(op->code)
                {
                    #if (BLE_PERIPHERAL || BLE_BROADCASTER)
                    case GAPM_ADV_NON_CONN:
                    case GAPM_ADV_UNDIRECT:
                    case GAPM_ADV_DIRECT:
                    case GAPM_ADV_DIRECT_LDC:
                    {
                        gapm_update_adv_op_state((struct gapm_start_advertise_cmd*)op , state);
                    }
                    break;
                    #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

                    #if (BLE_OBSERVER || BLE_CENTRAL)
                    case GAPM_SCAN_ACTIVE:
                    case GAPM_SCAN_PASSIVE:
                    {
                        gapm_update_scan_op_state((struct gapm_start_scan_cmd*)op , state);
                    }
                    break;
                    #endif // (BLE_OBSERVER || BLE_CENTRAL)

                    #if (BLE_CENTRAL)
                    case GAPM_CONNECTION_DIRECT:
                    case GAPM_CONNECTION_AUTO:
                    case GAPM_CONNECTION_SELECTIVE:
                    case GAPM_CONNECTION_GENERAL:
                    case GAPM_CONNECTION_NAME_REQUEST:
                    {
                        gapm_update_connect_op_state((struct gapm_start_connection_cmd*)op , state);
                    }
                    break;
                    #endif // (BLE_CENTRAL)

                    default:
                    {
                        // Error unexpected
                        state  = GAPM_OP_ERROR;
                        status = GAP_ERR_UNEXPECTED;
                    }
                    break;
                }
            }
        }

        if(!GAPM_IS_OP_FIELD_SET(*op, QUEUED))
        {
            if((GAPM_GET_OP_STATE(*op) == GAPM_OP_FINISH) || (state == GAPM_OP_ERROR))
            {
                uint8_t op_status = GAP_ERR_NO_ERROR;
                // error
                if((GAPM_GET_OP_STATE(*op) == GAPM_OP_ERROR) || (state == GAPM_OP_ERROR))
                {
                    op_status = status;
                }
                // cancel
                else if(GAPM_IS_OP_FIELD_SET(*op, CANCELED))
                {
                    op_status = GAP_ERR_CANCELED;
                }
                // timeout
                else if(GAPM_IS_OP_FIELD_SET(*op, TIMEOUT))
                {
                    op_status = GAP_ERR_TIMEOUT;
                }
                // else succeed

                // operation is finished
                gapm_send_complete_evt(op_type, op_status);
            }

            else if((prev_state != GAPM_GET_OP_STATE(*op))
                        && (state != GAPM_OP_INIT)
                        && (GAPM_GET_OP_STATE(*op) != GAPM_OP_WAIT))
            {
                // Inform that operation is pushed into kernel queue
                GAPM_SET_OP_FIELD(*op, QUEUED);
                // continue execution
                gapm_reschedule_operation(op_type);
            }
        }
        // an error occurs, send it later.
        else if(state == GAPM_OP_ERROR)
        {
            // keep that an error occurs.
            GAPM_SET_OP_STATE(*op, GAPM_OP_ERROR);
            // Store status code into address source because it is no more used
            op->addr_src = status;
        }
        // else do nothing
    }
}


struct gap_sec_key* gapm_get_irk(void)
{
    return &(gapm_env.irk);
}


bd_addr_t* gapm_get_bdaddr(void)
{
    return &(gapm_env.addr);
}

void gapm_basic_hci_cmd_send(uint16_t opcode)
{
    void *no_param = kernel_msg_alloc(HCI_COMMAND, 0, opcode, 0);
    hci_send_2_controller(no_param);
}
#if (BLE_CENTRAL || BLE_PERIPHERAL)
void gapm_setup_conn(kernel_msg_id_t const msgid, struct hci_le_enh_con_cmp_evt const *event)
{
    uint8_t new_state = GAPM_OP_ERROR;

    // retrieve air operation
    struct gapm_air_operation* air_op =
            (struct gapm_air_operation*) gapm_get_operation_ptr(GAPM_OP_AIR);

    ASSERT_ERR(air_op != NULL);

    if(event->status == COMMON_ERROR_NO_ERROR)
    {
        // connection creation succeeds, inform all layers...
        uint8_t conidx = gapm_con_create(msgid, GAPM_OP_AIR, event);

        if(conidx == GAP_INVALID_CONIDX)
        {
            // Perform an automatic disconnection - Not enough resources.
            /* allocate disconnect command message */
            struct hci_disconnect_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, event->conhdl, HCI_DISCONNECT_CMD_OPCODE, hci_disconnect_cmd);
            cmd->conhdl = event->conhdl;
            cmd->reason = COMMON_ERROR_REMOTE_DEV_TERM_LOW_RESOURCES;
            hci_send_2_controller(cmd);
            new_state = GAPM_OP_ERROR;
        }
        else
        {
            new_state = GAPM_OP_CONNECT;
            // reuse address source too keep connection index.
            air_op->addr_src = conidx;
        }
    }
    #if (BLE_CENTRAL)
    else if(event->status == COMMON_ERROR_UNKNOWN_CONNECTION_ID)
    {
        new_state = GAPM_OP_STOP;
    }
    #endif // (BLE_CENTRAL)
    #if (BLE_PERIPHERAL)
    else if(event->status == COMMON_ERROR_DIRECT_ADV_TO)
    {
        new_state = GAPM_OP_TERM_TIMEOUT;
    }
    #endif // (BLE_PERIPHERAL)

    // update state according to current state
    gapm_update_air_op_state(GAPM_OP_AIR, new_state, RW_ERR_HCI_TO_HL(event->status));

}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t gapm_get_adv_mode (uint8_t operation, struct gapm_air_operation* air_op)
{
    uint8_t mode = GAP_INVALID_MODE;

    /* retrieve operation to know what to do with report. */
    switch(operation)
    {
        case GAPM_SCAN_ACTIVE: // active scan operation
        case GAPM_SCAN_PASSIVE: // passive scan operation
        {
            // retrieve air operation
            struct gapm_start_scan_cmd* scan_cmd =
                    (struct gapm_start_scan_cmd*) air_op;

            // retrieve mode.
            mode = scan_cmd->mode;
        }
        break;
        case GAPM_CONNECTION_GENERAL: // General   connection operation
        case GAPM_CONNECTION_SELECTIVE: // Selective connection operation
        {
            /* It's an observer mode */;
            mode = GAP_OBSERVER_MODE;
        }
        break;
        default: /* nothing to do */ break;
    }

    return mode;
}

uint8_t gapm_get_local_addrtype (void)
{
    // set own address type
    uint8_t local_addr_type = GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE);

    if(local_addr_type & GAPM_CFG_ADDR_CTNL_PRIVACY)
    {
        local_addr_type = ((local_addr_type == GAPM_CFG_ADDR_CTNL_PRIVACY)
                ? ADDR_RPA_OR_PUBLIC: ADDR_RPA_OR_RAND);
    }
    else
    {
        local_addr_type = ((local_addr_type == GAPM_CFG_ADDR_PUBLIC)
                ? ADDR_PUBLIC: ADDR_RAND);
    }

    return (local_addr_type);
}





void gapm_op_reset_continue(uint8_t current_state, uint8_t status)
{
    bool finished = false;

    if(status != GAP_ERR_NO_ERROR)
    {
        // Abort of reset procedure
        finished = true;
    }
    else
    {
        switch(current_state)
        {
            case GAPM_OP_RESET_INIT:
            {
                // send a reset message to lower layers
                gapm_basic_hci_cmd_send(HCI_RESET_CMD_OPCODE);
            } break;

            case GAPM_OP_RESET_HCI:
            {
                /* default device mask */
                uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_EVT_MASK;

                /* query set event mask */
                struct hci_set_evt_mask_cmd *event_mask = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_SET_EVT_MASK_CMD_OPCODE, hci_set_evt_mask_cmd);

                /* fill up the event masking */
                memcpy(&event_mask->event_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

                /* send the event mask */
                hci_send_2_controller(event_mask);
            }
            break;
            case GAPM_OP_RESET_SET_EVT_MASK:
            {
                /* default device mask */
                uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_LE_EVT_MASK;

                /* send query LE event mask */
                struct hci_le_set_evt_mask_cmd *event_mask = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_EVT_MASK_CMD_OPCODE, hci_le_set_evt_mask_cmd);

                /* fill up the event masking */
                memcpy(&event_mask->le_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

                /* send the event mask */
                hci_send_2_controller(event_mask);
            } break;

            case GAPM_OP_RESET_LE_SET_EVT_MASK:
            {
                /* Get local device address. */
                gapm_basic_hci_cmd_send(HCI_RD_BD_ADDR_CMD_OPCODE);
            } break;

            case GAPM_OP_RESET_RD_BD_ADDR:
            {
                #if (BLE_CENTRAL || BLE_PERIPHERAL)
                /* send read buffer size */
                gapm_basic_hci_cmd_send(HCI_LE_RD_BUFF_SIZE_CMD_OPCODE);
                #else
                // End of reset procedure
                finished = true;
                #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
            } break;

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            case GAPM_OP_RESET_LE_RD_BUFF_SIZE:
            {
                // check if number of ble packet is available
                if(l2cm_get_nb_buffer_available() == 0)
                {
                    // number of buffer are shared with BT ACL packets

                    // Send read buffer size legacy command to link layer
                    gapm_basic_hci_cmd_send(HCI_RD_BUFF_SIZE_CMD_OPCODE);
                }
                else
                {
                    // End of reset procedure
                    finished = true;
                }
            } break;
            case GAPM_OP_RESET_RD_BUFF_SIZE:
            {
                // End of reset procedure
                finished = true;
            } break;
            #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
            default:
            {
                ASSERT_INFO(0, current_state, status);
                // Abort of reset procedure
                status = GAP_ERR_PROTOCOL_PROBLEM;
                finished = true;
            } break;
        }
    }

    if(finished)
    {
        // reset operation is finished, inform upper layers.
        gapm_send_complete_evt(GAPM_OP_CFG, status);

        // reset correctly finished.
        if(status == COMMON_ERROR_NO_ERROR)
        {
            // set state to idle
            kernel_state_set(TASK_GAPM, GAPM_IDLE);
        }
    }

}


void gapm_op_setup_continue(uint8_t current_state, uint8_t status)
{
    bool finished = false;

    // retrieve set config operation
    struct gapm_set_dev_config_cmd* set_cfg = (struct gapm_set_dev_config_cmd*) gapm_get_operation_ptr(GAPM_OP_CFG);

    if((set_cfg == NULL) || (set_cfg->operation != GAPM_SET_DEV_CONFIG))
    {
        status = GAP_ERR_PROTOCOL_PROBLEM;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // Abort of reset procedure
        finished = true;
    }
    else
    {
        switch(current_state)
        {
            case GAPM_OP_SETUP_INIT:
            #if BLE_DEBUG
            {
                if((set_cfg->role & GAP_ROLE_DBG_LE_4_0) != 0)
                {
                    // disable some event masks to simulate device as a 4.0 device
                    struct hci_le_set_evt_mask_cmd *event_mask = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_EVT_MASK_CMD_OPCODE, hci_le_set_evt_mask_cmd);

                    /* retrieve LE mask from configuration file */
                    memset(&event_mask->le_mask.mask[0], 0x00, (EVT_MASK_LEN));
                    event_mask->le_mask.mask[0] = (uint8_t) GAP_LE_EVT_4_0_MASK;

                    /* send the event mask */
                    hci_send_2_controller(event_mask);

                    break;
                }
            } // no break
            case GAPM_OP_SETUP_SET_4_0_LE_EVT_MASK:
            #endif // BLE_DEBUG
            #if (BLE_2MBPS)
            {
                // Write Default LE PHY
                struct hci_le_set_dft_phy_cmd *dft_phy = KERNEL_MSG_ALLOC(HCI_COMMAND, 0,
                        HCI_LE_SET_DFT_PHY_CMD_OPCODE, hci_le_set_dft_phy_cmd);

                // Fill data
                dft_phy->all_phys  = (set_cfg->tx_pref_rates == GAP_RATE_ANY) ? ALL_PHYS_TX_NO_PREF : ALL_PHYS_TX_PREF;
                dft_phy->all_phys |= (set_cfg->rx_pref_rates == GAP_RATE_ANY) ? ALL_PHYS_RX_NO_PREF : ALL_PHYS_RX_PREF;

                dft_phy->rx_phys  = set_cfg->rx_pref_rates;
                dft_phy->tx_phys  = set_cfg->tx_pref_rates;

                /* send the message */
                hci_send_2_controller(dft_phy);

            } break;

            case GAPM_OP_SETUP_SET_LE_DFT_PHY_CMD:
            #endif // (BLE_2MBPS)
            {
                // Write Suggested Default LE Data Length
                struct hci_le_wr_suggted_dft_data_len_cmd *set_sugg_data = KERNEL_MSG_ALLOC(HCI_COMMAND, 0,
                        HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, hci_le_wr_suggted_dft_data_len_cmd);

                // Fill data
                set_sugg_data->suggted_max_tx_octets = set_cfg->sugg_max_tx_octets;
                set_sugg_data->suggted_max_tx_time = set_cfg->sugg_max_tx_time;

                /* send the message */
                hci_send_2_controller(set_sugg_data);
            } break;


            case GAPM_OP_SETUP_WR_LE_DFT_DATA_LEN_CMD:
            {
                // Set Address configuration. Value was checked in set_dev_config
                GAPM_F_SET(gapm_env.cfg_flags, ADDR_TYPE, set_cfg->addr_type);
                gapm_env.renew_dur =  common_max(set_cfg->renew_dur, GAP_TMR_PRIV_ADDR_INT);

                // Manage address of the device.
                switch(set_cfg->addr_type)
                {
                    // Device Address generated using Privacy feature
                    case GAPM_CFG_ADDR_HOST_PRIVACY:
                    {
                        // mark that address has to be regenerated
                        GAPM_F_SET(gapm_env.cfg_flags, ADDR_RENEW, 1);
                    } // no break
                    // Device Address is a Public Static address
                    case GAPM_CFG_ADDR_PUBLIC:
                    {
                        // Get local device address from lower layers
                        gapm_basic_hci_cmd_send(HCI_RD_BD_ADDR_CMD_OPCODE);
                    } break;

                    // Device Address is a Private Static address
                    case GAPM_CFG_ADDR_PRIVATE:
                    {
                        // copy provided static address
                        memcpy(&(gapm_env.addr), &(set_cfg->addr), sizeof(bd_addr_t));

                        // set private address in lower layers
                        struct hci_le_set_rand_addr_cmd *rand_addr = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_RAND_ADDR_CMD_OPCODE, hci_le_set_rand_addr_cmd);
                        memcpy(&(rand_addr->rand_addr), &(gapm_env.addr), sizeof(bd_addr_t));
                        hci_send_2_controller(rand_addr);
                    } break;


                    // Private Device Address generated using Controller-based Privacy feature
                    case (GAPM_CFG_ADDR_CTNL_PRIVACY | GAPM_CFG_ADDR_PRIVATE):
                    // Public Device Address generated using Controller-based Privacy feature
                    case GAPM_CFG_ADDR_CTNL_PRIVACY:
                    {
                        // Set timeout for address generation
                        struct hci_le_set_rslv_priv_addr_to_cmd *set_to = KERNEL_MSG_ALLOC(HCI_COMMAND, 0,
                                HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE, hci_le_set_rslv_priv_addr_to_cmd);
                        // Fill data
                        set_to->rpa_timeout = set_cfg->renew_dur;
                        /* send the message */
                        hci_send_2_controller(set_to);
                    } break;

                    default:
                    {
                        ASSERT_INFO(0, set_cfg->addr_type, status);
                        // Abort of reset procedure
                        status = GAP_ERR_PROTOCOL_PROBLEM;
                        finished = true;
                    } break;
                }
            } break;

            case GAPM_OP_SETUP_SET_RENEW_TO:
            {
                // Enable Address Resolution in controller
                struct hci_le_set_addr_resol_en_cmd *en_addr_resol = KERNEL_MSG_ALLOC(HCI_COMMAND, 0,
                        HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE, hci_le_set_addr_resol_en_cmd);
                // Fill data
                en_addr_resol->enable = 1;
                /* send the message */
                hci_send_2_controller(en_addr_resol);
            } break;

            case GAPM_OP_SETUP_EN_CTRL_PRIV:
            {
                // retrieve operation
                struct gapm_set_dev_config_cmd* dev_cfg =
                        (struct gapm_set_dev_config_cmd*) gapm_get_operation_ptr(GAPM_OP_CFG);

                // Public address
                if (dev_cfg->addr_type == GAPM_CFG_ADDR_CTNL_PRIVACY)
                {
                    // Get local device address from lower layers
                    gapm_basic_hci_cmd_send(HCI_RD_BD_ADDR_CMD_OPCODE);
                }
                else
                {
                    // set private address in lower layers
                    struct hci_le_set_rand_addr_cmd *rand_addr = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_RAND_ADDR_CMD_OPCODE, hci_le_set_rand_addr_cmd);
                    memcpy(&(rand_addr->rand_addr), &(gapm_env.addr), sizeof(bd_addr_t));
                    hci_send_2_controller(rand_addr);
                }
            } break;

            case GAPM_OP_SETUP_ADDR_MGT:
            {
                #if (SECURE_CONNECTIONS)
                // check if p256 public key has to be generated
                if ((gapm_env.pairing_mode & GAPM_PAIRING_FORCE_P256_KEY_GEN) != 0)
                {
                    // clear generation status bit
                    gapm_env.pairing_mode &= ~GAPM_PAIRING_FORCE_P256_KEY_GEN;
                    // If Public Key not in NVRAM - then generate a new Public Key and store in NVRAM.
                    gapm_basic_hci_cmd_send(HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE);
                }
                else
                #endif // (SECURE_CONNECTIONS)
                {
                    finished = true;
                }
            } break;

            #if (SECURE_CONNECTIONS)
            case GAPM_OP_SETUP_RD_PRIV_KEY:
            {
                finished = true;
            } break;
            #endif // (SECURE_CONNECTIONS)

            default:
            {
                ASSERT_INFO(0, current_state, status);
                // Abort of reset procedure
                status = GAP_ERR_PROTOCOL_PROBLEM;
                finished = true;
            } break;
        }
    }

    if(finished)
    {
        // Setup operation is finished, inform upper layers.
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }
}


/// @} GAPM_UTIL
