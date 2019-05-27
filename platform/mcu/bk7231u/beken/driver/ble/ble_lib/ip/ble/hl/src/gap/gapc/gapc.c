/**
 ****************************************************************************************
 *
 * @file gapc.c
 *
 * @brief Generic Access Profile Controller Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC Generic Access Profile Controller
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include "gapc_int.h"
#include "l2cc.h"
#include "l2cc_task.h"
#include "attm.h"
#include "kernel_mem.h"
#include "kernel_timer.h"
#include "smpc_api.h" // Access to internal API Required
#include "common_math.h"
#include <string.h>

#ifdef BLE_AUDIO_AM0_TASK
#include "audio.h"
#endif // BLE_AUDIO_AM0_TASK
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

struct gapc_env_tag* gapc_env[GAPC_IDX_MAX];

/// GAP Controller task descriptor
static const struct kernel_task_desc TASK_DESC_GAPC  =
    {NULL, &gapc_default_handler, gapc_state, GAPC_STATE_MAX, GAPC_IDX_MAX};


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/**
 * Clean-up operation parameters
 *
 * @param[in] conidx connection index
 * @param[in] op_type       Operation type.
 */
static void gapc_operation_cleanup(uint8_t conidx, uint8_t op_type)
{
    if(op_type == GAPC_OP_SMP)
    {
        // Release the pairing information structure
        if (gapc_env[conidx]->smpc.info.pair != NULL)
        {
            kernel_free(gapc_env[conidx]->smpc.info.pair);
            gapc_env[conidx]->smpc.info.pair = NULL;
        }

        // Release the signature information structure
        if (gapc_env[conidx]->smpc.info.sign != NULL)
        {
            kernel_free(gapc_env[conidx]->smpc.info.sign);
            gapc_env[conidx]->smpc.info.sign = NULL;
        }

        // Stop the timeout timer if needed
        smpc_clear_timeout_timer(conidx);

        // Reset the internal state
        gapc_env[conidx]->smpc.state = SMPC_STATE_RESERVED;
    }

    // clean-up operation pointer.
    if(gapc_env[conidx]->operation[op_type] != NULL)
    {
        // protection to double free of operation if message already present in heap.
        if(!kernel_msg_in_queue(gapc_env[conidx]->operation[op_type]))
        {
            kernel_msg_free(kernel_param2msg(gapc_env[conidx]->operation[op_type]));
        }
        gapc_env[conidx]->operation[op_type] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Cleanup GAP controller resources for connection
 *
 * @param[in] conidx connection record index
 *
 ****************************************************************************************
 */
static void gapc_cleanup(uint8_t conidx)
{
    // clean-up task?
    if(gapc_env[conidx] != NULL)
    {
        kernel_state_t state = kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx));
        uint8_t i;

        for(i = 0 ; i < GAPC_OP_MAX ; i++)
        {
            if(state != GAPC_DISC_BUSY)
            {
                // free operation if exist
                gapc_operation_cleanup(conidx, i);
            }
            else
            {
                if(gapc_env[conidx]->operation[i] != NULL)
                {
                    // send connection lost error
                    gapc_send_complete_evt(conidx, i, GAP_ERR_DISCONNECTED);
                }
            }
        }

        kernel_free(gapc_env[conidx]);
        gapc_env[conidx] = NULL;
    }

    // set Free state
    kernel_state_set(KERNEL_BUILD_ID(TASK_GAPC, conidx), GAPC_FREE);
    /* Stop timer */
    kernel_timer_clear(GAPC_PARAM_UPDATE_TO_IND, KERNEL_BUILD_ID(TASK_GAPC, conidx));
    kernel_timer_clear(GAPC_SMP_REP_ATTEMPTS_TIMER_IND, KERNEL_BUILD_ID(TASK_GAPC, conidx));
}


/**
 ****************************************************************************************
 * @brief Create environment for given connection index.
 *
 * @param[in] conidx connection record index
 *
 ****************************************************************************************
 */
static void gapc_create(uint8_t conidx)
{
    ASSERT_ERR(conidx < GAPC_IDX_MAX);

    // allocate environment variable for current GAP controller.
    gapc_env[conidx] = (struct gapc_env_tag*) kernel_malloc(sizeof(struct gapc_env_tag), KERNEL_MEM_ENV);
    // clear environment data.
    memset(gapc_env[conidx], 0, sizeof(struct gapc_env_tag));

    // Set the default value of the Repeated Attempt timer
    gapc_env[conidx]->smpc.rep_att_timer_val = SMPC_REP_ATTEMPTS_TIMER_DEF_VAL;

    // Set the default value of device features
    gapc_env[conidx]->features = GAPC_ENCRYPT_FEAT_MASK |
                               /*  GAPC_CONN_PARAM_REQ_FEAT_MASK |*/
                                 GAPC_EXT_REJECT_IND_FEAT_MASK |
                                 GAPC_SLAVE_FEAT_EXCH_FEAT_MASK |
                                 GAPC_LE_PING_FEAT_MASK;

    // set device into a ready state
    kernel_state_set(KERNEL_BUILD_ID(TASK_GAPC, conidx), GAPC_READY);
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
void gapc_init(bool reset)
{
    // Index
    uint8_t conidx;

    if(!reset)
    {
        // Create GAP Controller task
        kernel_task_create(TASK_GAPC, &TASK_DESC_GAPC);

        // Initialize GAP controllers environment variable
        for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
        {
            gapc_env[conidx] = NULL;
        }
    }

    // Initialize GAP controllers
    for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
    {
        // clean environment variables
        gapc_cleanup(conidx);
    }
}

uint8_t gapc_con_create(kernel_msg_id_t const msgid, struct hci_le_enh_con_cmp_evt const *con_params,
                        kernel_task_id_t requester, bd_addr_t* laddr, uint8_t laddr_type)
{
#if BLE_EMB_PRESENT
    uint8_t conidx = con_params->conhdl;

    if(kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx)) != GAPC_FREE)
    {
        // error, return wrong connection index.
        conidx = GAP_INVALID_CONIDX;
    }
#else // ! BLE_EMB_PRESENT
    uint8_t conidx;

    // Find first available connection index
    for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
    {
        // find first task index within free state.
        if(kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx)) == GAPC_FREE)
        {
            break;
        }
    }

    // No free slot found
    if(conidx == GAPC_IDX_MAX)
    {
        // error, return wrong connection index.
        conidx = GAP_INVALID_CONIDX;
    }
#endif // BLE_EMB_PRESENT
    else
    {
        // Create environment variable for current connection.
        gapc_create(conidx);

        // set specific connection parameters
        gapc_env[conidx]->conhdl = con_params->conhdl;

        GAPC_SET_FIELD(conidx, ROLE, con_params->role);

        // Check if RPA is provided, otherwise use public identity
        uint8_t empty_addr[BD_ADDR_LEN] = {0,0,0,0,0,0};


        if (memcmp(&(con_params->peer_rslv_priv_addr), empty_addr, sizeof(bd_addr_t)))
        {
            // keep peer address information
            memcpy(&(gapc_env[conidx]->src[SMPC_INFO_PEER].addr), &(con_params->peer_rslv_priv_addr),
                    sizeof(bd_addr_t));
            // Save address type for peer
            gapc_env[conidx]->src[SMPC_INFO_PEER].addr_type = ADDR_RAND;
        }
        else
        {
            // keep peer address information
            memcpy(&(gapc_env[conidx]->src[SMPC_INFO_PEER].addr), &(con_params->peer_addr),
                    sizeof(bd_addr_t));

            // Save address type for peer
            gapc_env[conidx]->src[SMPC_INFO_PEER].addr_type = con_params->peer_addr_type;
        }


        if (memcmp(&(con_params->loc_rslv_priv_addr), empty_addr, sizeof(bd_addr_t)))
        {
            // keep local address information used to create the link
            memcpy(&(gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr), &(con_params->loc_rslv_priv_addr),
                    sizeof(bd_addr_t));
            // Save address type for local
            gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr_type = ADDR_RAND;
        }
        else
        {
            // keep local address information used to create the link
            memcpy(&(gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr), laddr,
                    sizeof(bd_addr_t));
            // Save address type for local
            gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr_type = laddr_type;
        }


        if(requester != TASK_GAPM)
        {
            // Send connection indication message to task that requests connection.
            struct gapc_connection_req_ind * connection_ind = KERNEL_MSG_ALLOC(GAPC_CONNECTION_REQ_IND,
                    APP_MAIN_TASK, KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_connection_req_ind);

            // fill parameters
            connection_ind->conhdl         = con_params->conhdl;
            connection_ind->peer_addr_type = con_params->peer_addr_type;
            memcpy(&(connection_ind->peer_addr), &(con_params->peer_addr),
                    sizeof(bd_addr_t));
            connection_ind->con_interval   = con_params->con_interval;
            connection_ind->con_latency    = con_params->con_latency;
            connection_ind->sup_to         = con_params->sup_to;
            connection_ind->clk_accuracy   = con_params->clk_accuracy;

            // send indication
            kernel_msg_send(connection_ind);
        }
    }

    return conidx;
}


uint8_t gapc_con_cleanup(uint8_t conidx)
{
    // Check connection index before doing anything.
    if(conidx != GAP_INVALID_CONIDX)
    {
        // Remove task instance linked to connection index
        gapc_cleanup(conidx);
    }

    // return connection index.
    return conidx;
}

void gapc_send_disconect_ind(uint8_t conidx,  uint8_t reason, uint8_t conhdl,
                             kernel_task_id_t dest_id)
{
    // send disconnection message.
    struct gapc_disconnect_ind * disconnect_ind = KERNEL_MSG_ALLOC(GAPC_DISCONNECT_IND,
            dest_id, KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_disconnect_ind);

    // fill parameters
    disconnect_ind->conhdl   = conhdl;
    disconnect_ind->reason   = reason;

    // send indication
    kernel_msg_send(disconnect_ind);
}



uint8_t gapc_get_conidx(uint16_t conhdl)
{
    uint8_t conidx = 0;

    // Find first available connection index
    for (conidx = 0; conidx < GAPC_IDX_MAX; conidx++)
    {
        // find first task index within free state.
        if((gapc_env[conidx] != NULL)  && (gapc_env[conidx]->conhdl == conhdl))
        {
            break;
        }
    }

    // Nothing found
    if(conidx == GAPC_IDX_MAX)
    {
        // error, return wrong connection index.
        conidx = GAP_INVALID_CONIDX;
    }

    return conidx;
}

uint16_t gapc_get_conhdl(uint8_t conidx)
{
    uint16_t conhdl = GAP_INVALID_CONHDL;

    // check if environment variable exist
    if((conidx < GAPC_IDX_MAX) && (gapc_env[conidx] != NULL))
    {
        conhdl = gapc_env[conidx]->conhdl;
    }

    return conhdl;
}

uint8_t gapc_get_role(uint8_t conidx)
{
    uint8_t ret =  ROLE_MASTER;

    // Index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (gapc_env[conidx] != NULL))
    {
        ret = GAPC_GET_FIELD(conidx, ROLE);
    }

    return ret;
}

struct gap_bdaddr* gapc_get_bdaddr(uint8_t conidx, uint8_t src)
{
    struct gap_bdaddr* addr = NULL;

    // index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (src < SMPC_INFO_MAX) && (gapc_env[conidx] != NULL))
    {
        addr = &(gapc_env[conidx]->src[src]);
    }

    return addr;
}

struct gap_sec_key* gapc_get_csrk(uint8_t conidx, uint8_t src)
{
    struct gap_sec_key* csrk = NULL;

    // index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (src < SMPC_INFO_MAX) && (gapc_env[conidx] != NULL))
    {
        csrk = &(gapc_env[conidx]->smpc.csrk[src]);
    }

    return csrk;
}

uint32_t gapc_get_sign_counter(uint8_t conidx, uint8_t src)
{
    uint32_t sign_counter = 0;

    // index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (src < SMPC_INFO_MAX) && (gapc_env[conidx] != NULL))
    {
        sign_counter = gapc_env[conidx]->smpc.sign_counter[src];
    }

    return sign_counter;
}

void gapc_send_complete_evt(uint8_t conidx, uint8_t op_type, uint8_t status)
{
    if(gapc_env[conidx]->operation[op_type] != NULL)
    {
        // send command completed event
        struct gapc_cmp_evt *cmp_evt = KERNEL_MSG_ALLOC(GAPC_CMP_EVT, gapc_get_requester(conidx, op_type),
                KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_cmp_evt);

        /* fill up the parameters */
        cmp_evt->operation = *((uint8_t*) gapc_env[conidx]->operation[op_type]);
        cmp_evt->status = status;

        /* send the complete event */
        kernel_msg_send(cmp_evt);
    }

    // clean-up operation parameters
    gapc_operation_cleanup(conidx, op_type);

    // set state to ready
    gapc_update_state(conidx, (1 << op_type), false);
}


void gapc_send_error_evt(uint8_t conidx, uint8_t operation, const kernel_task_id_t requester, uint8_t status)
{
    // prepare command completed event with error status
    struct gapc_cmp_evt* cmp_evt = KERNEL_MSG_ALLOC(GAPC_CMP_EVT,
            requester, KERNEL_BUILD_ID(TASK_GAPC, conidx), gapc_cmp_evt);

    cmp_evt->operation = operation;
    cmp_evt->status = status;

    // send event
    kernel_msg_send(cmp_evt);
}

uint8_t gapc_get_operation(uint8_t conidx, uint8_t op_type)
{
    // by default no operation
    uint8_t ret = GAPC_NO_OP;

    ASSERT_ERR(conidx < GAPC_IDX_MAX);
    ASSERT_ERR(op_type < GAPC_OP_MAX);

    // check if an operation is registered
    if(gapc_env[conidx]->operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t*) gapc_env[conidx]->operation[op_type]));
    }

    return ret;
}

void* gapc_get_operation_ptr(uint8_t conidx, uint8_t op_type)
{
    ASSERT_ERR(conidx  < GAPC_IDX_MAX);
    ASSERT_ERR(op_type < GAPC_OP_MAX);
    // return operation pointer
    return gapc_env[conidx]->operation[op_type];
}

void gapc_set_operation_ptr(uint8_t conidx, uint8_t op_type, void* op)
{
    ASSERT_ERR(conidx  < GAPC_IDX_MAX);
    ASSERT_ERR(op_type < GAPC_OP_MAX);
    // set operation pointer
    gapc_env[conidx]->operation[op_type] = op;
}

bool gapc_reschedule_operation(uint8_t conidx, uint8_t op_type)
{
    bool ret = false;

    ASSERT_ERR(conidx  < GAPC_IDX_MAX);
    ASSERT_ERR(op_type < GAPC_OP_MAX);
    // check if operation not null
    if(gapc_env[conidx]->operation[op_type] != NULL)
    {
        ret = true;
        // request kernel to reschedule operation with operation source ID
        kernel_msg_forward(gapc_env[conidx]->operation[op_type],
                KERNEL_BUILD_ID(TASK_GAPC, conidx), kernel_msg_src_id_get(gapc_env[conidx]->operation[op_type]));
    }

    return ret;
}


kernel_task_id_t gapc_get_requester(uint8_t conidx, uint8_t op_type)
{
    kernel_task_id_t ret = 0;
    ASSERT_ERR(conidx  < GAPC_IDX_MAX);
    ASSERT_ERR(op_type < GAPC_OP_MAX);

    // check if an operation is registered
    if(gapc_env[conidx]->operation[op_type] != NULL)
    {
        // retrieve operation requester.
        ret = kernel_msg_src_id_get(gapc_env[conidx]->operation[op_type]);
    }

    return ret;
}



bool gapc_is_sec_set(uint8_t conidx, uint8_t sec_req)
{
    bool ret = false;

    switch(sec_req)
    {
        // Link is bonded
        case GAPC_LK_BONDED:
        {
            ret = GAPC_GET_FIELD(conidx, BONDED);
        }
        break;
        // Link is encrypted
        case GAPC_LK_ENCRYPTED:
        {
            ret = GAPC_GET_FIELD(conidx, ENCRYPTED);
        }
        break;

        // Link LTK has been exchanged during pairing
        case GAPC_LK_LTK_PRESENT:
        {
            ret = GAPC_GET_FIELD(conidx, LTK_PRESENT);
        }
        break;
        default: /* Nothing to do */ break;
    }

    return ret;
}

uint8_t gapc_lk_sec_lvl_get(uint8_t conidx)
{
    uint8_t sec_lvl = GAP_LK_NO_AUTH;
    // index should not be invalid; this is just for protection
    if((conidx < GAPC_IDX_MAX) && (gapc_env[conidx] != NULL))
    {
        // retrieve link security level
        sec_lvl = GAPC_GET_FIELD(conidx, SEC_LVL);
    }
    return sec_lvl;
}


uint8_t gapc_enc_keysize_get(uint8_t conidx)
{
    uint8_t key_size = 0;

    // index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (gapc_env[conidx] != NULL))
    {
        key_size = gapc_env[conidx]->smpc.key_size;
    }

    return key_size;
}

void gapc_enc_keysize_set(uint8_t conidx, uint8_t key_size)
{
    // index should not be invalid; this is just for protection
    if((conidx != GAP_INVALID_CONIDX) && (gapc_env[conidx] != NULL))
    {
        gapc_env[conidx]->smpc.key_size = key_size;
    }
}



void gapc_link_encrypted(uint8_t conidx)
{
    // index should not be invalid; this is just for protection
    if((conidx < GAPC_IDX_MAX) && (gapc_env[conidx] != NULL))
    {
        // Link Encrypted
        GAPC_SET_FIELD(conidx, ENCRYPTED, true);
    }
}


void gapc_auth_set(uint8_t conidx, uint8_t auth, bool ltk_present)
{
    // index should not be invalid; this is just for protection
    if((conidx < GAPC_IDX_MAX) && (gapc_env[conidx] != NULL))
    {
        uint8_t sec_lvl = GAP_LK_NO_AUTH;
        GAPC_SET_FIELD(conidx, BONDED,     ((auth & GAP_AUTH_BOND) == GAP_AUTH_BOND));
        GAPC_SET_FIELD(conidx, LTK_PRESENT, ltk_present);

        // secure connection bit set, we are on a secure connected link
        if(auth & GAP_AUTH_SEC_CON)
        {
            sec_lvl = GAP_LK_SEC_CON;
        }
        // MITM flag set, authenticated link
        else if (auth & GAP_AUTH_MITM)
        {
            sec_lvl = GAP_LK_AUTH;
        }
        // only bond flag set, or in encrypted link, we are on a unauthenticated link
        else if ((auth & GAP_AUTH_BOND) || GAPC_GET_FIELD(conidx, ENCRYPTED))
        {
            sec_lvl = GAP_LK_UNAUTH;
        }
        // authentication level updated
        GAPC_SET_FIELD(conidx, SEC_LVL, sec_lvl);
    }
}

uint8_t gapc_auth_get(uint8_t conidx)
{
    uint8_t auth = 0;

    // index should not be invalid; this is just for protection
    if((conidx < GAPC_IDX_MAX) && (gapc_env[conidx] != NULL))
    {
        if(GAPC_GET_FIELD(conidx, BONDED))
        {
            auth |= GAP_AUTH_BOND;
        }

        switch(GAPC_GET_FIELD(conidx, SEC_LVL))
        {
            case GAP_LK_AUTH:
            {
                auth |= GAP_AUTH_MITM;
            } break;
            case GAP_LK_SEC_CON:
            {
                auth |= GAP_AUTH_SEC_CON;
            } break;
            default: /* Nothing to do */ break;
        }
    }

    return auth;
}

bool gapc_svc_chg_ccc_get(uint8_t conidx)
{
    return (GAPC_GET_FIELD(conidx, SVC_CHG_CCC) != 0);

}

void gapc_svc_chg_ccc_set(uint8_t conidx, bool enable)
{
    GAPC_SET_FIELD(conidx, SVC_CHG_CCC, enable);
}

void gapc_update_state(uint8_t conidx, kernel_state_t state, bool busy)
{
    // change state only if there is no ongoing disconnect.
    if(gapc_env[conidx]->disc_requester == 0)
    {
        kernel_state_t old_state  = kernel_state_get(KERNEL_BUILD_ID(TASK_GAPC, conidx));
        if(busy)
        {
            // set state to busy
            kernel_state_set(KERNEL_BUILD_ID(TASK_GAPC, conidx), old_state | state);
        }
        else
        {
            // set state to idle
            kernel_state_set(KERNEL_BUILD_ID(TASK_GAPC, conidx), old_state & ~(state));
        }
    }
}


bool gapc_param_update_sanity(uint16_t intv_max, uint16_t intv_min, uint16_t latency, uint16_t timeout)
{
    /* check for the range validity of the values */
    #if(BLE_AUDIO)
    if ( (intv_max < AUDIO_MIN_INTERVAL || intv_max > GAP_CNX_INTERVAL_MAX) ||
         (intv_min < AUDIO_MIN_INTERVAL || intv_min > GAP_CNX_INTERVAL_MAX) ||
         (timeout  < GAP_CNX_SUP_TO_MIN || timeout  > GAP_CNX_SUP_TO_MAX)   ||
         (latency  > GAP_CNX_LATENCY_MAX) )
    {
        return false;
    }
    #else // !(BLE_AUDIO)
    if ( (intv_max < GAP_CNX_INTERVAL_MIN || intv_max > GAP_CNX_INTERVAL_MAX) ||
         (intv_min < GAP_CNX_INTERVAL_MIN || intv_min > GAP_CNX_INTERVAL_MAX) ||
         (timeout  < GAP_CNX_SUP_TO_MIN   || timeout  > GAP_CNX_SUP_TO_MAX)   ||
         (latency  > GAP_CNX_LATENCY_MAX) )
    {
        return false;
    }
    #endif // (BLE_AUDIO)
    return true;
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} GAPC
