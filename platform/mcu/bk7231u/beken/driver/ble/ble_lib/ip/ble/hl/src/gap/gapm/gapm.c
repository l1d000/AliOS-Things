/**
 ****************************************************************************************
 *
 * @file gapm.c
 *
 * @brief Generic Access Profile Manager Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM Generic Access Profile Manager
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"  // Software configuration

#include "gap.h"          // Generic access profile
#include "gapm.h"         // Generic access profile Manager

#include "common_math.h"      // Mathematic library
#include "kernel_mem.h"       // Kernel memory management
#include "kernel_timer.h"     // Kernel timers
#include "gapc.h"         // Generic access profile Controller
#include "gapc_int.h" // Internal API required

#include "gapm_int.h"     // Generic access profile Manager Internal
#include "gapm_util.h"    // Generic access profile Manager util function

#include "attm.h"         // Attribute Database management

#include "l2cm.h"         // L2CAP Manager API
#include "l2cc_pdu.h"     // L2CAP PDU defines

#include "gattc.h"        // Generic Attribute
#include "gattm.h"        // Generic Attribute Manager


#if (NVDS_SUPPORT)
#include "nvds.h"
#endif // (NVDS_SUPPORT)

#include "smpm_api.h" // Access to internal API Required

#if (BLE_PROFILES)
#include "prf.h"
#include "RomCallFlash.h"
#endif // (BLE_PROFILES)

#ifdef BLE_AUDIO_AM0_TASK
#include "am0_api.h"
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
struct gapm_env_tag gapm_env;


/// GAP Manager task descriptor
static const struct kernel_task_desc TASK_DESC_GAPM  =
    {NULL, &gapm_default_handler, gapm_state, GAPM_STATE_MAX, GAPM_IDX_MAX};


#if (BLE_ATTS)
/// GAP Attribute database description
static const struct attm_desc gapm_att_db[GAP_IDX_NUMBER] =
{
    // GAP_IDX_PRIM_SVC - GAP service
    [GAP_IDX_PRIM_SVC]              =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_CHAR_DEVNAME - device name declaration
    [GAP_IDX_CHAR_DEVNAME]          =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_DEVNAME - device name definition
    [GAP_IDX_DEVNAME]               =   {ATT_CHAR_DEVICE_NAME, PERM(RD, ENABLE), PERM(RI, ENABLE), GAP_MAX_NAME_SIZE},
    // GAP_IDX_CHAR_ICON - appearance declaration
    [GAP_IDX_CHAR_ICON]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ICON -appearance
    [GAP_IDX_ICON]                  =   {ATT_CHAR_APPEARANCE, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint16_t)},
    // GAP_IDX_CHAR_SLAVE_PREF_PARAM - Peripheral parameters declaration
    [GAP_IDX_CHAR_SLAVE_PREF_PARAM] =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_SLAVE_PREF_PARAM - Peripheral parameters definition
    [GAP_IDX_SLAVE_PREF_PARAM]      =   {ATT_CHAR_PERIPH_PREF_CON_PARAM, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(struct gap_slv_pref)},
    // GAP_IDX_CHAR_ADDR_RESOL - Central Address Resolution declaration
    [GAP_IDX_CHAR_CNT_ADDR_RESOL]   =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ADDR_RESOL_SUPP - Central Address Resolution supported
    [GAP_IDX_CNT_ADDR_RESOL]        =   {ATT_CHAR_CTL_ADDR_RESOL_SUPP, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint8_t)},

};
#endif /* (BLE_ATTS) */


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup operation
 *
 * @param[in] op_type Operation type.
 ****************************************************************************************
 */
static void gapm_operation_cleanup(uint8_t op_type)
{
    // check if operation is freed
    if(gapm_env.operation[op_type] != NULL)
    {
        // check if operation is in kernel queue
        if(!kernel_msg_in_queue(gapm_env.operation[op_type]))
        {
            // free operation
            kernel_msg_free(kernel_param2msg(gapm_env.operation[op_type]));
            gapm_env.operation[op_type] = NULL;
        }
    }

    // specific air operation cleanup
     if(op_type == GAPM_OP_AIR)
     {
        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        kernel_timer_clear(GAPM_LIM_DISC_TO_IND , TASK_GAPM);
        #endif // (BLE_PERIPHERAL || BLE_BROADCASTER)

        #if (BLE_CENTRAL || BLE_OBSERVER)
        // stop limited scan duration timer
        kernel_timer_clear(GAPM_SCAN_TO_IND , TASK_GAPM);
        // free bd address filter
        if(gapm_env.scan_filter != NULL)
        {
            kernel_free(gapm_env.scan_filter);
            gapm_env.scan_filter = NULL;
        }
        #endif // (BLE_CENTRAL || BLE_OBSERVER)
    }

     // set operation state to Idle
     gapm_update_state(op_type, false);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_init(bool reset)
{
    int i;
    // boot configuration
    if(!reset)
    {
        // Create GAP Manager task
        kernel_task_create(TASK_GAPM, &TASK_DESC_GAPM);

        // by default set operations pointer to null
        for (i = 0 ; i < GAPM_OP_MAX ; i++)
        {
            gapm_set_operation_ptr(i ,NULL);
        }

        #if (BLE_LECB)
        // Initialize list of LE PSM
        common_list_init(&(gapm_env.reg_le_psm));
        #endif // (BLE_LECB)

        #if (BLE_CENTRAL || BLE_OBSERVER)
        gapm_env.scan_filter = NULL;
        #endif // (BLE_CENTRAL || BLE_OBSERVER)

        #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
        // by default enable embedded host.

{
        extern uint32_t  ble_dut_flag;
        if(ble_dut_flag)
            gapm_env.embedded_host = false;
        else
            gapm_env.embedded_host = true;
}

        #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    }

    #if (BLE_LECB)
    // remove all registered LE_PSM
    gapm_le_psm_cleanup();
    #endif // (BLE_LECB)

    // Profile manager initialization
    #if (BLE_PROFILES)
    prf_init(reset);
		//rom_env.prf_init(reset);
    #endif // (BLE_PROFILES)

    for (i = 0 ; i < GAPM_OP_MAX ; i++)
    {
        gapm_operation_cleanup(i);
    }

    // stop timers
    kernel_timer_clear(GAPM_ADDR_RENEW_TO_IND , TASK_GAPM);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize GAP controllers
    gapc_init(reset);
    #endif //(BLE_CENTRAL || BLE_PERIPHERAL)

    // clear current role
    gapm_env.role = GAP_ROLE_NONE;
    // all connections reset
    gapm_env.connections = 0;
    #if (BLE_LECB)
    gapm_env.nb_lecb     = 0;
    #endif // (BLE_LECB)
    /* Set the GAP state to GAP_DEV_SETUP
     * - First time GAP is used, a reset shall be performed to correctly initialize lower
     *   layers configuration. It can be performed automatically if receive a ready event
     *   from lower layers, but environment, since it's not mandatory to trigger this
     *   message, GAP must wait for a reset request before doing anything.
     */
    kernel_state_set(TASK_GAPM, GAPM_DEVICE_SETUP);
}

#if (BLE_ATTS)
uint8_t gapm_init_attr(uint16_t start_hdl, uint32_t feat)
{
    /* initialize Start Handle */
    gapm_env.svc_start_hdl = start_hdl;

    /* Create the database */
    return attm_svc_create_db(&(gapm_env.svc_start_hdl), ATT_SVC_GENERIC_ACCESS, (uint8_t*) &feat,
            GAP_IDX_NUMBER, NULL, TASK_GAPC, &gapm_att_db[0],
            PERM(SVC_MI, ENABLE));
}

#endif // (BLE_ATTS)

uint8_t gapm_get_operation(uint8_t op_type)
{
    // by default no operation
    uint8_t ret = GAPM_NO_OP;

    ASSERT_ERR(op_type < GAPM_OP_MAX);

    // check if an operation is registered
    if(gapm_env.operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t*) gapm_env.operation[op_type]));
    }

    return ret;
}


kernel_task_id_t gapm_get_requester(uint8_t op_type)
{
    kernel_task_id_t ret = 0;
    ASSERT_ERR(op_type < GAPM_OP_MAX);

    // check if an operation is registered
    if(gapm_env.operation[op_type] != NULL)
    {
        // retrieve operation requester.
        ret = kernel_msg_src_id_get(gapm_env.operation[op_type]);
    }

    return ret;
}


bool gapm_reschedule_operation(uint8_t op_type)
{
    bool ret = false;

    ASSERT_ERR(op_type < GAPM_OP_MAX);
    // check if operation not null
    if(gapm_env.operation[op_type] != NULL)
    {
        ret = true;
        // request kernel to reschedule operation with operation source ID
        kernel_msg_forward(gapm_env.operation[op_type],
                TASK_GAPM, kernel_msg_src_id_get(gapm_env.operation[op_type]));
    }

    return ret;
}



void gapm_send_complete_evt(uint8_t op_type, uint8_t status)
{
    uint8_t* op_cmd = (uint8_t*) gapm_get_operation_ptr(op_type);

    // check that operation is valid
    if(op_cmd != NULL)
    {
        // prepare command completed event
        struct gapm_cmp_evt* cmp_evt = KERNEL_MSG_ALLOC(GAPM_CMP_EVT,
                gapm_get_requester(op_type), TASK_GAPM, gapm_cmp_evt);

        cmp_evt->operation = *op_cmd;
        cmp_evt->status = status;

        // send event
        kernel_msg_send(cmp_evt);
    }

    // cleanup operation
    gapm_operation_cleanup(op_type);
}

void gapm_send_error_evt(uint8_t operation, const kernel_task_id_t requester, uint8_t status)
{
    // check that requester value is valid
    if(requester != 0)
    {
        // prepare command completed event with error status
        struct gapm_cmp_evt* cmp_evt = KERNEL_MSG_ALLOC(GAPM_CMP_EVT,
                requester, TASK_GAPM, gapm_cmp_evt);

        cmp_evt->operation = operation;
        cmp_evt->status = status;

        // send event
        kernel_msg_send(cmp_evt);
    }
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t gapm_con_create(kernel_msg_id_t const msgid, uint8_t operation, struct hci_le_enh_con_cmp_evt const *con_params)
{
    struct gapm_air_operation* air_op = (struct gapm_air_operation*)gapm_get_operation_ptr(operation);

    uint8_t addr_type = gapm_get_address_type();

    // First create GAP controller task (that send indication message)
    uint8_t conidx = gapc_con_create(msgid, con_params,
            ((air_op->code == GAPM_CONNECTION_NAME_REQUEST) ? TASK_GAPM : gapm_get_requester(operation)),
            &(gapm_env.addr), (((addr_type == GAPM_CFG_ADDR_PUBLIC) || (addr_type == GAPM_CFG_ADDR_CTNL_PRIVACY))
                    ? ADDR_PUBLIC : ADDR_RAND));

    // check if an error occurs.
    if(conidx != GAP_INVALID_CONIDX)
    {
        // Increment number of connections.
        gapm_env.connections++;

        /* ******** Inform other tasks that connection has been established. ******** */

        // Inform L2CAP about new connection
        l2cm_create(conidx);
        // Inform GATT about new connection
        gattm_create(conidx);


        #if (BLE_PROFILES)
        // Inform profiles about new connection
        prf_create(conidx);
			//	rom_env.prf_create(conidx);
        #endif /* (BLE_PROFILES) */


        #ifdef BLE_AUDIO_AM0_TASK
        // Inform Audio Mode 0 task that new connection is created
        am0_task_create(conidx);
        #endif // BLE_AUDIO_AM0_TASK
    }

    return conidx;
}


void gapm_con_enable(uint8_t conidx)
{
    // sanity check.
    if(conidx != GAP_INVALID_CONIDX)
    {
        // Inform ATT that connection information has be set
        gattc_con_enable(conidx);
    }
}

void gapm_con_cleanup(uint8_t conidx, uint16_t conhdl, uint8_t reason)
{
    // check if an error occurs.
    if(conidx != GAP_INVALID_CONIDX)
    {
        // Decrement number of connections.
        gapm_env.connections--;

        /* ******** Inform other tasks that connection has been disconnected. ******** */

        // Inform GAPC about terminated connection
        gapc_con_cleanup(conidx);

        // Inform L2CAP about terminated connection
        l2cm_cleanup(conidx);
        // Inform GATT about terminated connection
        gattm_cleanup(conidx);

        #if (BLE_PROFILES)
        // Inform profiles about terminated connection
        prf_cleanup(conidx, reason);
			//  rom_env.prf_cleanup(conidx, reason);
        #endif /* (BLE_PROFILES) */

        #ifdef BLE_AUDIO_AM0_TASK
        // Inform Audio Mode 0 task about terminated connection
        am0_task_cleanup(conidx);
        #endif // BLE_AUDIO_AM0_TASK
    }
}
#endif /* (BLE_CENTRAL || BLE_PERIPHERAL) */

kernel_task_id_t gapm_get_id_from_task(kernel_msg_id_t task)
{
    kernel_task_id_t id = TASK_ID_INVALID;
    uint8_t idx = KERNEL_IDX_GET(task);
    task = KERNEL_TYPE_GET(task);

    switch(task)
    {
        case TASK_GAPM:  id = TASK_ID_GAPM;  break;
        case TASK_GAPC:  id = TASK_ID_GAPC;  break;
        case TASK_GATTM: id = TASK_ID_GATTM; break;
        case TASK_GATTC: id = TASK_ID_GATTC; break;
        case TASK_L2CC:  id = TASK_ID_L2CC;  break;
        #if (AHI_TL_SUPPORT)
        case TASK_AHI:   id = TASK_ID_AHI;   break;
        #endif // (AHI_TL_SUPPORT)
        #ifdef BLE_AUDIO_AM0_TASK
        case TASK_AM0:   id = TASK_ID_AM0;   break;
        #endif // BLE_AUDIO_AM0_TASK
        default:
        {
            #if (BLE_PROFILES)
            // check if profile manager is able to retrieve the task id
            id = prf_get_id_from_task(task);
					//	id = rom_env.prf_get_id_from_task(task);
            #endif // (BLE_PROFILES)
        }
        break;
    }

    return KERNEL_BUILD_ID(id, idx);
}

kernel_task_id_t gapm_get_task_from_id(kernel_msg_id_t id)
{
    kernel_task_id_t task = TASK_BLE_NONE;
    uint8_t idx = KERNEL_IDX_GET(id);
    id = KERNEL_TYPE_GET(id);

    switch(id)
    {
        case TASK_ID_GAPM:  task = TASK_GAPM;  break;
        case TASK_ID_GAPC:  task = TASK_GAPC;  break;
        case TASK_ID_GATTM: task = TASK_GATTM; break;
        case TASK_ID_GATTC: task = TASK_GATTC; break;
        case TASK_ID_L2CC:  task = TASK_L2CC;  break;
        #if (AHI_TL_SUPPORT)
        case TASK_ID_AHI:   task = TASK_AHI;   break;
        #endif // (AHI_TL_SUPPORT)
        #ifdef BLE_AUDIO_AM0_TASK
        case TASK_ID_AM0:   task = TASK_AM0;   break;
        #endif // BLE_AUDIO_AM0_TASK
        default:
        {
            #if (BLE_PROFILES)
            // check if profile manager is able to retrieve the task number
            task = prf_get_task_from_id(id);
					//  task = rom_env.prf_get_task_from_id(id);
            #endif // (BLE_PROFILES)
        }
        break;
    }

    return (kernel_task_check(KERNEL_BUILD_ID(task, idx)));
};

#if (BLE_CENTRAL || BLE_PERIPHERAL)

bool gapm_is_disc_connection(uint8_t conidx)
{
    bool ret = false;

    if(gapm_get_operation(GAPM_OP_AIR) == GAPM_CONNECTION_NAME_REQUEST)
    {
        struct gapm_start_connection_cmd *connect =
                (struct gapm_start_connection_cmd *) gapm_get_operation_ptr(GAPM_OP_AIR);

        // for the moment only name discovery uses a connection
        if((GAPM_GET_OP_STATE(connect->op) == GAPM_OP_NAME_REQ) && (connect->op.addr_src == conidx))
        {
            ret = true;
        }
    }

    return ret;
}

uint16_t gapm_get_max_mtu(void)
{
    return gapm_env.max_mtu;
}

uint16_t gapm_get_max_mps(void)
{
    return gapm_env.max_mps;
}

void gapm_set_max_mtu(uint16_t mtu)
{
    // The MTU value must be within the range [L2C_MIN_LE_MTUSIG:L2C_MIN_LE_MTUSIG]
    gapm_env.max_mtu = common_max(L2C_MIN_LE_MTUSIG, common_min(mtu, GAP_MAX_LE_MTU));
}

void gapm_set_max_mps(uint16_t mps)
{
    // The MPS value must be within the range [L2C_MIN_LE_MTUSIG: MTU]
    gapm_env.max_mps = common_max(L2C_MIN_LE_MTUSIG, common_min(mps, gapm_env.max_mtu));
}
#endif /* (BLE_CENTRAL || BLE_PERIPHERAL) */
uint8_t gapm_addr_check( uint8_t addr_type)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    // Controller and Host privacy are mutually excluded
    if((addr_type == GAPM_CFG_ADDR_PUBLIC) ||
            (addr_type == GAPM_CFG_ADDR_PRIVATE) ||
            (addr_type == GAPM_CFG_ADDR_HOST_PRIVACY) ||
            (addr_type == (GAPM_CFG_ADDR_CTNL_PRIVACY | GAPM_CFG_ADDR_PUBLIC)) ||
            (addr_type == (GAPM_CFG_ADDR_CTNL_PRIVACY | GAPM_CFG_ADDR_PRIVATE)))
    {
        status = GAP_ERR_NO_ERROR;
    }

    return status;
}

uint8_t gapm_dle_val_check(uint16_t sugg_oct, uint16_t sugg_time)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    // Check suggested Data Length parameters
    if ((sugg_oct >= BLE_MIN_OCTETS) &&
            (sugg_oct <= BLE_MAX_OCTETS) &&
            (sugg_time >= BLE_MIN_TIME) &&
            (sugg_time <= BLE_MAX_TIME))
    {
        status = GAP_ERR_NO_ERROR;
    }

    return status;
}

#if (SECURE_CONNECTIONS)
public_key_t* gapm_get_local_public_key(void)
{
    // Returns the local Public Key - X and Y coordinates.
    return &gapm_env.public_key;
}
#endif // (SECURE_CONNECTIONS)


void gapm_update_state(uint8_t operation, bool busy)
{
    kernel_state_t old_state  = kernel_state_get(TASK_GAPM);
    if(old_state != GAPM_DEVICE_SETUP)
    {
        if(busy)
        {
            // set state to busy
            kernel_state_set(TASK_GAPM, old_state | (1 << operation));
        }
        else
        {
            // set state to idle
            kernel_state_set(TASK_GAPM, old_state & ~(1 << operation));
        }
    }
}

bool gapm_is_legacy_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_LEGACY) !=0);
}


bool gapm_is_sec_con_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_SEC_CON) !=0);
}

#ifdef BLE_AUDIO_AM0_TASK
bool gapm_is_audio_am0_sup(void)
{
    return GAPM_F_GET(gapm_env.audio_cfg, AUDIO_AM0_SUP) != 0;
}
#endif // BLE_AUDIO_AM0_TASK



#if (BLE_LECB)

struct gapm_le_psm_info* gapm_le_psm_find(uint16_t le_psm)
{
    // search if LE_PSM is present.
    struct gapm_le_psm_info* info = (struct gapm_le_psm_info*) common_list_pick(&(gapm_env.reg_le_psm));

    // browse register le_psm list
    while(info)
    {
        // check if LE_PSM already registered
        if(info->le_psm == le_psm)
        {
            break;
        }

        // go to next element
        info = (struct gapm_le_psm_info*) info->hdr.next;
    }

    return info;
}


uint8_t gapm_le_psm_get_info(uint16_t le_psm, uint8_t conidx, kernel_task_id_t *app_task, uint8_t *sec_lvl)
{
    uint8_t status = GAP_ERR_NOT_FOUND;
    // search if LE_PSM is present.
    struct gapm_le_psm_info* info = gapm_le_psm_find(le_psm);

    if(info != NULL)
    {
        *sec_lvl  = (info->sec_lvl & GAPM_LE_PSM_SEC_LVL_MASK);
        *app_task = info->task_id;

        // check if target task is multi-instantiated
        if(info->sec_lvl & GAPM_LE_PSM_MI_TASK_MASK)
        {
            *app_task = KERNEL_BUILD_ID(KERNEL_TYPE_GET(info->task_id) , conidx);
        }

        status = GAP_ERR_NO_ERROR;
    }

    return status;
}


uint8_t gapm_lecb_register(uint16_t le_psm, bool peer_con_init)
{
    uint8_t status = L2C_ERR_NO_RES_AVAIL;

    // check that resources are available
    if(gapm_env.nb_lecb < gapm_env.max_nb_lecb)
    {
        status = GAP_ERR_NO_ERROR;
        gapm_env.nb_lecb++;

        // increment number of connection for registerd LE_PSM
        if(peer_con_init)
        {
            struct gapm_le_psm_info* info = gapm_le_psm_find(le_psm);

            if(info != NULL)
            {
                info->nb_est_lk++;
            }
        }
    }

    return (status);
}


uint8_t gapm_lecb_unregister(uint16_t le_psm, bool peer_con_init)
{
    // decrement total number of connection
    gapm_env.nb_lecb--;

    // decrement number of connection for registerd LE_PSM
    if(peer_con_init)
    {
        struct gapm_le_psm_info* info = gapm_le_psm_find(le_psm);

        if(info != NULL)
        {
            info->nb_est_lk--;
        }
    }

    return (GAP_ERR_NO_ERROR);
}


// remove all registered LE_PSM
void gapm_le_psm_cleanup(void)
{
    while(!common_list_is_empty(&(gapm_env.reg_le_psm)))
    {
        struct common_list_hdr * hdr = common_list_pop_front(&(gapm_env.reg_le_psm));
        kernel_free(hdr);
    }
}
#endif // (BLE_LECB)


#if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
bool gapm_is_embedded_host(void)
{
   return gapm_env.embedded_host;
}


void gapm_set_embedded_host(bool enable)
{
   gapm_env.embedded_host = enable;
}
#endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)


uint8_t gapm_get_address_type(void)
{
   return (uint8_t) GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE);
}

#if (BLE_ATTS)

bool gapm_is_pref_con_param_pres(void)
{
    return (bool) GAPM_F_GET(gapm_env.cfg_flags, PREF_CON_PAR_PRES);
}

uint16_t gapm_get_att_handle(uint8_t att_idx)
{
    uint16_t handle = ATT_INVALID_HANDLE;

    if(gapm_env.svc_start_hdl != 0)
    {
        handle = gapm_env.svc_start_hdl + att_idx;

        if((att_idx > GAP_IDX_SLAVE_PREF_PARAM) && !GAPM_F_GET(gapm_env.cfg_flags, PREF_CON_PAR_PRES))
        {
            handle -= 2;
        }
    }

    return handle;
}
#endif // (BLE_ATTS)


bool gapm_svc_chg_en(void)
{
    return (GAPM_F_GET(gapm_env.cfg_flags, SVC_CHG_EN) != 0);
}

#if (BLE_DEBUG)
bool gapm_dbg_mode_en(void)
{
    return (GAPM_F_GET(gapm_env.cfg_flags, DBG_MODE_EN) != 0);
}

void gapm_set_svc_start_hdl(uint16_t start_hdl)
{
    gapm_env.svc_start_hdl = start_hdl;
}
#endif // (BLE_DEBUG)


/// @} GAPM
