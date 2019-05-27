/**
 ****************************************************************************************
 *
 * @file llc_llcp.c
 *
 * @brief Implementation of the functions for control pdu transmission/reception
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCLLCP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"


#if (BLE_PERIPHERAL || BLE_CENTRAL)
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "ble_compiler.h"
#include "common_error.h"
#include "common_endian.h"
#include "common_bt.h"
#include "common_llcp.h"
#include "em_buf.h"
#include "kernel_task.h"
#include "kernel_timer.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_ble_em_rx_desc.h"
#include "reg_ble_em_cs.h"
#include "llm.h"
#include "llm_util.h"
#include "common_version.h"
#include "llc_task.h"
#include "llc_util.h"
#include "llcontrl.h"
#include "llc_llcp.h"
#include "lld_pdu.h"
#include "lld_util.h"
#include "lld.h"
#if (BLE_AUDIO)
#include "audio.h"
#endif // (BLE_AUDIO)
/*
 * MACROS
 ****************************************************************************************
 */
/// retrieve message from pdu in handler
#define LLC_GET_BASE_MSG(pdu) \
    (struct llc_llcp_recv_ind*) (((int32_t) pdu) - (sizeof(struct llc_llcp_recv_ind) - sizeof(union llcp_pdu)))

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// state of instant action
enum llc_instant_state
{
    /// Instant process is ignored
    LLC_INSTANT_IGNORED,
    /// Instant is accepted
    LLC_INSTANT_ACCEPTED,
    /// Instant LMP collision detected
    LLC_INSTANT_COLLISION,
    /// Instant is passed
    LLC_INSTANT_PASSED,
    /// Mic failure detected
    LLC_INSTANT_MIC_FAILURE,
    /// Reject
    LLC_INSTANT_REJECT,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void llc_llcp_send(uint8_t conhdl, void *param, uint8_t opcode);

/// LLCP PDU function handler pointer type definition
typedef int (*llcp_pdu_handler_func_t)(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, union llcp_pdu *pdu);


static int llc_llcp_unknown_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, uint8_t opcode);
static int llcp_con_upd_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_upd_ind *param);
static int llcp_channel_map_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_channel_map_ind *param);
static int llcp_terminate_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_terminate_ind *param);
static int llcp_feats_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_feats_req *param);
static int llcp_feats_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_feats_rsp *param);
static int llcp_vers_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_vers_ind *param);
static int llcp_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_enc_req *param);
static int llcp_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_enc_rsp *param);
static int llcp_start_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_start_enc_req *param);
static int llcp_start_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_start_enc_rsp *param);
static int llcp_pause_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_pause_enc_req *param);
static int llcp_pause_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_pause_enc_rsp *param);
static int llcp_reject_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_reject_ind *param);
static int llcp_unknown_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_unknown_rsp *param);
#if !(BLE_QUALIF)
static int llcp_slave_feature_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_slave_feature_req *param);
static int llcp_con_param_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_param_req *param);
static int llcp_con_param_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_param_rsp *param);
static int llcp_reject_ind_ext_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_reject_ind_ext *param);
static int llcp_ping_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, void *param) ;
static int llcp_ping_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, void *param);
static int llcp_length_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_length_req *param);
static int llcp_length_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_length_rsp *param);
#if (BLE_2MBPS)
static int llcp_phy_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_req *param);
static int llcp_phy_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_rsp *param);
static int llcp_phy_upd_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_upd_req *param);
#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#define HANDLER(pdu) (llcp_pdu_handler_func_t) pdu##_handler

struct llcp_pdu_handler_info
{
    /// Message handler
    llcp_pdu_handler_func_t handler;
    /// Allowed in interrupt context
    bool int_ctx_allowed;
    /// Encryption authorization
    uint8_t enc_auth;
};


///Table for normal LMP PDUs handler utilities
static const struct llcp_pdu_handler_info llcp_pdu_handler[LLCP_OPCODE_MAX_OPCODE] =
{
    [LLCP_CONNECTION_UPDATE_IND_OPCODE] = { HANDLER(llcp_con_upd_ind),         true,   LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_CHANNEL_MAP_IND_OPCODE]       = { HANDLER(llcp_channel_map_ind),     true,   LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_TERMINATE_IND_OPCODE]         = { HANDLER(llcp_terminate_ind),       false,  (LLC_LLCP_START_ENC_AUTHZED|LLC_LLCP_PAUSE_ENC_AUTHZED), },
    [LLCP_ENC_REQ_OPCODE]               = { HANDLER(llcp_enc_req),             false,  LLC_LLCP_START_ENC_AUTHZED,                              },
    [LLCP_ENC_RSP_OPCODE]               = { HANDLER(llcp_enc_rsp),             false,  LLC_LLCP_START_ENC_AUTHZED,                              },
    [LLCP_START_ENC_REQ_OPCODE]         = { HANDLER(llcp_start_enc_req),       false,  LLC_LLCP_START_ENC_AUTHZED,                              },
    [LLCP_START_ENC_RSP_OPCODE]         = { HANDLER(llcp_start_enc_rsp),       false,  LLC_LLCP_START_ENC_AUTHZED,                              },
    [LLCP_UNKNOWN_RSP_OPCODE]           = { HANDLER(llcp_unknown_rsp),         false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_FEATURE_REQ_OPCODE]           = { HANDLER(llcp_feats_req),           false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_FEATURE_RSP_OPCODE]           = { HANDLER(llcp_feats_rsp),           false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_PAUSE_ENC_REQ_OPCODE]         = { HANDLER(llcp_pause_enc_req),       false,  LLC_LLCP_PAUSE_ENC_AUTHZED,                              },
    [LLCP_PAUSE_ENC_RSP_OPCODE]         = { HANDLER(llcp_pause_enc_rsp),       false,  LLC_LLCP_PAUSE_ENC_AUTHZED,                              },
    [LLCP_VERSION_IND_OPCODE]           = { HANDLER(llcp_vers_ind),            false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_REJECT_IND_OPCODE]            = { HANDLER(llcp_reject_ind),          false,  LLC_LLCP_START_ENC_AUTHZED,                              },
#if !(BLE_QUALIF)
    [LLCP_SLAVE_FEATURE_REQ_OPCODE]     = { HANDLER(llcp_slave_feature_req),   false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_CONNECTION_PARAM_REQ_OPCODE]  = { HANDLER(llcp_con_param_req),       false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_CONNECTION_PARAM_RSP_OPCODE]  = { HANDLER(llcp_con_param_rsp),       false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_REJECT_IND_EXT_OPCODE]        = { HANDLER(llcp_reject_ind_ext),      false,  LLC_LLCP_START_ENC_AUTHZED,                              },
    [LLCP_PING_REQ_OPCODE]              = { HANDLER(llcp_ping_req),            false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_PING_RSP_OPCODE]              = { HANDLER(llcp_ping_rsp),            false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_LENGTH_REQ_OPCODE]            = { HANDLER(llcp_length_req),          false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_LENGTH_RSP_OPCODE]            = { HANDLER(llcp_length_rsp),          false,  LLC_LLCP_NO_AUTHZED,                                     },
    #if (BLE_2MBPS)
    [LLCP_PHY_REQ_OPCODE]               = { HANDLER(llcp_phy_req),             false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_PHY_RSP_OPCODE]               = { HANDLER(llcp_phy_rsp),             false,  LLC_LLCP_NO_AUTHZED,                                     },
    [LLCP_PHY_UPD_IND_OPCODE]           = { HANDLER(llcp_phy_upd_ind),         true,   LLC_LLCP_NO_AUTHZED,                                     },
    #endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Allocates and sends the read remote version information pdu
 *
 ****************************************************************************************
 */
void llc_llcp_version_ind_pdu_send(uint16_t conhdl)
{
    struct llcp_vers_ind pdu;

    pdu.opcode = LLCP_VERSION_IND_OPCODE;
    pdu.vers = RWBT_SW_VERSION_MAJOR;
    pdu.compid = common_htobs(RW_COMP_ID);
    pdu.subvers = common_htobs(COMMON_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD));

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}


/**
 ****************************************************************************************
 * @brief Allocates and sends the channel map request pdu
 *
 ****************************************************************************************
 */
void llc_llcp_ch_map_update_pdu_send(uint16_t conhdl)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    struct llcp_channel_map_ind pdu;

    pdu.opcode = LLCP_CHANNEL_MAP_IND_OPCODE;
    pdu.instant = common_htobs(lld_util_instant_get((void*)LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt), (uint8_t)LLD_UTIL_CHMAP_UPDATE));
    memcpy(&pdu.ch_map.map[0], &llc_env_ptr->n_ch_map.map[0], LE_CHNL_MAP_LEN);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the pause encryption request pdu
 *
 ****************************************************************************************
 */
void llc_llcp_pause_enc_req_pdu_send(uint16_t conhdl)
{
    struct llcp_pause_enc_req pdu;
    kernel_task_id_t taskid = KERNEL_BUILD_ID(TASK_LLC, conhdl);

    pdu.opcode = LLCP_PAUSE_ENC_REQ_OPCODE;
    // Start the response timeout
    kernel_timer_set(LLC_LLCP_RSP_TO, taskid, LLC_DFT_RSP_TO);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the pause encryption response pdu
 *
 ****************************************************************************************
 */
void llc_llcp_pause_enc_rsp_pdu_send(uint16_t conhdl)
{
    struct llcp_pause_enc_rsp pdu;

    pdu.opcode = LLCP_PAUSE_ENC_RSP_OPCODE;

    // Disable the encryption in the CS only for the RX part
    ble_link_set(conhdl, ble_link_get(conhdl) & ~BLE_RXCRYPT_EN_BIT);
    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX, false);
    
    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the encryption request pdu
 *
 ****************************************************************************************
 */
void llc_llcp_enc_req_pdu_send(uint16_t conhdl, struct hci_le_start_enc_cmd *param)
{
    struct llcp_enc_req pdu;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    pdu.opcode = LLCP_ENC_REQ_OPCODE;

    common_write16p(&pdu.ediv, common_htobs(param->enc_div));          // EDIV
    memcpy(&pdu.rand.nb[0], &param->nb.nb[0], RAND_NB_LEN);    // Rand
    memcpy(&pdu.ivm.iv[0], &llc_env_ptr->encrypt.randn[0],INIT_VECT_LEN);  // IVm
    memcpy(&pdu.skdm.skdiv[0], &llc_env_ptr->encrypt.randn[INIT_VECT_LEN],SESS_KEY_DIV_LEN);  // SKDm

    // Save the IVm in the control structure
    llc_util_ivm_set(conhdl, pdu.ivm.iv);
    // Save the LTK
    memcpy(&llc_env_ptr->encrypt.ltk.ltk[0], &param->ltk.ltk[0],KEY_LEN);
    // Save the SKDm
    memcpy(&llc_env_ptr->encrypt.skd.skd[0], &pdu.skdm.skdiv[0], SESS_KEY_DIV_LEN);

    // Disable the encryption in the CS only for the RX part
    ble_link_set(conhdl, ble_link_get(conhdl) & ~BLE_RXCRYPT_EN_BIT);
    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX, false);

    // Start the response timeout
    kernel_timer_set(LLC_LLCP_RSP_TO, KERNEL_BUILD_ID(TASK_LLC, conhdl), LLC_DFT_RSP_TO);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the encryption response pdu
 *
 ****************************************************************************************
 */
void llc_llcp_enc_rsp_pdu_send(uint16_t conhdl, struct llcp_enc_req *param)
{
    struct llcp_enc_rsp pdu;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    pdu.opcode = LLCP_ENC_RSP_OPCODE;

    memcpy(&pdu.ivs.iv[0], &llc_env_ptr->encrypt.randn[0],INIT_VECT_LEN);  // IVs
    memcpy(&pdu.skds.skdiv[0], &llc_env_ptr->encrypt.randn[INIT_VECT_LEN],SESS_KEY_DIV_LEN);  // SKDs

    // Save the IVs in the control structure
    llc_util_ivs_set(conhdl, pdu.ivs.iv);
    // Save the SKDs
    memcpy(&llc_env_ptr->encrypt.skd.skd[SESS_KEY_DIV_LEN], &pdu.skds.skdiv[0],
           SESS_KEY_DIV_LEN);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the start encryption response pdu
 *
 ****************************************************************************************
 */
void llc_llcp_start_enc_rsp_pdu_send(uint16_t conhdl)
{
    struct llcp_start_enc_rsp pdu;
    //Get environment pointer
    pdu.opcode = LLCP_START_ENC_RSP_OPCODE;

    // Enable encryption in both RX and TX
    ble_link_set(conhdl, ble_link_get(conhdl) | BLE_TXCRYPT_EN_BIT | BLE_RXCRYPT_EN_BIT);
    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_ENABLE, true);

    lld_util_compute_ce_max(llc_env[conhdl]->elt, llc_env[conhdl]->data_len_ext_info.conn_eff_max_tx_time,
                                llc_env[conhdl]->data_len_ext_info.conn_eff_max_rx_time);
    #if(BLE_AUDIO)
    ble_crypt_mode_setf(conhdl, audio_get_encryption_mode(conhdl));
    #endif
    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Allocates and sends the reject (extended) indication pdu
 *
 ****************************************************************************************
 */
void llc_llcp_reject_ind_pdu_send(uint16_t conhdl, uint8_t rej_opcode, uint8_t reason)
{

    #if !(BLE_QUALIF)
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    // If the features have been exchanged and the peer supports it, use extended reject indication
    // Always use extended reject indication for param_req and param_rsp
    if ((GETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH) && (llc_env_ptr->feats_used.feats[0] & BLE_REJ_IND_EXT_FEATURE)) ||
        (rej_opcode > LLCP_REJECT_IND_OPCODE))
    {
        struct llcp_reject_ind_ext pdu;

        pdu.opcode = LLCP_REJECT_IND_EXT_OPCODE;
        pdu.err_code = reason;
        pdu.rej_opcode = rej_opcode;

        llc_llcp_send(conhdl, &pdu, pdu.opcode);
    }
    else
    #endif
    {
        struct llcp_reject_ind pdu;

        pdu.opcode = LLCP_REJECT_IND_OPCODE;
        pdu.err_code = reason;
        if((rej_opcode ==LLCP_START_ENC_REQ_OPCODE) || (rej_opcode == LLCP_ENC_REQ_OPCODE))
        {
            // Transmission of the LL_REJECT_IND terminates the Encryption Start Procedure
            LLC_UTIL_ENC_STATE_SET(conhdl, LLC_ENC_DISABLED);
        }

        llc_llcp_send(conhdl, &pdu, pdu.opcode);
    }
}

/**
 ****************************************************************************************
 * @brief Sends the connection update request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_con_update_pdu_send(uint16_t conhdl, struct llcp_con_upd_ind *param)
{
    struct llcp_con_upd_ind pdu;

    pdu.opcode = LLCP_CONNECTION_UPDATE_IND_OPCODE;
    pdu.win_size = param->win_size;
    pdu.win_off  = common_htobs(param->win_off);
    pdu.interv   = common_htobs(param->interv);
    pdu.latency  = common_htobs(param->latency);
    pdu.timeout  = common_htobs(param->timeout);
    pdu.instant  = common_htobs(param->instant);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Sends the connection parameters request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_con_param_req_pdu_send(uint16_t conhdl, struct llc_con_upd_req_ind *param)
{
    struct llcp_con_param_req pdu;

    pdu.opcode = LLCP_CONNECTION_PARAM_REQ_OPCODE;
    pdu.interval_min =  common_htobs(param->interval_min);
    pdu.interval_max =  common_htobs(param->interval_max);
    pdu.latency = common_htobs(param->con_latency);
    pdu.timeout = common_htobs(param->superv_to);
    pdu.pref_period = param->pref_period;
    pdu.ref_con_event_count = common_htobs(param->ref_con_event_count);
    pdu.offset0 = common_htobs(param->offset0);
    pdu.offset1 = common_htobs(param->offset1);
    pdu.offset2 = common_htobs(param->offset2);
    pdu.offset3 = common_htobs(param->offset3);
    pdu.offset4 = common_htobs(param->offset4);
    pdu.offset5 = common_htobs(param->offset5);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the connection parameters response pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_con_param_rsp_pdu_send(uint16_t conhdl, struct llc_con_upd_req_ind *param)
{
    struct llcp_con_param_rsp pdu;

    pdu.opcode              = LLCP_CONNECTION_PARAM_RSP_OPCODE;
    pdu.interval_min        = common_htobs(param->interval_min);
    pdu.interval_max        = common_htobs(param->interval_max);
    pdu.latency             = common_htobs(param->con_latency);
    pdu.timeout             = common_htobs(param->superv_to);
    pdu.pref_period         = param->pref_period;
    pdu.ref_con_event_count = common_htobs(param->ref_con_event_count);
    pdu.offset0             = common_htobs(param->offset0);
    pdu.offset1             = common_htobs(param->offset1);
    pdu.offset2             = common_htobs(param->offset2);
    pdu.offset3             = common_htobs(param->offset3);
    pdu.offset4             = common_htobs(param->offset4);
    pdu.offset5             = common_htobs(param->offset5);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}
#endif
/**
 ****************************************************************************************
 * @brief Sends the features request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_feats_req_pdu_send(uint16_t conhdl)
{
    struct llcp_feats_req pdu;

    if(lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
    {
        pdu.opcode = LLCP_FEATURE_REQ_OPCODE;
    }
    #if !(BLE_QUALIF)
    else
    {
        pdu.opcode = LLCP_SLAVE_FEATURE_REQ_OPCODE;
    }
    #endif
        // Get the supported features
    llm_util_get_supp_features(&pdu.feats);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the features response pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_feats_rsp_pdu_send(uint16_t conhdl)
{
    struct llcp_feats_rsp pdu;

    pdu.opcode = LLCP_FEATURE_RSP_OPCODE;
    // Get the supported features
    llm_util_get_supp_features(&pdu.feats);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}


/**
 ****************************************************************************************
 * @brief Sends the start encryption request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_start_enc_req_pdu_send(uint16_t conhdl)
{
    struct llcp_start_enc_req pdu;

    // Enable encryption in RX only
    ble_link_set(conhdl, ble_link_get(conhdl) | BLE_RXCRYPT_EN_BIT);
    LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX, true);

    #if(BLE_AUDIO)
    ble_crypt_mode_setf(conhdl, audio_get_encryption_mode(conhdl));
    #endif

    // Initialize the CCM counters
    ble_txccmpktcnt0_set(conhdl,0);
    ble_txccmpktcnt1_set(conhdl,0);
    ble_txccmpktcnt2_set(conhdl,0);
    ble_rxccmpktcnt0_set(conhdl,0);
    ble_rxccmpktcnt1_set(conhdl,0);
    ble_rxccmpktcnt2_set(conhdl,0);

    pdu.opcode = LLCP_START_ENC_REQ_OPCODE;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the terminate indication pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_terminate_ind_pdu_send(uint16_t conhdl, uint8_t err_code)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    struct llcp_terminate_ind pdu;
    kernel_task_id_t taskid = KERNEL_BUILD_ID(TASK_LLC, conhdl);
    kernel_state_t state = kernel_state_get(taskid);

    // Set the reason of the disconnection
    switch(err_code)
    {
        case COMMON_ERROR_REMOTE_USER_TERM_CON:
        {
            llc_env_ptr->disc_reason = COMMON_ERROR_CON_TERM_BY_LOCAL_HOST;
        }break;
        default:
        {
            llc_env_ptr->disc_reason = err_code;
        }break;
    }

    pdu.opcode = LLCP_TERMINATE_IND_OPCODE;
    pdu.err_code = err_code;

    // Set the state of the LLC
    llc_state_update(taskid, &state, LLC_DISC_BUSY, true);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);

    // Vol 6 Part B - 5.1.6 Termination Procedure
    //
    // The Link Layer shall start a timer, Tterminate, when the LL_TERMINATE_IND
    // PDU has been queued for transmission. The initiating Link Layer shall send
    // LL_TERMINATE_IND PDUs until an acknowledgment is received or until the
    // timer, Tterminate, expires, after which it shall exit the Connection State and
    // transition to the Standby State. The initial value for Tterminate shall be set to
    // value of the connSupervisionTimeout.
    kernel_timer_set(LLC_LLCP_RSP_TO, taskid, llc_env_ptr->sup_to);
}

/**
 ****************************************************************************************
 * @brief Sends the unknown response pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_unknown_rsp_send_pdu(uint16_t conhdl, uint8_t unk_type)
{
    struct llcp_unknown_rsp pdu;

    pdu.opcode = LLCP_UNKNOWN_RSP_OPCODE;
    pdu.unk_type = unk_type;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}
#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Sends the ping request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_ping_req_pdu_send(uint16_t conhdl)
{
    struct llcp_ping_req pdu;

    pdu.opcode = LLCP_PING_REQ_OPCODE;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the ping response pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_ping_rsp_pdu_send(uint16_t conhdl)
{
    struct llcp_ping_rsp pdu;

    pdu.opcode = LLCP_PING_RSP_OPCODE;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the length request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_length_req_pdu_send(uint16_t conhdl)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    struct llcp_length_req pdu;

    pdu.opcode = LLCP_LENGTH_REQ_OPCODE;
    pdu.max_rx_octets = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_rx_octets);
    pdu.max_rx_time = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_rx_time);
    pdu.max_tx_octets = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_tx_octets);
    pdu.max_tx_time = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_tx_time);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

/**
 ****************************************************************************************
 * @brief Sends the length request pdu.
 *
 ****************************************************************************************
 */
void llc_llcp_length_rsp_pdu_send(uint16_t conhdl)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    struct llcp_length_rsp pdu;

    pdu.opcode = LLCP_LENGTH_RSP_OPCODE;
    pdu.max_rx_octets = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_rx_octets);
    pdu.max_rx_time = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_rx_time);
    pdu.max_tx_octets = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_tx_octets);
    pdu.max_tx_time = common_htobs(llc_env_ptr->data_len_ext_info.conn_max_tx_time);

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

#if (BLE_2MBPS)
void llc_llcp_phy_req_pdu_send(uint16_t conhdl, struct  llcp_phy_req *param)
{
    struct llcp_phy_req pdu;

    pdu.opcode = LLCP_PHY_REQ_OPCODE;
    pdu.rx_phys = param->rx_phys;
    pdu.tx_phys = param->tx_phys;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

void llc_llcp_phy_rsp_pdu_send(uint16_t conhdl, struct  llcp_phy_rsp *param)
{
    struct llcp_phy_rsp pdu;

    pdu.opcode  = LLCP_PHY_RSP_OPCODE;
    pdu.rx_phys = param->rx_phys;
    pdu.tx_phys = param->tx_phys;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}

void llc_llcp_phy_upd_ind_pdu_send(uint16_t conhdl,struct  llcp_phy_upd_req *param)
{
    struct llcp_phy_upd_req pdu;

    pdu.opcode      = LLCP_PHY_UPD_IND_OPCODE;
    pdu.m_to_s_phy  = param->m_to_s_phy;
    pdu.s_to_m_phy  = param->s_to_m_phy;
    pdu.instant     = param->instant;

    llc_llcp_send(conhdl, &pdu, pdu.opcode);
}
#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the reception of the control pdu
 *
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Sends the programmed LLCP PDU on the given connection handle.
 *
 * @param[in] conhdl  The connection handle on which the LLCP has to be transmitted
 * @param[in] param   Pointer on the PDU to be sent
 * @param[in] opcode  The LLCP opcode is directly passed to avoid structure alignment issue
 *
 ****************************************************************************************
 */
static void llc_llcp_send(uint8_t conhdl, void *param, uint8_t opcode)
{

    uint8_t llcp_length = sizeof(union llcp_pdu);
    // LLCP can be received, so allocate a message structure
    void *llcp_ptr= kernel_malloc(llcp_length, KERNEL_MEM_KERNEL_MSG);

    struct llcp_pdu_tag *llcp_elt = (struct llcp_pdu_tag*)kernel_malloc(sizeof(struct llcp_pdu_tag), KERNEL_MEM_ENV);

    if(llcp_ptr && llcp_elt)
    {
        // Get associated BLE event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt);
        // Get list of data packets ready for programming
        struct common_list *list = &evt->tx_llcp_pdu_rdy;

        // Copy the llcp pdu in a heap
        memcpy(llcp_ptr, param,llcp_length);
        // Fill the element before pushing in the list
        llcp_elt->ptr = llcp_ptr;
        llcp_elt->idx = conhdl;
        llcp_elt->opcode = opcode;

        GLOBAL_INT_DIS();
        // Push the llcp pdu allocated at the end of the TX llcp pending list
        common_list_push_back(list, &llcp_elt->hdr);
        GLOBAL_INT_RES();

        #if(BLE_PERIPHERAL)
        // Schedule the next event as soon as possible
        lld_evt_schedule_next( llc_env[conhdl]->elt);
        #endif //(BLE_PERIPHERAL)
    }
    else
    {
        ASSERT_INFO(0, (llcp_ptr && llcp_elt), llcp_length);
    }
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP reject indication (Extended or not)
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] extended True if it's a LLCP reject extended indication; false else
 ****************************************************************************************
 */
static void llc_llcp_reject_ind(uint16_t conhdl, kernel_task_id_t const dest_id, uint8_t err_code, bool extended)
{
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Indicate if reception of the message was expected
    bool is_expected = true;
    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
    if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
    {
        // Check state
        switch (llc_env_ptr->loc_proc_state)
        {
            case LLC_LOC_WAIT_ENC_RSP:
            case LLC_LOC_WAIT_SK_AND_START_ENC_REQ:
            case LLC_LOC_WAIT_SK:
            case LLC_LOC_WAIT_START_ENC_REQ:
            {
                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

                // We can accept this PDU only if we are performing the initial Encryption
                // Start Procedure, because otherwise the slave should have sent a terminate
                // indication


                if ((lld_get_mode(conhdl) == LLD_EVT_MST_MODE) && (!(LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_REFRESH_PENDING))))
                {
                    // Reception of the LL_REJECT_IND terminates the Encryption Start Procedure
                    LLC_UTIL_ENC_STATE_SET(conhdl, LLC_ENC_DISABLED);

                    // Notify the host of the Encryption Start Failure
                    llc_common_enc_change_evt_send(conhdl, 0, err_code);

                    // Go back to the connected state
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;

                    llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, false);
                }
            } break;

            case LLC_LOC_WAIT_CON_PARAM_RSP:
            {
                // 4.1 feature, REJECT_IND_EXT must be used
                if (!extended || (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF)))
                {
                    is_expected = false;
                    break;
                }
                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                #if (BLE_CENTRAL)
                if ((err_code == COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE) && (lld_get_mode(conhdl) == LLD_EVT_MST_MODE))
                {
                    // Get associated BLE Event environment
                    struct llc_con_upd_req_ind *param_req =
                            (struct llc_con_upd_req_ind *) llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);

                    // reschedule the message
                    param_req->operation = LLC_CON_UP_FORCE;
                    kernel_msg_send(param_req);
                }
                else
                #endif // (BLE_CENTRAL)
                {
                    // Check if we have to send the command complete event. It is done only if the update
                    // was requested by the host or if any of the connection parameters has changed
                    if (GETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ))
                    {
                        SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ, false);

                        // Checks if the event is not filtered
                        if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT))
                        {
                            struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);
                            // Send the LE META event connection update to the host
                            llc_con_update_complete_send(err_code, conhdl, evt);
                        }
                    }

                    // Clear Operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);
                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                }
            } break;

            case LLC_LOC_WAIT_CON_UPD_INSTANT: //Case where the Slave has received a channel map request PDU after started a connection on param request procedure (COLLISION)
            {
                /* Ignore the error apply anyway the new interval */
            } break;
            case LLC_LOC_WAIT_PING_RSP:
            {
                // Set the state of the LLC
                llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
            } break;
            case LLC_LOC_WAIT_MAP_UPD_INSTANT:
            {
                /* Ignore the error apply anyway the new channel map */
            } break;
            #if (!BLE_QUALIF)
            #if (BLE_2MBPS)
            case LLC_LOC_WAIT_PHY_UPD_INSTANT:
            {
                /* Ignore the error apply anyway the new interval */
            } break;
            case LLC_LOC_WAIT_PHY_RSP:
            case LLC_LOC_WAIT_PHY_UPD_REQ:
            {
                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);

                //  REJECT_IND_EXT must be used
                if (!extended || (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF)))
                {
                    is_expected = false;
                    break;
                }

                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                {
                    struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), llc_phy_upd_req_ind);
                    phy_update->operation   = LLC_PHY_UPD_TERMINATE;
                    phy_update->rx_phys     = evt->evt.conn.rx_phy;
                    phy_update->tx_phys     = evt->evt.conn.tx_phy;
                    phy_update->status      = err_code;
                    kernel_msg_send(phy_update);
                }

            } break;
            #endif // (BLE_2MBPS)

            case LLC_LOC_WAIT_LENGTH_RSP:
            {
                if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
                {
                    is_expected = false;
                    break;
                }

                // If we are slave and the master answered rejected_extended, reset the LLCP response TO

                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

                // Set the state of the LLC
                llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
				
            } break;
            #endif //(!BLE_QUALIF)

            default:
            {
                // The packet has been received in an unexpected state
                is_expected = false;
            } break;
        }
    }
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        // 4.1 feature, REJECT_IND_EXT must be used
        if (!extended || LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
        {
            is_expected = false;
        }
        else if( (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_CON_UPD_REQ)
                #if (BLE_2MBPS)
                || (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PHY_UPD_INSTANT)
                || (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PHY_UPD_REQ)
                #endif // (BLE_2MBPS)
                )

        {
            #if (BLE_2MBPS)
            if (llc_env_ptr->rem_proc_state != LLC_REM_WAIT_PHY_UPD_INSTANT)
            #endif // (BLE_2MBPS)
            {
                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
            }
            // Set the state of the LLC
            llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, false);
            llc_env_ptr->rem_proc_state = LLC_REM_IDLE;
        }
    }
    else
    {
        // The packet has been received in an unexpected state
        is_expected = false;
    }

    if (!is_expected)
    {
        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
        {
            // This unexpected packet has been received during an Encryption Start
            // or an Encryption Pause Procedure, so we need to exit the connection
            // state immediately with reason "MIC Failure".
            llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
        }
    }
}




#if (BLE_TESTER)
void llc_llcp_tester_send(uint8_t conhdl, uint8_t length, uint8_t *data)
{
    uint8_t llcp_length = sizeof(union llcp_pdu);
    // LLCP can be received, so allocate a message structure
    void *llcp_ptr= kernel_malloc(llcp_length, KERNEL_MEM_KERNEL_MSG);

    struct llcp_pdu_tag *llcp_elt = (struct llcp_pdu_tag*)kernel_malloc(sizeof(struct llcp_pdu_tag), KERNEL_MEM_ENV);

    if(llcp_ptr && llcp_elt)
    {
        // Get associated BLE event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt);
        // Get list of data packets ready for programming
        struct common_list *list = &evt->tx_llcp_pdu_rdy;

        // Copy the llcp pdu in a heap
        memcpy(llcp_ptr, data, length);
        // Fill the element before pushing in the list
        llcp_elt->ptr = llcp_ptr;
        llcp_elt->idx = conhdl;
        llcp_elt->opcode = LLCP_OPCODE_DEBUG;
        llcp_elt->pdu_length = length;
        GLOBAL_INT_DIS();
        // Push the llcp pdu allocated at the end of the TX llcp pending list
        common_list_push_back(list, &llcp_elt->hdr);
        GLOBAL_INT_RES();
    }
    else
    {
        ASSERT_INFO(0, (llcp_ptr && llcp_elt), llcp_length);
    }
}
#endif // BLE_TESTER


int llc_llcp_recv_handler(kernel_task_id_t dest_id, uint8_t status, union llcp_pdu* pdu, bool int_ctx)
{
    int msg_status = int_ctx ? KERNEL_MSG_SAVED : KERNEL_MSG_CONSUMED;
    uint8_t state  = kernel_state_get(dest_id);

    // check if state is Free or in disconnected state
    if(   llc_state_chk(state, LLC_FREE)
       || (llc_state_chk(state, LLC_DISC_BUSY)
               && (pdu->opcode != LLCP_TERMINATE_IND_OPCODE) && (pdu->opcode != LLCP_START_ENC_RSP_OPCODE)))
    {
        // Discard the message
    }
    else
    {
        uint8_t conhdl = KERNEL_IDX_GET(dest_id);
        struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

        // Check if LLC messages have to be discarded
        if (GETF(llc_env_ptr->llc_status, LLC_STAT_LLCP_DISCARD))
        {
            // discard the message
        }
        // valid llcp pdu
        else if (status == COMMON_ERROR_NO_ERROR)
        {
            ASSERT_INFO(pdu->opcode < LLCP_OPCODE_MAX_OPCODE, dest_id, pdu->opcode);

            // check if process of message is allowed in interrupt context
            if(int_ctx && (!llcp_pdu_handler[pdu->opcode].int_ctx_allowed))
            {
                // ignore process of message
            }
            else
            {
                // execute message handler
                msg_status = llcp_pdu_handler[pdu->opcode].handler(conhdl, dest_id, int_ctx, pdu);
            }
        }
        // unknown pdu
        else if ((status == COMMON_ERROR_UNKNOWN_LMP_PDU) && !int_ctx)
        {
            msg_status = llc_llcp_unknown_ind_handler(conhdl, dest_id, pdu->opcode);
        }
        // invalid length
        else
        {
            // ignore pdu
        }
    }

    return (msg_status);
}


/*
 * LLCP Message handler
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the reception of a unknown LLCP
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llc_llcp_unknown_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, uint8_t opcode)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    
    // check if RX Flow is off
    if(LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else
    {
        llc_llcp_unknown_rsp_send_pdu(conhdl, opcode);
    }

    return(msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP connection parameter update request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */	
static int llcp_con_upd_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_upd_ind *param)
{
	//UART_PRINTF("%s\r\n", __func__);
	
    // Message status
    int msg_status  = int_ctx ? KERNEL_MSG_SAVED : KERNEL_MSG_CONSUMED;

    #if (BLE_PERIPHERAL)
    // retrieve origin message
    struct llc_llcp_recv_ind* msg = LLC_GET_BASE_MSG(param);


    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint8_t state = kernel_state_get(dest_id);

    if(int_ctx) // true
    {
        // mark instant ignored
        msg->dummy = LLC_INSTANT_IGNORED;
		//UART_PRINTF("1\r\n");
    }

    if(msg->dummy == LLC_INSTANT_IGNORED)
    {
        // Check if we are allowed to receive this packet
        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
        {
            // This packet has been received during an Encryption Start
            // or an Encryption Pause Procedure, so we need to exit the connection
            // state immediately with reason "MIC Failure".
            msg->dummy = LLC_INSTANT_MIC_FAILURE;
			//UART_PRINTF("2\r\n");
        }
        // Instant operation is already on-going
        else if (lld_util_instant_ongoing(llc_env_ptr->elt))
        {
            msg->dummy = LLC_INSTANT_COLLISION;
			//UART_PRINTF("3\r\n");
        }
        //This LLCP request shall be issued by the master only
        else if (lld_get_mode(conhdl) != LLD_EVT_SLV_MODE)
        {
            // Not allowed reject
            msg->dummy = LLC_INSTANT_REJECT;
			///UART_PRINTF("4\r\n");
        }
        else
        {
        	//UART_PRINTF("5\r\n");
            uint16_t instant = common_btohs(param->instant);
            struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);
			 
            // Check if instant has passed
            if(((uint16_t)((instant - evt->evt.conn.counter) % 65536) < 32767))
            {
                llc_env_ptr->n_sup_to = common_btohs(param->timeout);

                // Instant has not passed, so program the LLD to schedule the parameter update
                lld_con_update_ind(llc_env_ptr->elt, param);

                msg->dummy = LLC_INSTANT_ACCEPTED;
				//UART_PRINTF("6\r\n");
            }
            else
            {
                msg->dummy = LLC_INSTANT_PASSED;
				//UART_PRINTF("7\r\n");
            }
        }
    }

    if(!int_ctx)
    {		
    	//UART_PRINTF("8\r\n");
        // execute state update according to retrieved info
        switch(msg->dummy)
        {
            case LLC_INSTANT_IGNORED: /* Nothing to do */ break;
            case LLC_INSTANT_PASSED:
            {
                // Connection is considered lost, the Link Layer does not send any further packets
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_INSTANT_PASSED);
				//UART_PRINTF("9\r\n");
            } break;
            case LLC_INSTANT_MIC_FAILURE:
            {
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
				//UART_PRINTF("10\r\n");
            } break;
            case LLC_INSTANT_COLLISION:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
				//UART_PRINTF("11\r\n");
            } break;
            case LLC_INSTANT_REJECT:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED);
				//UART_PRINTF("12\r\n");
            } break;
            case LLC_INSTANT_ACCEPTED:
            {
				//UART_PRINTF("13\r\n");
                // check if connection param is initiated by local device
                if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                    && (   (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_CON_PARAM_RSP)
                        || (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_CON_UPD_REQ)))
                {

                    // Stop the LLCP Response TO
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                    // Go back to IDLE state
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    // Clear Operation
                    llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);
                }

                if(llc_env_ptr->rem_proc_state == LLC_REM_WAIT_CON_UPD_REQ)
                {
                    // Stop the LLCP Response TO
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
					//UART_PRINTF("15\r\n");
                }

                // instant proceed and finished, nothing to do
                if(GETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED))
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED, 0);
					//UART_PRINTF("16\r\n");
                }
                else
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 1);
                    llc_env_ptr->rem_proc_state = LLC_REM_WAIT_CON_UPD_INSTANT;
                    llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
					//UART_PRINTF("17\r\n");
                }
            } break;
            default:
            {
                ASSERT_INFO(0, msg->dummy, dest_id);
            }break;
        }
    }
    #endif //(BLE_PERIPHERAL)

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP channel map update request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_channel_map_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_channel_map_ind *param)
{
    // Message status
    int msg_status  = int_ctx ? KERNEL_MSG_SAVED : KERNEL_MSG_CONSUMED;

    #if (BLE_PERIPHERAL)
    // retrieve origin message
    struct llc_llcp_recv_ind* msg = LLC_GET_BASE_MSG(param);

    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    if(int_ctx)
    {
        // mark instant ignored
        msg->dummy = LLC_INSTANT_IGNORED;
    }

    if(msg->dummy == LLC_INSTANT_IGNORED)
    {
        // Check if we are allowed to receive this packet
        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
        {
            // This packet has been received during an Encryption Start
            // or an Encryption Pause Procedure, so we need to exit the connection
            // state immediately with reason "MIC Failure".
            msg->dummy = LLC_INSTANT_MIC_FAILURE;
        }
        // Instant operation is already on-going
        else if (lld_util_instant_ongoing(llc_env_ptr->elt))
        {
            msg->dummy = LLC_INSTANT_COLLISION;
        }
        // This LLCP request shall be issued by the master only
        else if ((lld_get_mode(conhdl) != LLD_EVT_SLV_MODE)
                || (llc_state_chk(state, LLC_REM_PROC_BUSY)))
        {
            // Not allowed reject
            msg->dummy = LLC_INSTANT_REJECT;
        }
        else
        {
            uint16_t instant = common_btohs(param->instant);
            struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);

            // Checks if instant is passed.
            if(((uint16_t)((instant - evt->evt.conn.counter) % 65536) < 32767))
            {
                // Copy the new channel map from the PDU parameters
                llc_env_ptr->n_ch_map = param->ch_map;

                // Instant has not passed, so program the LLD to schedule the parameter update
                lld_ch_map_ind(llc_env_ptr->elt, instant);

                msg->dummy = LLC_INSTANT_ACCEPTED;
            }
            else
            {
                msg->dummy = LLC_INSTANT_PASSED;
            }
        }
    }

    if(!int_ctx)
    {
        // execute state update according to retrieved info
        switch(msg->dummy)
        {
            case LLC_INSTANT_IGNORED: /* Nothing to do */ break;
            case LLC_INSTANT_PASSED:
            {
                // Connection is considered lost, the Link Layer does not send any further packets
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_INSTANT_PASSED);
            } break;
            case LLC_INSTANT_MIC_FAILURE:
            {
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
            } break;
            case LLC_INSTANT_COLLISION:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
            } break;
            case LLC_INSTANT_REJECT:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED);
            } break;
            case LLC_INSTANT_ACCEPTED:
            {
                // instant proceed and finished, nothing to do
                if(GETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED))
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED, 0);
                }
                else
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 1);
                    llc_env_ptr->rem_proc_state = LLC_REM_WAIT_MAP_UPD_INSTANT;
                    llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
                }
            }break;
            default:
            {
                ASSERT_INFO(0, msg->dummy, dest_id);
            }break;
        }
    }
    #endif //(BLE_PERIPHERAL)
    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP terminate indication
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_terminate_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_terminate_ind *param)
{
    // Clears the LLCP response timeout in case a LLCP procedure is currently ongoing
    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

    // Save the reason of the disconnection
    llc_env[conhdl]->disc_reason = param->err_code;
    //Set the flag
    SETF(llc_env[conhdl]->llc_status, LLC_STAT_DISC_REM_REQ, true);
    // We are master, so we have to wait for one additional event before disconnecting
    // in order to send the acknowledgment
    kernel_state_set(dest_id, LLC_DISC_BUSY);

    // slave could be disconnected immediately
    if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
    {
        llc_util_dicon_procedure(conhdl, param->err_code);
    }

    return(KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_feats_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_feats_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    #if (BLE_TESTER)
    if (!(llc_env_ptr->tester_params.tester_feats & LLC_TESTER_IGNORE_FEAT_REQ))
    #endif // (BLE_TESTER)
    {
        // Check if we are allowed to receive this packet
        if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF)))
        {
            // This packet has been received during an Encryption Start
            // or an Encryption Pause Procedure, so we need to exit the connection
            // state immediately with reason "MIC Failure".
            llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
        }
        else
        {
            // LLC is READY to receive commands, process it immediately
            // Merge local features and peer features
        	uint8_t idx=0 ;
            for( idx=0 ; idx < LE_FEATS_LEN ; idx++)
            {
                llc_env_ptr->feats_used.feats[idx] &= param->feats.feats[idx];
            }

            // Features have been exchanged
            SETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH, true);

            // Send the Feature Response PDU
            llc_llcp_feats_rsp_pdu_send(conhdl);
        }
    }
    return(msg_status);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP Slave feature request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_slave_feature_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_slave_feature_req *param)
{
    // use same handler used by feature request
    return llcp_feats_req_handler(conhdl, dest_id, int_ctx, (struct llcp_feats_req *)param);
}
#endif // !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_feats_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_feats_rsp *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // local procedure is busy
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_FEAT_RSP))
    {
        kernel_task_id_t taskid = KERNEL_BUILD_ID(TASK_LLC, conhdl);

        // Clear response time out
        kernel_timer_clear(LLC_LLCP_RSP_TO, taskid);

        // Merge local features and peer features
        uint8_t idx=0 ;
        for( idx=0 ; idx < LE_FEATS_LEN ; idx++)
        {
            llc_env_ptr->feats_used.feats[idx] &= param->feats.feats[idx];
        }

        // Features have been exchanged
        SETF(llc_env_ptr->llc_status, LLC_STAT_FEAT_EXCH, true);

        // Send the meta event with the values received from the peer
        llc_feats_rd_event_send(COMMON_ERROR_NO_ERROR, conhdl, &param->feats);

        // Go back to IDLE state
        llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
        llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
    }
    else
    {
        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED);
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_vers_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_vers_ind *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // check if waiting for TX ACK
    else
    {
        // Handle the reception of the LL_VERSION_IND
        bool peer_vers_known = GETF(llc_env_ptr->llc_status, LLC_STAT_PEER_VERS_KNOWN);

        // Save the peer version in our environment
        llc_env_ptr->peer_version.vers    = param->vers;
        llc_env_ptr->peer_version.compid  = param->compid;
        llc_env_ptr->peer_version.subvers = param->subvers;
        SETF(llc_env_ptr->llc_status, LLC_STAT_PEER_VERS_KNOWN, true);

        if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_VERS_IND))
        {
            // We got the response for the version indication procedure we initiated
            kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

            // Send the command complete event
            llc_version_rd_event_send(COMMON_ERROR_NO_ERROR, conhdl);

            // Go back to IDLE state
            llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
        }
        else if (!peer_vers_known)
        {
            // The procedure is initiated by the peer, reply and simply wait for Ack
            llc_llcp_version_ind_pdu_send(conhdl);
        }
    }

    return(msg_status);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP connection parameters request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_con_param_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_param_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // check if remote procedure is on-going
    else if (llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
    }
    else
    {
        bool reject = false;
        if (lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
        {
            // check if master reject negociation
            if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
            {
                switch(llc_env_ptr->loc_proc_state)
                {
                    case LLC_LOC_WAIT_CON_PARAM_RSP:
                    case LLC_LOC_WAIT_CON_UPD_INSTANT:
                    {
                        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
                        reject = true;
                    } break;
                    #if(BLE_2MBPS)
                    case LLC_LOC_WAIT_PHY_RSP:
                    case LLC_LOC_WAIT_PHY_UPD_REQ:
                    {
                        //If slave side do not reject peer procedure
                        if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
                        {
                            break;
                        }
                    }
                    //no Break
                    case LLC_LOC_WAIT_PHY_UPD_INSTANT:
                    #endif // (BLE_2MBPS)
                    case LLC_LOC_WAIT_MAP_UPD_INSTANT:
                    {
                        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_DIFF_TRANSACTION_COLLISION);
                        reject = true;
                    } break;

                    default: /* Nothing to do */ break;
                }
            }
        }

        if(!reject)
        {
            // LLC is READY to receive commands, process it immediately
            struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);

            if (   (param->interval_min > param->interval_max) || (param->latency > LLC_CNX_LATENCY_MAX)
                #if(BLE_AUDIO)
                || (param->interval_max > LLC_CNX_INTERVAL_MAX)|| (param->interval_min < AUDIO_MIN_INTERVAL)
                #else
                || (param->interval_max > LLC_CNX_INTERVAL_MAX)|| (param->interval_min < LLC_CNX_INTERVAL_MIN)
                #endif // (BLE_AUDIO)
                || (param->timeout > LLC_CNX_SUP_TO_MAX)       || (param->timeout < LLC_CNX_SUP_TO_MIN)
                // CSA/ESR6 : supervision timeout minimum value does not apply for connection request with a 4.0 device.
                // so supervision timeout must be <=  (1 + Conn_Latency) * Conn_Interval_Max *2
                // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
                // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
                // (sup_to * 10) <= ((1+latency)* con_interval*1.25*2)
                // to simplify computation and remove floating point we factor everything by 2/5
                // (hci_sup_to * 4) <= ((1+hci_latency)* hci_interval)
               || ((((uint32_t) param->timeout) <<2) <= ((1 + ((uint32_t)param->latency)) * ((uint32_t)param->interval_max))))
            {
                llc_llcp_reject_ind_pdu_send(conhdl, LLCP_CONNECTION_PARAM_REQ_OPCODE, COMMON_ERROR_INVALID_LMP_PARAM);
            }
            // check if connection parameter are changed
            else if (   (param->interval_min > (evt->interval / 2))
                     || (param->interval_max < (evt->interval / 2))
                     || (param->latency      != (evt->evt.conn.latency - 1))
                     || (param->timeout      != llc_env_ptr->sup_to))
            {
                //If Event not masked
                if(llm_util_check_evt_mask(LE_REM_CON_PARA_REQ_EVT_BIT))
                {
                    // save requested parameters
                    struct llc_con_upd_req_ind *req_param = KERNEL_MSG_ALLOC(LLC_CON_UPD_REQ_IND, dest_id, dest_id, llc_con_upd_req_ind);

                    req_param->operation           = LLC_CON_UP_PEER_REQ;
                    req_param->interval_min        = param->interval_min;
                    req_param->interval_max        = param->interval_max;
                    req_param->con_latency         = param->latency;
                    req_param->superv_to           = param->timeout;
                    req_param->pref_period         = param->pref_period;
                    req_param->ref_con_event_count = param->ref_con_event_count;
                    req_param->offset0             = param->offset0;
                    req_param->offset1             = param->offset1;
                    req_param->offset2             = param->offset2;
                    req_param->offset3             = param->offset3;
                    req_param->offset4             = param->offset4;
                    req_param->offset5             = param->offset5;
                    kernel_msg_send(req_param);

                }
                /*
                 * If the request is being indicated to the Host and the event to the Host is
                 * masked, then the Link Layer shall issue an LL_REJECT_IND_EXT PDU with
                 * Error Code 0x1A
                 */
                else
                {
                    llc_llcp_reject_ind_pdu_send(conhdl, LLCP_CONNECTION_PARAM_REQ_OPCODE, COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE);
                }
            }
            // it's an anchor point move request
            else
            {   // Request handled by the controller
                if (lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
                {
                    // request parameter update
                    struct llc_con_upd_req_ind *req_param = KERNEL_MSG_ALLOC(LLC_CON_UPD_REQ_IND, dest_id, dest_id, llc_con_upd_req_ind);

                    req_param->operation           = LLC_CON_UP_FORCE;
                    req_param->con_intv_min        = param->interval_min;
                    req_param->con_intv_max        = param->interval_max;
                    req_param->superv_to           = llc_env_ptr->sup_to;
                    req_param->ce_len_min          = llc_env_ptr->elt->duration_min/SLOT_SIZE;
                    req_param->ce_len_max          = llc_env_ptr->elt->duration_min/SLOT_SIZE;
                    req_param->interval_min        = param->interval_min;
                    req_param->interval_max        = param->interval_max;
                    req_param->con_latency         = param->latency;
                    req_param->superv_to           = param->timeout;
                    req_param->pref_period         = param->pref_period;
                    req_param->ref_con_event_count = param->ref_con_event_count;
                    req_param->offset0             = param->offset0;
                    req_param->offset1             = param->offset1;
                    req_param->offset2             = param->offset2;
                    req_param->offset3             = param->offset3;
                    req_param->offset4             = param->offset4;
                    req_param->offset5             = param->offset5;

                    kernel_msg_send(req_param);
                }
                else if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
                {
                    // Slave sends the connection parameter response PDU
                    struct llc_con_upd_req_ind data;

                    data.interval_min        = param->interval_min;
                    data.interval_max        = param->interval_max;
                    data.con_latency         = param->latency;
                    data.superv_to           = param->timeout;
                    data.pref_period         = param->pref_period;
                    data.ref_con_event_count = param->ref_con_event_count;
                    data.offset0             = param->offset0;
                    data.offset1             = param->offset1;
                    data.offset2             = param->offset2;
                    data.offset3             = param->offset3;
                    data.offset4             = param->offset4;
                    data.offset5             = param->offset5;

                    if (lld_con_param_rsp(conhdl, llc_env_ptr->elt, &data) == COMMON_ERROR_NO_ERROR)
                    {
                        // Wait for the response from the host
                        llc_env_ptr->rem_proc_state = LLC_REM_WAIT_CON_UPD_REQ;
                        llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);

                        llc_llcp_con_param_rsp_pdu_send(conhdl, &data);
                        // Start the LLCP Response TO
                        kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                    }
                    else
                    {
                        llc_llcp_reject_ind_pdu_send(conhdl, LLCP_CONNECTION_PARAM_REQ_OPCODE, COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
                    }
                }
            }
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP connection parameters response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_con_param_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_con_param_rsp *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // check if local procedure is on-going
    else if (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_CON_PARAM_RSP)
    {
        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSPECIFIED_ERROR);
    }
    else
    {
        // LLC is READY to receive commands, process it immediately

        // Reset the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        if (lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
        {
            struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

            struct llc_con_upd_req_ind *req_param = (struct llc_con_upd_req_ind *)llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);

            // Check if the peer parameters are compatible with the host connection update request, else reject
            if ( (param->interval_min <= req_param->con_intv_max) && (param->interval_max >= req_param->con_intv_min))
            {
                req_param->operation    = LLC_CON_UP_LOC_REQ;
                req_param->con_intv_min = param->interval_min;
                req_param->con_intv_max = param->interval_max;
                req_param->con_latency  = common_max(param->latency, req_param->con_latency) ;
                req_param->superv_to    = common_max(param->timeout, req_param->superv_to);
                req_param->ce_len_min   = llc_env_ptr->elt->duration_min/SLOT_SIZE;
                req_param->ce_len_max   = llc_env_ptr->elt->duration_min/SLOT_SIZE;

                kernel_msg_send(req_param);
            }
            else
            {
                // Get associated BLE Event environment
                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);

                llc_llcp_reject_ind_pdu_send(conhdl, LLCP_CONNECTION_PARAM_RSP_OPCODE, COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);

                // Check if we have to send the command complete event. It is done only if the update
                // was requested by the host or if any of the connection parameters has changed
                if (GETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ))
                {
                    SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ, false);
                    // Checks if the event is not filtered
                    if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT))
                    {
                        // Send the LE META event connection update to the host
                        llc_con_update_complete_send(COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE, conhdl, evt);
                    }
                }

                // Switch back the state in connected (PROCEDURE ABORTED)
                llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;

                // Clear Operation
                llc_util_clear_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);
            }
        }
    }

    return(msg_status);
}
#endif //#if !(BLE_QUALIF)

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP encryption request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_enc_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Check if we are allowed to receive this packet
    if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
            && (llc_env_ptr->rem_proc_state != LLC_REM_WAIT_ENC_REQ))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // This LLCP request shall be issued by the master only
    else if (lld_get_mode(conhdl) != LLD_EVT_SLV_MODE)
    {
        // Nothing to do, ignore the request
    }
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY)
            && (llc_env_ptr->rem_proc_state != LLC_REM_WAIT_ENC_REQ))
    {
        // Not expected, maybe a procedure collision
    }
    else
    {
        uint8_t tx_pkt_cnt;
        llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
        llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, true);
        llc_env_ptr->rem_proc_state = LLC_REM_WAIT_TP_FOR_ENC_REQ;

        llc_util_set_operation_ptr(conhdl, LLC_OP_ENCRYPT, LLC_GET_BASE_MSG(param));


        // Clear the response time out
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        GLOBAL_INT_DIS();
        // Encryption procedure is requested by the master, so we are not
        // supposed to transmit or receive any unexpected PDUs until procedure
        // is complete
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_FLOW_OFF, true);
        tx_pkt_cnt = lld_util_get_tx_pkt_cnt(llc_env_ptr->elt);
        GLOBAL_INT_RES();

        // Check if we still have data programmed for transmission
        if (tx_pkt_cnt)
        {
            SETF(llc_env_ptr->llc_status, LLC_STAT_WAIT_TRAFFIC_PAUSED, true);
        }
        else
        {
            // device ready to continue starting encryption
            kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
        }

        msg_status = KERNEL_MSG_NO_FREE;
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP encryption response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_enc_rsp *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Check if we are allowed to receive this packet
    if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
            && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_ENC_RSP))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }

    if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_ENC_RSP))
    {
        // Not expected, maybe a procedure collision
    }
    else
    {
        struct llm_enc_req *msg = KERNEL_MSG_ALLOC(LLM_ENC_REQ, TASK_LLM, dest_id, llm_enc_req);
        // Save the SKDs
        memcpy(&llc_env_ptr->encrypt.skd.skd[SESS_KEY_DIV_LEN],
                &param->skds.skdiv[0], SESS_KEY_DIV_LEN);

        // Ask to the AES to generate the SK by giving the SKD an LTK
        memcpy(&msg->key.ltk[0], &llc_env_ptr->encrypt.ltk.ltk[0], KEY_LEN);
        memcpy(&msg->plain_data[0], &llc_env_ptr->encrypt.skd.skd[0], KEY_LEN);

        // Send the command to start the SK generation
        kernel_msg_send(msg);

        // Save the IVs in the control structure
        llc_util_ivs_set(conhdl, param->ivs.iv);

        GLOBAL_INT_DIS();
        // Encryption procedure is now started in the slave, so we are not supposed to
        // receive any unexpected PDUs
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX_FLOW_OFF, true);
        GLOBAL_INT_RES();
        // Set the state of the LLC
        llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_SK_AND_START_ENC_REQ;

        // Restart the response timeout in case we don't get the LL_START_ENC_REQ
        kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP start encryption request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_start_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx,  struct llcp_start_enc_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Check if we are allowed to receive this packet
    if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
            && (   (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_SK_AND_START_ENC_REQ)
                    && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_START_ENC_REQ)))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }

    if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_SK_AND_START_ENC_REQ)
            && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_START_ENC_REQ))
    {
        // Not expected, maybe a procedure collision
    }
    else
    {
        // update state of the procedure
        if(llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_SK_AND_START_ENC_REQ)
        {
            llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_SK;
        }
        else
        {
            llc_env_ptr->loc_proc_state = LLC_LOC_SEND_START_ENC_RSP;
            // device ready to continue starting encryption
            kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP start encryption response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_start_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_start_enc_rsp *param)
{

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Check if we are allowed to receive this packet
    if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
            && (   (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_START_ENC_RSP)
                    && (llc_env_ptr->rem_proc_state != LLC_REM_WAIT_START_ENC_RSP)))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // disconnection on-going
    else if (llc_state_chk(state, LLC_DISC_BUSY))
    {
        // enable encryption to transmit termitate in an encrypted way
        if((lld_get_mode(conhdl) == LLD_EVT_SLV_MODE) && (llc_env_ptr->rem_proc_state == LLC_REM_WAIT_START_ENC_RSP))
        {
            ble_link_set(conhdl, ble_link_get(conhdl) | BLE_TXCRYPT_EN_BIT | BLE_RXCRYPT_EN_BIT);
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_ENABLE, true);
        }
    }
    else
    {
        // Clear the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        // Check the device role to perform the correct processing
        if(lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
        {
            llc_env_ptr->rem_proc_state = LLC_REM_WAIT_START_ENC_RSP_ACK;

            // restart LLCP TO timer if on-going transaction is pending
            if (llc_state_chk(state, LLC_LOC_PROC_BUSY))
            {
                // Restartt LLCP TO timer
                kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
            }

            // Send the LL_START_ENC_RSP message
            llc_llcp_start_enc_rsp_pdu_send(conhdl);
            GLOBAL_INT_DIS();
            // RX flow can now be restarted
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX_FLOW_OFF, false);
            GLOBAL_INT_RES();
        }
        else
        {
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
            llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;

            GLOBAL_INT_DIS();
            // TX and RX flow can now be restarted
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_FLOW_OFF | LLC_ENC_PAUSE_PENDING, false);
            GLOBAL_INT_RES();
            llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, false);
        }

        // Check which procedure was pending
        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_REFRESH_PENDING))
        {
            // The key refresh procedure is now complete, clear the refresh pending bit
            LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_REFRESH_PENDING, false);

            // Notify the host that the key has been refreshed
            llc_common_enc_key_ref_comp_evt_send(conhdl, COMMON_ERROR_NO_ERROR);
        }
        else
        {
            // Notify the host that the connection is now encrypted
            llc_common_enc_change_evt_send(conhdl, 1, COMMON_ERROR_NO_ERROR);
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP pause encryption request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_pause_enc_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_pause_enc_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // Check if we are allowed to receive this packet
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // This LLCP request shall be issued by the master only
    else if (lld_get_mode(conhdl) != LLD_EVT_SLV_MODE)
    {
        // Nothing to do, ignore the request
    }
    else if(!llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        uint8_t tx_pkt_cnt;
        llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);

        // Check if a local procedure is on-going, pause it
        // the LL_PAUSE_ENC_REQ
        if (llc_state_chk(state, LLC_LOC_PROC_BUSY))
        {
            // Clear the response time out
            kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
        }


        llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
        llc_state_update(dest_id, &state, LLC_TRAFFIC_PAUSED_BUSY, true);
        llc_env_ptr->rem_proc_state = LLC_REM_WAIT_TP_FOR_PAUSE_ENC_REQ;

        GLOBAL_INT_DIS();
        // Encryption procedure is requested by the master, so we are not supposed
        // to transmit or receive any unexpected PDUs until procedure is complete
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_FLOW_OFF, true);
        tx_pkt_cnt = lld_util_get_tx_pkt_cnt(llc_env_ptr->elt);
        GLOBAL_INT_RES();

        // Check if we still have data programmed for transmission
        if (tx_pkt_cnt)
        {
            SETF(llc_env_ptr->llc_status, LLC_STAT_WAIT_TRAFFIC_PAUSED, true);
        }
        else
        {
            // device ready to continue starting encryption
            kernel_msg_send_basic(LLC_ENC_MGT_IND,dest_id,dest_id);
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP pause encryption response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_pause_enc_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_pause_enc_rsp *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    bool proceed = false;

    // pause request initiated by local device
    if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
        && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_PAUSE_ENC_RSP))
    {
        // Reset the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        // Disable encryption for both TX and RX
        ble_link_set(conhdl, ble_link_get(conhdl) & ~(BLE_TXCRYPT_EN_BIT | BLE_RXCRYPT_EN_BIT));
        llc_env_ptr->loc_proc_state = LLC_LOC_WAIT_PAUSE_ENC_RSP_SENT;
        // If we are master, send back a LL_PAUSE_ENC_RSP PDU
        llc_llcp_pause_enc_rsp_pdu_send(conhdl);

        GLOBAL_INT_DIS();
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_ENABLE, false);
        // Encryption procedure is now started in the slave, so we are not supposed to
        // receive any unexpected PDUs
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_RX_FLOW_OFF, true);
        GLOBAL_INT_RES();
        proceed = true;
    }
    // pause request initiated by peer device
    else if(llc_state_chk(state, LLC_REM_PROC_BUSY)
            &(llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PAUSE_ENC_RSP))
    {
        // Reset the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        // Disable encryption for both TX and RX
        ble_link_set(conhdl, ble_link_get(conhdl) & ~(BLE_TXCRYPT_EN_BIT | BLE_RXCRYPT_EN_BIT));
        LLC_UTIL_ENC_STATE_UP(conhdl, LLC_ENC_ENABLE | LLC_ENC_PAUSE_PENDING, false);

        // The Pause Procedure is complete, we are now waiting for the master to
        // restart the encryption start procedure
        llc_env_ptr->rem_proc_state = LLC_REM_WAIT_ENC_REQ;

        // Set the LLCP response timer, in case the master would never restart the
        // encryption start procedure
        kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
        proceed = true;
    }

    // The packet has been received in an unexpected state
    if ((!proceed) && (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF)))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }

    return(msg_status);
}



/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP reject indication
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_reject_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_reject_ind *param)
{
    // use common reject handler
    llc_llcp_reject_ind(conhdl, dest_id, param->err_code, false);
    return (KERNEL_MSG_CONSUMED);
}

#if !(BLE_QUALIF)
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP Extended reject indication
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int llcp_reject_ind_ext_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_reject_ind_ext *param)
{
    // use common reject handler
    llc_llcp_reject_ind(conhdl, dest_id, param->err_code, true);
    return (KERNEL_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP ping request
 *
 * @param[in] msgid Id of the message received (probably   unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_ping_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, void *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;

    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else
    {
        llc_llcp_ping_rsp_pdu_send(conhdl);
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP ping response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_ping_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, void *param)
{
    // Message status
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This packet has been unexpectedly received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
            && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_PING_RSP))
    {
        // Go back to remote procedure idle state
        llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
        llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
        // Reset the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
    }

    return(KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP length request
 *
 * @param[in] msgid Id of the message received (probably   unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int llcp_length_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_length_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_REQ_RCVD, true);

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // check if waiting for TX ACK
    else
    {
        uint16_t temp_conn_eff_max_rx_octets;
        uint16_t temp_conn_eff_max_rx_time;
        uint16_t temp_conn_eff_max_tx_octets;
        uint16_t temp_conn_eff_max_tx_time;


        temp_conn_eff_max_rx_octets = common_min(llc_env_ptr->data_len_ext_info.conn_max_rx_octets,param->max_tx_octets);
        temp_conn_eff_max_rx_time = common_min(llc_env_ptr->data_len_ext_info.conn_max_rx_time,param->max_tx_time);
        temp_conn_eff_max_tx_octets = common_min(llc_env_ptr->data_len_ext_info.conn_max_tx_octets,param->max_rx_octets);
        temp_conn_eff_max_tx_time = common_min(llc_env_ptr->data_len_ext_info.conn_max_tx_time,param->max_rx_time);

        if ((llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets != temp_conn_eff_max_rx_octets)
                ||(llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time != temp_conn_eff_max_rx_time)
                ||(llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets != temp_conn_eff_max_tx_octets)
                ||(llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time != temp_conn_eff_max_tx_time))
        {
            GLOBAL_INT_DIS();
            llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets = temp_conn_eff_max_rx_octets;
            llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time   = temp_conn_eff_max_rx_time;
            llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets = temp_conn_eff_max_tx_octets;
            llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time   = temp_conn_eff_max_tx_time;
            //Set the new value in the driver
            lld_util_eff_tx_time_set(llc_env_ptr->elt, temp_conn_eff_max_tx_time, temp_conn_eff_max_tx_octets);
            GLOBAL_INT_RES();
            SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, false);
        }

        llc_llcp_length_rsp_pdu_send(conhdl);

        // enable the possibility to received command
        SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_REQ_RCVD, false);

        // Send the META event if not already sent and not masked
        if((llm_util_check_evt_mask(LE_DATA_LEN_CHG_EVT_BIT)) && !(GETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT)))
        {
            // Need to send event to the host
            // allocate the event message
            struct hci_le_data_len_chg_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_data_len_chg_evt);

            // fill event parameters
            event->subcode = HCI_LE_DATA_LEN_CHG_EVT_SUBCODE;
            event->conhdl= common_htobs(conhdl);
            event->max_rx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets;
            event->max_rx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time;
            event->max_tx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets;
            event->max_tx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time;

            // send the message
            hci_send_2_host(event);
            SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, true);
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP length response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_length_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_length_rsp *param)
{

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    // check if waiting for TX ACK
    else
    {
        // Clear the response time out
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_LENGTH_RSP))
        {
            uint16_t temp_conn_eff_max_rx_octets;
            uint16_t temp_conn_eff_max_rx_time;
            uint16_t temp_conn_eff_max_tx_octets;
            uint16_t temp_conn_eff_max_tx_time;

            // Go back to IDLE state
            llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
            llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);

            temp_conn_eff_max_rx_octets = common_min(llc_env_ptr->data_len_ext_info.conn_max_rx_octets,param->max_tx_octets);
            temp_conn_eff_max_rx_time = common_min(llc_env_ptr->data_len_ext_info.conn_max_rx_time,param->max_tx_time);
            temp_conn_eff_max_tx_octets = common_min(llc_env_ptr->data_len_ext_info.conn_max_tx_octets,param->max_rx_octets);
            temp_conn_eff_max_tx_time = common_min(llc_env_ptr->data_len_ext_info.conn_max_tx_time,param->max_rx_time);

            lld_util_compute_ce_max(llc_env_ptr->elt,temp_conn_eff_max_tx_time,temp_conn_eff_max_rx_time);
            if ((llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets != temp_conn_eff_max_rx_octets)
                    ||(llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time != temp_conn_eff_max_rx_time)
                    ||(llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets != temp_conn_eff_max_tx_octets)
                    ||(llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time != temp_conn_eff_max_tx_time))
            {
                llc_env_ptr->data_len_ext_info.conn_max_tx_octets = temp_conn_eff_max_tx_octets;
                llc_env_ptr->data_len_ext_info.conn_max_tx_time = temp_conn_eff_max_tx_time;
                GLOBAL_INT_DIS();
                llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets = temp_conn_eff_max_rx_octets;
                llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time   = temp_conn_eff_max_rx_time;
                llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets = temp_conn_eff_max_tx_octets;
                llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time   = temp_conn_eff_max_tx_time;
                //Set the new value in the driver
                lld_util_eff_tx_time_set(llc_env_ptr->elt, temp_conn_eff_max_tx_time, temp_conn_eff_max_tx_octets);
                GLOBAL_INT_RES();
                //Send the event to the host
                SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, false);
            }
            // Send the META event if not masked
            if((llm_util_check_evt_mask(LE_DATA_LEN_CHG_EVT_BIT)) && !(GETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT)))
            {
                // Need to send event to the host
                // allocate the event message
                struct hci_le_data_len_chg_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_data_len_chg_evt);

                // fill event parameters
                event->subcode = HCI_LE_DATA_LEN_CHG_EVT_SUBCODE;
                event->conhdl= common_htobs(conhdl);
                event->max_rx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets;
                event->max_rx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time;
                event->max_tx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets;
                event->max_tx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time;

                // send the message
                hci_send_2_host(event);
                SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, true);
            }
        }
        else
        {
            llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED);
        }
    }

    return(msg_status);
}


#if (BLE_2MBPS)

/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY request
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_phy_req_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_req *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    /**
     * Check if remote procedure is on-going
     */
    else if (llc_state_chk(state, LLC_REM_PROC_BUSY))
    {
        llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
    }
    else
    {
        bool reject = false;
        /**
         * Check collision
         */
        if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
        {
            switch(llc_env_ptr->loc_proc_state)
            {
            case LLC_LOC_WAIT_PHY_RSP:
                if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
                {
                    break;
                }
                //if Master no Break
            case LLC_LOC_WAIT_PHY_UPD_INSTANT:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
                reject = true;
            } break;

            case LLC_LOC_WAIT_CON_PARAM_RSP:
            case LLC_LOC_WAIT_CON_UPD_REQ:
            {
                //If slave side do not reject peer procedure
                if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
                {
                    break;
                }
            }
            //no Break
            case LLC_LOC_WAIT_CON_UPD_INSTANT:
            case LLC_LOC_WAIT_MAP_UPD_INSTANT:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_DIFF_TRANSACTION_COLLISION);
                reject = true;
            } break;

            default: /* Nothing to do */ break;
            }
        }

        /**
         * Check the values received
         * At Least one bit in each field shall be set to 1
         */
        if ((!(param->rx_phys) || !(param->tx_phys)) && (!reject))
        {
            //Values out of range
            llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
            reject = true;
        }

        if (!reject)
        {
            //If the event is masked do not start procedure
            if(llm_util_check_evt_mask(LE_PHY_UPD_CMP_EVT_BIT))
            {
                llc_state_update(dest_id, &state, LLC_REM_PROC_BUSY, true);
                /**
                 * Slave side
                 */
                if (lld_get_mode(conhdl) == LLD_EVT_SLV_MODE)
                {
                    struct llcp_phy_rsp phy_rsp;

                    llm_util_get_default_phy(&phy_rsp.tx_phys, &phy_rsp.rx_phys);

                    llc_llcp_phy_rsp_pdu_send(conhdl, &phy_rsp);

                    // Set the state of the LLC
                    llc_env[conhdl]->rem_proc_state = LLC_REM_WAIT_PHY_UPD_REQ;

                    // Start the LLCP Response TO
                    kernel_timer_set(LLC_LLCP_RSP_TO, dest_id, LLC_DFT_RSP_TO);
                }
                /**
                 * Master side
                 */
                else
                {
                    struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, dest_id, dest_id, llc_phy_upd_req_ind);

                    phy_update->operation   = LLC_PHY_UPD_PEER_REQ;
                    phy_update->rx_phys     = param->rx_phys;
                    phy_update->tx_phys     = param->tx_phys;

                    kernel_msg_send(phy_update);
                }
            }
            else //If the event is masked reject the procedure
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE);
            }
        }
    }

    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_phy_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_rsp *param)
{

    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    #if (BLE_PERIPHERAL)

    // The packet has been received in an unexpected state
    if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else
    {
        struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, dest_id, dest_id, llc_phy_upd_req_ind);
        // Sanity checks if local procedure is on-going and we are master
        ASSERT_ERR(lld_get_mode(conhdl) == LLD_EVT_MST_MODE);

        // Reset the LLCP response timeout
        kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

        // Check if at Least one bit in each field shall be set to 1
         if ((param->rx_phys) || (param->tx_phys))
        {
            phy_update->operation   = LLC_PHY_UPD_PEER_RSP;
        }
        else
        {
            llc_llcp_reject_ind_pdu_send(conhdl, LLCP_CONNECTION_PARAM_RSP_OPCODE, COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
            phy_update->operation   = LLC_PHY_UPD_TERMINATE;
            phy_update->status      = COMMON_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;

        }
         phy_update->rx_phys     = param->rx_phys;
         phy_update->tx_phys     = param->tx_phys;

         kernel_msg_send(phy_update);
    }
    #endif // (BLE_PERIPHERAL)
    return(msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY update indication
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_phy_upd_ind_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_phy_upd_req *param)
{
    // Message status
    int msg_status  = int_ctx ? KERNEL_MSG_SAVED : KERNEL_MSG_CONSUMED;

    #if (BLE_PERIPHERAL)
    // retrieve origin message
    struct llc_llcp_recv_ind* msg = LLC_GET_BASE_MSG(param);

    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint8_t state = kernel_state_get(dest_id);

    if(int_ctx)
    {
        // mark instant ignored
        msg->dummy = LLC_INSTANT_IGNORED;
    }

    if(msg->dummy == LLC_INSTANT_IGNORED)
    {
        // Check if we are allowed to receive this packet
        if (LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
        {
            // This packet has been received during an Encryption Start
            // or an Encryption Pause Procedure, so we need to exit the connection
            // state immediately with reason "MIC Failure".
            msg->dummy = LLC_INSTANT_MIC_FAILURE;
        }
        // Instant operation is already on-going
        else if (lld_util_instant_ongoing(llc_env_ptr->elt))
        {
            msg->dummy = LLC_INSTANT_COLLISION;
        }
        else if ((llc_state_chk(state, LLC_REM_PROC_BUSY)
                && (llc_env_ptr->rem_proc_state != LLC_REM_WAIT_PHY_UPD_REQ))
                // This LLCP request shall be issued by the master only
                || (lld_get_mode(conhdl) != LLD_EVT_SLV_MODE))
        {
            msg->dummy = LLC_INSTANT_REJECT;
        }
        else
        {
            /**
             * If both the M_TO_S_PHY and S_TO_M_PHY fields are zero then there is no Instant and shall be ignored on receipt.
             * And a least one phy has changed
             */
            uint8_t tx_phy, rx_phy;

            // Stop the LLCP Response TO
            kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
            //Gets the current values used by the link
            lld_util_get_phys(llc_env_ptr->elt, &tx_phy, &rx_phy);
            if(  ((param->m_to_s_phy != PHYS_NO_PREF) || (param->s_to_m_phy != PHYS_NO_PREF))
               && ((param->m_to_s_phy != rx_phy) || (param->s_to_m_phy != tx_phy)))
            {

                uint16_t instant = common_btohs(param->instant);
                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);

                // Check if instant has passed
                if(((uint16_t)((instant - evt->evt.conn.counter) % 65536) < 32767))
                {
                    // Instant has not passed, so program the LLD to schedule the phy update
                    msg->dummy = LLC_INSTANT_ACCEPTED;
                    //Gives to the driver the instant, phys values and the type of the action
                    //Like this function is called by the SLAVE the TX direction is S -> M
                    lld_util_phy_update_req(llc_env_ptr->elt, param->instant, param->s_to_m_phy, param->m_to_s_phy);
                }
                else
                {
                    msg->dummy = LLC_INSTANT_PASSED;
                }
            }
            //Case where the peer doesn't want to change any phy
            else
            {
                struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, dest_id, dest_id, llc_phy_upd_req_ind);
                if ((param->instant == 0) ||((param->m_to_s_phy == rx_phy) && (param->s_to_m_phy == tx_phy)))
                {
                    // Do not wait the instant
                    llc_env_ptr->rem_proc_state = LLC_REM_PHY_NO_INSTANT;
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 1);
                }

                // Accept the instant without check
                msg->dummy = LLC_INSTANT_ACCEPTED;
                                
                phy_update->operation   = LLC_PHY_UPD_TERMINATE;
                phy_update->rx_phys     = rx_phy;
                phy_update->tx_phys     = tx_phy;
                phy_update->status      = COMMON_ERROR_NO_ERROR;
                kernel_msg_send(phy_update);

            }
        }
    }

    if(!int_ctx)
    {
        // execute state update according to retrieved info
        switch(msg->dummy)
        {
            case LLC_INSTANT_IGNORED: /* Nothing to do */ break;
            case LLC_INSTANT_PASSED:
            {
                // Connection is considered lost, the Link Layer does not send any further packets
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_INSTANT_PASSED);
            } break;
            case LLC_INSTANT_MIC_FAILURE:
            {
                llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
            } break;
            case LLC_INSTANT_COLLISION:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_LMP_COLLISION);
            } break;
            case LLC_INSTANT_REJECT:
            {
                llc_llcp_reject_ind_pdu_send(conhdl, param->opcode, COMMON_ERROR_UNSUPPORTED);
            }break;
            case LLC_INSTANT_ACCEPTED:
            {
                // check if connection param is initiated by local device
                if(llc_state_chk(state, LLC_LOC_PROC_BUSY)
                        && (llc_env_ptr->loc_proc_state == LLC_LOC_WAIT_PHY_UPD_REQ))
                {

                    // Stop the LLCP Response TO
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                    // Set the state of the LLC
                    llc_env[conhdl]->loc_proc_state = LLC_LOC_WAIT_PHY_UPD_INSTANT;
                    //Gives to the driver the instant, phys values and the type of the action
                    //Like this function is called by the SLAVE the TX direction is S -> M
                    lld_util_phy_update_req(llc_env_ptr->elt, param->instant, param->s_to_m_phy, param->m_to_s_phy);
                }

                if(llc_env_ptr->rem_proc_state == LLC_REM_WAIT_PHY_UPD_REQ)
                {
                    // Stop the LLCP Response TO
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                    llc_env_ptr->rem_proc_state = LLC_REM_WAIT_PHY_UPD_INSTANT;
                }

                // instant proceed and finished, nothing to do
                if(GETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED))
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_INSTANT_PROCEED, 0);
                }
                else
                {
                    SETF(llc_env_ptr->llc_status,  LLC_STAT_LLCP_INSTANT_EXTRACTED, 1);
                }
            } break;

            default:
            {
                ASSERT_INFO(0, msg->dummy, dest_id);
            }break;
        }
    }
#endif //(BLE_PERIPHERAL)

    return(msg_status);
}

#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP unknown response
 *
 * @param[in] conhdl  Connection handle of the link which received the message
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] int_ctx Indicate if the LLCP is proceed under interrupt context
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llcp_unknown_rsp_handler(uint16_t conhdl, kernel_task_id_t dest_id, bool int_ctx, struct llcp_unknown_rsp *param)
{
    // Message status
    int msg_status  = KERNEL_MSG_CONSUMED;
    uint8_t state = kernel_state_get(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    // The packet has been received in an unexpected state
    if ((LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_RX_FLOW_OFF))
            && (llc_env_ptr->loc_proc_state != LLC_LOC_WAIT_ENC_RSP))
    {
        // This unexpected packet has been received during an Encryption Start
        // or an Encryption Pause Procedure, so we need to exit the connection
        // state immediately with reason "MIC Failure".
        llc_util_dicon_procedure(conhdl, COMMON_ERROR_TERMINATED_MIC_FAILURE);
    }
    else if(llc_state_chk(state, LLC_LOC_PROC_BUSY))
    {
        // Check state
        switch (llc_env_ptr->loc_proc_state)
        {
            // No break
            case LLC_LOC_WAIT_ENC_RSP:
            {
                // We can accept this PDU only if we are performing the initial Encryption
                // Start Procedure
                if (!(LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_REFRESH_PENDING)))
                {
                    // Reception of the LL_UNKNOWN_RSP terminates the Encryption Start Procedure
                    LLC_UTIL_ENC_STATE_SET(conhdl, LLC_ENC_DISABLED);

                    // Notify the host of the Encryption Start Failure
                    llc_common_enc_change_evt_send(conhdl, 0, COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE);

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                }
            } break;
            #if !(BLE_QUALIF)
            case LLC_LOC_WAIT_FEAT_RSP:
            {
                if (param->unk_type == LLCP_SLAVE_FEATURE_REQ_OPCODE)
                {
                    // Clear the response time out
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);

                    // Currently a feature request procedure can only be initiated by the host
                    // Therefore, the host is notified that the read remote features procedure completed
                    llc_feats_rd_event_send(COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE, conhdl, &llc_env_ptr->feats_used);

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                }
            } break;
            case LLC_LOC_WAIT_CON_PARAM_RSP:
            {
                if (param->unk_type == LLCP_CONNECTION_PARAM_REQ_OPCODE)
                {
                    // Reset the LLCP response timeout
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                    #if (BLE_CENTRAL)
                    if (lld_get_mode(conhdl) == LLD_EVT_MST_MODE)
                    {
                        struct llc_con_upd_req_ind *param_req = (struct llc_con_upd_req_ind *) llc_util_get_operation_ptr(conhdl, LLC_OP_LOC_PARAM_UPD);
                        // reschedule the message
                        param_req->operation = LLC_CON_UP_FORCE;
                        kernel_msg_send(param_req);
                    }
                    else
                    #endif // (BLE_CENTRAL)
                    {
                        // Check if we have to send the command complete event. It is done only if the update
                        // was requested by the host or if any of the connection parameters has changed
                        if (GETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ))
                        {
                            SETF(llc_env_ptr->llc_status, LLC_STAT_UPDATE_HOST_REQ, false);
                            // Checks if the event is not filtered
                            if (llm_util_check_evt_mask(LE_CON_UPD_EVT_BIT))
                            {
                                // Get associated BLE Event environment
                                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env_ptr->elt);
                                // Send the LE META event connection update to the host
                                llc_con_update_complete_send(COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE, conhdl, evt);
                            }
                        }

                        // Clear Operation
                        llc_util_clear_operation_ptr( conhdl, LLC_OP_LOC_PARAM_UPD);
                        // Set the state of the LLC
                        llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                        llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                    }
                }
            } break;
            case LLC_LOC_WAIT_LENGTH_RSP:
            {
                if (param->unk_type == LLCP_LENGTH_REQ_OPCODE)
                {
                    // Clear the response time out
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                    // Avoid to send one more time the LLCP_LENGTH_REQ
                    llc_env_ptr->data_len_ext_info.send_req_not_allowed = true;
                    // Send the META event if not masked
                    if((llm_util_check_evt_mask(LE_DATA_LEN_CHG_EVT_BIT)) && !(GETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT)))
                    {
                        // Need to send event to the host
                        // allocate the event message
                        struct hci_le_data_len_chg_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_data_len_chg_evt);

                        // fill event parameters
                        event->subcode = HCI_LE_DATA_LEN_CHG_EVT_SUBCODE;
                        event->conhdl= common_htobs(conhdl);
                        event->max_rx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_octets;
                        event->max_rx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_rx_time;
                        event->max_tx_octets = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_octets;
                        event->max_tx_time = llc_env_ptr->data_len_ext_info.conn_eff_max_tx_time;

                        // send the message
                        hci_send_2_host(event);
                        SETF(llc_env_ptr->data_len_ext_info.data_len_ext_flag, LLC_DLE_EVT_SENT, true);
                    }

                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                }
            } break;
            case LLC_LOC_WAIT_PING_RSP:
            {
                if (param->unk_type == LLCP_PING_REQ_OPCODE)
                {
                    // Set the state of the LLC
                    llc_state_update(dest_id, &state, LLC_LOC_PROC_BUSY, false);
                    llc_env_ptr->loc_proc_state = LLC_LOC_IDLE;
                    // Reset the LLCP response timeout
                    kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                }
            } break;

            #if (BLE_2MBPS)
            case LLC_LOC_WAIT_PHY_RSP:
            case LLC_LOC_WAIT_PHY_UPD_REQ:
            {
                struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env[conhdl]->elt);
                // Reset the LLCP response timeout
                kernel_timer_clear(LLC_LLCP_RSP_TO, dest_id);
                {
                    struct llc_phy_upd_req_ind * phy_update = KERNEL_MSG_ALLOC(LLC_PHY_UPD_REQ_IND, KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), KERNEL_BUILD_ID(TASK_LLC, evt->conhdl), llc_phy_upd_req_ind);
                    phy_update->operation   = LLC_PHY_UPD_TERMINATE;
                    phy_update->rx_phys     =  evt->evt.conn.rx_phy;
                    phy_update->tx_phys     = evt->evt.conn.tx_phy;
                    phy_update->status      = COMMON_ERROR_UNSUPPORTED_REMOTE_FEATURE;
                    kernel_msg_send(phy_update);
                }
                SETF(llc_env_ptr->llc_status, LLC_STAT_PHY_ENABLED, false);

            } break;
            #endif // (BLE_2MBPS)
            #endif //#if !(BLE_QUALIF)

            default: /* Nothing to do */ break;
        }
    }

    return msg_status;
}

uint8_t llc_llcp_get_autorize(uint8_t opcode)
{
    uint8_t res = LLC_LLCP_NO_AUTHZED;
    if(opcode < LLCP_OPCODE_MAX_OPCODE)
    {
        res = llcp_pdu_handler[opcode].enc_auth;
    }

    return res;
}

#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCLLCP
