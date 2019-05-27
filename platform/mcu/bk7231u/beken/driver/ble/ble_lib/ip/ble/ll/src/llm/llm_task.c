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

#include "rwip_config.h"

#include "em_buf.h"
#include "llm.h"
#include "llm_task.h"

#include "common_bt.h"
#include "common_endian.h"
#include "common_error.h"
#include "common_list.h"
#include "common_math.h"
#include "common_utils.h"
#include "common_version.h"
#include "kernel_event.h"
#include "kernel_msg.h"
#include "kernel_timer.h"
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
 * @brief Handles the encrypt data request from internal task.
 * The handler processes the request by encrypt the plain text data in the request
 * using the key given in the command and returns the encrypted data to the requester.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_enc_req_handler(kernel_msg_id_t const msgid,
                               struct llm_enc_req const *param,
                               kernel_task_id_t const dest_id,
                               kernel_task_id_t const src_id)
{
    //extract the kernel_msg pointer from the param passed
    struct kernel_msg * msg = kernel_param2msg(param);

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

#if SECURE_CONNECTIONS
#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles the end of ECC multiplication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_ecc_result_ind_handler(kernel_msg_id_t const msgid, struct ecc_result_ind const *param,
                                      kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    if (llm_le_env.cur_ecc_multiplication == LLM_DHKEY_GENERATION)
    {
        if(llm_util_check_evt_mask(LE_GEN_DHKEY_CMP_EVT_BIT))
        {
            struct hci_le_generate_dhkey_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE,hci_le_generate_dhkey_cmp_evt);

            // fill event parameters
            event->subcode = HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE;
            event->status = COMMON_ERROR_NO_ERROR;

            memcpy(&(event->dh_key[0]), &param->key_res_x[0], 32);

            // send the message
            hci_send_2_host(event);
        }
    }
    else if (llm_le_env.cur_ecc_multiplication == LLM_PUBLIC_KEY_GENERATION)
    {
        if(llm_util_check_evt_mask(LE_RD_LOC_P256_PUB_KEY_CMP_EVT_BIT))
        {
            struct hci_le_generate_p256_public_key_cmp_evt *event = KERNEL_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE,hci_le_generate_p256_public_key_cmp_evt);

            // fill event parameters
            event->subcode = HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE;
            event->status = COMMON_ERROR_NO_ERROR;

            memcpy(&(event->public_key.x[0]), &param->key_res_x[0], 32);
            memcpy(&(event->public_key.y[0]), &param->key_res_y[0], 32);

            // Send HCI Public Key P256 Complete Event.
            hci_send_2_host(event);
        }
        #if (NVDS_SUPPORT)
        // Write the secret key to NVDS - the Host GAP will write the Public Key
        if (NVDS_OK != nvds_put(NVDS_TAG_LE_PRIVATE_KEY_P256, NVDS_LEN_LE_PRIVATE_KEY_P256, &(llm_le_env.secret_key256[0])))
        {
            ASSERT_ERR(0);// Could not write new secret key to NVDS
        }
        #endif //(NVDS_SUPPORT)
    }
    else
    {
        ASSERT_ERR(0);
    }

    llm_le_env.cur_ecc_multiplication = LLM_ECC_IDLE;

    return (KERNEL_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
#endif //SECURE_CONNECTIONS


/**
 ****************************************************************************************
 * @brief Handles end of timer that manages renewal of Resolving private addresses.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_le_enh_priv_addr_renew_timer_handler(kernel_msg_id_t const msgid, void const *param,
                                                    kernel_task_id_t const dest_id, kernel_task_id_t const src_id)
{
    // mark the renewal timer disabled
    SETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN, false);

    // force renew of rpa addresses.
    if(llm_le_env.elt != NULL)
    {
        lld_ral_renew_req(llm_le_env.elt);
    }
    else
    {
        lld_util_ral_force_rpa_renew();
    }


    // LLM is not in idle mode, restart the renewal timer
    if(kernel_state_get(TASK_LLM) != LLM_IDLE)
    {
        // restart the timer that manages renewal of resolvable private addresses.
        kernel_timer_set(LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER, TASK_LLM, KERNEL_TIME_IN_SEC(llm_le_env.enh_priv_rpa_timeout));
        SETF(llm_le_env.enh_priv_info, LLM_RPA_RENEW_TIMER_EN, true);
    }

    return (KERNEL_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of the event stopping indication from the LLD. Upon the
 * reception of this message, the LLM host back to IDLE state.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int lld_stop_ind_handler(kernel_msg_id_t const msgid,
                                void const *param,
                                kernel_task_id_t const dest_id,
                                kernel_task_id_t const src_id)
{
    if (kernel_state_get(TASK_LLM) == LLM_STOPPING)
    {
        if( (llm_le_env.state == LLM_ADVERTISING) || (llm_le_env.state == LLM_SCANNING) || (llm_le_env.state == LLM_INITIATING))
        {

            // Send the command complete event
            llm_common_cmd_complete_send(llm_le_env.opcode, COMMON_ERROR_NO_ERROR);
            #if(BLE_CENTRAL)
            if(llm_le_env.state == LLM_INITIATING)
            {
                llc_le_con_cmp_evt_send(COMMON_ERROR_UNKNOWN_CONNECTION_ID, 0xFFFF, NULL);
            }
            #endif // (BLE_CENTRAL)
            llm_le_env.state = LLM_IDLE;
        }
        // Set the state to IDLE
        kernel_state_set(TASK_LLM, LLM_IDLE);

        // Reset element pointer
        llm_le_env.elt = NULL;

        // Reset the connection handle
        llm_le_env.conhdl_alloc = 0xFFFF;
    }

    return (KERNEL_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Default message handler for patch purpose
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_dft_handler(kernel_msg_id_t const msgid,
                           void *param,
                           kernel_task_id_t const dest_id,
                           kernel_task_id_t const src_id)
{
    // Message is not expected, drop the message
    return (KERNEL_MSG_CONSUMED);
}


#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles the delay between 2 consecutive HCI cmd.
 * The handler allow the reception of a new host channel classification command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int llm_le_set_host_ch_class_cmd_sto_handler(kernel_msg_id_t const msgid,
                                                    void const *param,
                                                    kernel_task_id_t const dest_id,
                                                    kernel_task_id_t const src_id)
{
    #if (BLE_CENTRAL && BLE_CHNL_ASSESS)
    llm_le_env.ch_map_assess.llm_le_set_host_ch_class_cmd_sto = true;
    #endif
    return (KERNEL_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL)



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
extern int hci_command_handler(kernel_msg_id_t const msgid, void const *param, kernel_task_id_t const dest_id, kernel_task_id_t const src_id);

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
static const struct kernel_msg_handler llm_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KERNEL_MSG_DEFAULT_HANDLER,                (kernel_msg_func_t)llm_dft_handler},
    {LLD_STOP_IND,                          (kernel_msg_func_t)lld_stop_ind_handler},
    {LLM_STOP_IND,                          (kernel_msg_func_t)lld_stop_ind_handler},
    #if (BLE_CENTRAL)
    {LLM_LE_SET_HOST_CH_CLASS_CMD_STO,      (kernel_msg_func_t)llm_le_set_host_ch_class_cmd_sto_handler},
    #endif // (BLE_CENTRAL)

    {LLM_ENC_REQ,                           (kernel_msg_func_t)llm_enc_req_handler},

    #if SECURE_CONNECTIONS
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    {LLM_ECC_RESULT_IND,                    (kernel_msg_func_t)llm_ecc_result_ind_handler},
    #endif //(BLE_CENTRAL || BLE_PERIPHERAL)
    #endif //SECURE_CONNECTIONS

    {LLM_LE_ENH_PRIV_ADDR_RENEW_TIMER,      (kernel_msg_func_t)llm_le_enh_priv_addr_renew_timer_handler},
    {HCI_COMMAND,                           (kernel_msg_func_t)hci_command_handler}
};

/// Specifies the message handlers that are common to all states.
const struct kernel_state_handler llm_default_handler = KERNEL_STATE_HANDLER(llm_default_state);

/// Defines the placeholder for the states of all the task instances.
kernel_state_t llm_state[LLM_IDX_MAX];

/// @} LLMTASK
