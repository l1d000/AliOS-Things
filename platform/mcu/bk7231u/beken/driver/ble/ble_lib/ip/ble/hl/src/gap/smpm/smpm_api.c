/**
 ****************************************************************************************
 *
 * @file smpm_api.c
 *
 * @brief SMPM API implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SMPM_API
 * @ingroup SMPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "rwip_config.h"
#if (BLE_SMPM)
#include "smpm_api.h"
#include "smp_common.h"
#include <string.h>

#include "hci.h"



/*
 * LOCAL FUNCTION DEFINITONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Send a generate Random Number request to the HCI.
 ****************************************************************************************
 */
static void smpm_send_gen_rand_nb_req(void)
{
    void *no_param = kernel_msg_alloc(HCI_COMMAND, 0, HCI_LE_RAND_CMD_OPCODE, 0);
    hci_send_2_controller(no_param);
}

/**
 ****************************************************************************************
 * @brief Check the address type provided by the application.
 *
 * @param[in]  addr_type            Provided address type to check
 * @param[out] true if the address type is valid, false else
 ****************************************************************************************
 */
static bool smpm_check_addr_type(uint8_t addr_type)
{
    bool status;

    switch (addr_type)
    {
        case (GAP_STATIC_ADDR):
        case (GAP_NON_RSLV_ADDR):
        case (GAP_RSLV_ADDR):
        {
            status = true;
        } break;

        default:
        {
            status = false;
        }
        break;
    }

    return (status);
}

/*
 * FUNCTION DEFINITONS
 ****************************************************************************************
 */


uint8_t smpm_gen_rand_addr(uint8_t addr_type)
{
    // Command Status
    uint8_t status = GAP_ERR_NO_ERROR;

    // Check the operation code and the address type
    if (smpm_check_addr_type(addr_type))
    {
        // Ask for generation of a random number
        smpm_send_gen_rand_nb_req();
    }
    else
    {
        status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK,
                SMP_ERROR_INVALID_PARAM);
    }

    return (status);
}

void smpm_resolv_addr(bd_addr_t* addr, struct gap_sec_key *irk)
{
    // Prand' =  0[0:12] || prand (padding to have a 128-bit value)
    uint8_t prand_bis[GAP_KEY_LEN];

    // Clear prand_bis
    memset(&prand_bis[0], 0x00, GAP_KEY_LEN);

    /*
     * Address provided from higher layers in LSB->MSB
     *      -----------------------------------------------------------------------------------------
     *      | hash[0:(GAP_ADDR_HASH_LEN-1)] | prand[(GAP_ADDR_HASH_LEN:(BD_ADDR_LEN-1)] |
     *      -----------------------------------------------------------------------------------------
     *
     * prand_bis value sent to LL shall be LSB->MSB
     *      --------------------------------------------------------------------------------------
     *      | prand[0:(GAP_ADDR_PRAND_LEN-1)] | 0[(GAP_ADDR_PRAND_LEN):(GAP_KEY_LEN-1)]  |
     *      --------------------------------------------------------------------------------------
     */

    // Copy prand value in prand_bis
    memcpy(&(prand_bis[0]), &(addr->addr[GAP_ADDR_PRAND_LEN]), GAP_ADDR_PRAND_LEN);

    // Ask for generation of a random number
    smpm_send_encrypt_req((uint8_t*)irk, &prand_bis[0]);
}


/**
 ****************************************************************************************
 * @brief Use the encryption block
 *
 * @param[in] operand_1 First operand as encryption block input (16 bytes)
 * @param[in] operand_2 Second operand as encryption block input (16 bytes)
 *
 ****************************************************************************************
 */
void smpm_use_enc_block(uint8_t *operand_1, uint8_t *operand_2)
{
    // Ask for generation of a random number
    smpm_send_encrypt_req(operand_1, operand_2);
}



/**
 ****************************************************************************************
 * @brief Generate a random number
 ****************************************************************************************
 */
void smpm_gen_rand_nb(void)
{
    // Ask for generation of a random number
    smpm_send_gen_rand_nb_req();
}


void smpm_send_encrypt_req(uint8_t *operand_1, uint8_t *operand_2)
{
    struct hci_le_enc_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_ENC_CMD_OPCODE, hci_le_enc_cmd);

    memcpy(&cmd->key.ltk[0], operand_1, GAP_KEY_LEN);
    memcpy(&cmd->plain_data[0], operand_2, GAP_KEY_LEN);

    hci_send_2_controller(cmd);
}

#if (SECURE_CONNECTIONS)
void smpm_send_generate_dh_key(uint8_t *operand_1, uint8_t *operand_2)
{
    struct hci_le_generate_dh_key_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_GEN_DHKEY_CMD_OPCODE, hci_le_generate_dh_key_cmd);

    memcpy((void*)&cmd->public_key[0],(void*)operand_1, 32);
    memcpy((void*)&cmd->public_key[32],(void*)operand_2, 32);

    hci_send_2_controller(cmd);
}
#endif // (SECURE_CONNECTIONS)
/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

#endif // (BLE_SMPM)
/// @} SMPM_API
