/**
 ****************************************************************************************
 *
 * @file em_buf.c
 *
 * @brief BLET EM buffers
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup EM_BUF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // stack configuration

#if (BLE_EMB_PRESENT)

#include <stddef.h>          // standard definition
#include <stdint.h>          // standard integer definition
#include <stdbool.h>         // standard boolean definition
#include <string.h>          // string manipulation
#include "em_buf.h"          // ble buffer definition

#include "kernel_mem.h"          // Kernel memory management

#include "em_map.h"

#include "reg_ble_em_tx_desc.h"         // EM TX_DESC BLE register
#include "reg_ble_em_rx_desc.h"         // EM RX_DESC BLE register
#include "reg_ble_em_tx_buffer_data.h"  // EM TX_BUFFER BLE register
#include "reg_ble_em_tx_buffer_cntl.h"  // EM TX_BUFFER BLE register
#include "reg_ble_em_rx_buffer.h"       // EM RX_BUFFER BLE register
#include "reg_common_em_et.h"
/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// BLE EM buffer management environment
struct em_buf_env_tag em_buf_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void em_buf_init(void)
{
    int i;

    // Current RX buffer initialization
    em_buf_env.rx_current = 0;

    // Initialize the list of TX data buffers and descriptor
    common_list_init(&em_buf_env.tx_desc_free);
    common_list_init(&em_buf_env.tx_buff_free);

    // Save the address of the first TX descriptor
    em_buf_env.tx_desc = (struct em_buf_tx_desc *)(EM_BASE_ADDR +
                                                   REG_BLE_EM_TX_DESC_ADDR_GET(0));

    // Fill in the TX control and non connected descriptors lists
    for (i=0; i<(BLE_TX_DESC_CNTL+BLE_TX_DESC_ADV); i++)
    {
        // Store the index of the TX buffers
        em_buf_env.tx_desc_node[i].idx = i;
        // Initialize TX descriptor data pointer
        ble_txdataptr_set(i, REG_BLE_EM_TX_BUFFER_CNTL_ADDR_GET(i));
        ble_txdle_set(i, 0);
        //Set by default all the descriptor are acknowledged
        ble_txcntl_set(i, BLE_TXDONE_BIT);
    }

    // Fill in the TX descriptors data lists
    for (i=(BLE_TX_DESC_CNTL+BLE_TX_DESC_ADV); i<BLE_TX_DESC_CNT; i++)
    {
        // Store the index of the TX buffers
        em_buf_env.tx_desc_node[i].idx = i;
        ble_txdataptr_set((int)i, (uint16_t)0);
        ble_txdle_set(i, 0);
        //Set by default all the descriptor are acknowledged
        ble_txcntl_set(i, BLE_TXDONE_BIT);
        common_list_push_back(&em_buf_env.tx_desc_free, &em_buf_env.tx_desc_node[i].hdr);
    }

    #if (BLE_TX_BUFFER_CNT > 0)
    // Fill in the TX buffer data lists
    for (i=0; i<BLE_TX_BUFFER_CNT; i++)
    {
        // Store the index of the TX buffers
        em_buf_env.tx_buff_node[i].idx = i;
        em_buf_env.tx_buff_node[i].buf_ptr = REG_BLE_EM_TX_BUFFER_DATA_ADDR_GET(i);
        common_list_push_back(&em_buf_env.tx_buff_free, &em_buf_env.tx_buff_node[i].hdr);
    }
    #endif // (BLE_TX_BUFFER_CNT > 0)

    // Fill in the RX buffer lists
    for (i=0; i<BLE_RX_BUFFER_CNT; i++)
    {
        int j = em_buf_rx_next(i);

        // Initialize RX descriptor data pointer
        ble_rxdataptr_set(i, REG_BLE_EM_RX_BUFFER_ADDR_GET(i));

        // Link the RX descriptors together in exchange memory
        ble_rxcntl_set(i, REG_BLE_EM_RX_DESC_ADDR_GET(j));
    }
}

void em_buf_rx_free(uint8_t hdl)
{
    // Clear RX done bit from the cntl field
    ble_rxdone_setf(hdl, 0);
}

uint8_t *em_buf_rx_buff_addr_get(uint16_t rx_hdl)
{
    return (uint8_t *)(REG_BLE_EM_RX_BUFFER_BASE_ADDR + ble_rxdataptr_get(rx_hdl));
}

uint8_t *em_buf_tx_buff_addr_get(struct em_buf_tx_desc *tx_desc)
{
    return (uint8_t *)(REG_BLE_EM_TX_BUFFER_DATA_BASE_ADDR + tx_desc->txdataptr);
}

bool em_buf_tx_free(struct em_desc_node *desc_to_be_freed)
{
    uint16_t dle_field = ble_txdle_get(desc_to_be_freed->idx);
    bool buffer_flushed = false;
    GLOBAL_INT_DIS();

    // If the buffer can be freed
    if(dle_field & BLE_FREEBUFF_BIT)
    {
        buffer_flushed = true;
        #if (BLE_TX_BUFFER_CNT > 0)
        // Get the idex of the buffer in the descriptor and free the buffer
        em_buf_tx_buff_free(dle_field & BLE_BUFFIDX_MASK);
        #endif //(BLE_TX_BUFFER_CNT > 0)
    }

    // Free the descriptor
    em_buf_tx_desc_free(desc_to_be_freed);
    GLOBAL_INT_RES();
    return (buffer_flushed);
}



struct em_buf_tx_desc *em_buf_tx_desc_addr_get(uint16_t idx)
{
  // Pop a descriptor from the TX free list
  return(&em_buf_env.tx_desc[idx]);
}

struct em_desc_node *em_buf_tx_desc_node_get(uint16_t idx)
{
  // Pop a descriptor from the TX free list
  return(&em_buf_env.tx_desc_node[idx]);
}

uint8_t em_buf_rx_current_get(void)
{
  return (em_buf_env.rx_current);
}

void em_buf_rx_current_set(uint8_t hdl)
{
  em_buf_env.rx_current = hdl;
}

uint8_t em_buf_rx_next(uint8_t hdl)
{
  return ((hdl + 1) % BLE_RX_BUFFER_CNT);
}


/**
 ****************************************************************************************
 * @brief Allocation of a TX data descriptor
 *
 * @return The pointer to the TX descriptor corresponding to the allocated buffer, NULL if
 *         no buffers are available anymore.
 *
 ****************************************************************************************
 */
  struct em_desc_node* em_buf_tx_desc_alloc(void)
{
    return((struct em_desc_node *)common_list_pop_front(&em_buf_env.tx_desc_free));
}
/**
 ****************************************************************************************
 * @brief Allocation of a TX data descriptor and a TX data buffer
 *
 * @return The pointer to the TX descriptor corresponding to handle
 *
 ****************************************************************************************
 */
  struct em_buf_node *em_buf_tx_alloc(void)
{
    struct em_buf_node *node = NULL;
    GLOBAL_INT_DIS();
    // Get free element from free list
    node = (struct em_buf_node *) common_list_pop_front(&em_buf_env.tx_buff_free);
    GLOBAL_INT_RES();
    return node;
}

/**
 ****************************************************************************************
 * @brief Returns the pointer to the TX descriptor from the index
 *
 * @return The pointer to the TX descriptor corresponding to the index.
 *
 ****************************************************************************************
 */
  struct em_buf_tx_desc *em_buf_tx_desc_get(uint16_t idx)
{
    // Pop a descriptor from the TX free list
    return(&em_buf_env.tx_desc[idx]);
}
/**
 ****************************************************************************************
 * @brief Freeing of a TX descriptor
 *
 * @param desc  The pointer to the TX descriptor to be freed
 *
 ****************************************************************************************
 */
  void em_buf_tx_desc_free(struct em_desc_node *desc)
{
    //Clear Descriptor fields
    ble_txphce_set(desc->idx, 0);
    ble_txdle_set(desc->idx, 0);
    ble_txdataptr_set(desc->idx, 0);
    // Push back the descriptor in the TX free list
    common_list_push_back(&em_buf_env.tx_desc_free, &desc->hdr);
}

#if (BLE_TX_BUFFER_CNT > 0)
/**
 ****************************************************************************************
 * @brief Freeing of a TX buffer
 *
 * @param desc  The pointer to the TX descriptor to be freed
 *
 ****************************************************************************************
 */
  void em_buf_tx_buff_free(int idx)
{
    struct em_buf_node *buff_node = &em_buf_env.tx_buff_node[idx];
    // Push back the buffer in the TX free list
    common_list_push_back(&em_buf_env.tx_buff_free, &buff_node->hdr);
}
#endif // (BLE_TX_BUFFER_CNT > 0)

/**
 ****************************************************************************************
 * @brief Returns the pointer to the TX node from the index
 *
 * @return The pointer to the TX node corresponding to the index.
 *
 ****************************************************************************************
 */
  struct em_desc_node *em_buf_node_get(uint16_t idx)
{
    // Pop a descriptor from the TX free list
    return(&em_buf_env.tx_desc_node[idx]);
}



/**
 ****************************************************************************************
 * @brief Read a 8bits value in EM
 *
 * @param[in] em_addr  Exchange memory address
 *
 * @return 8bits value
 *
 ****************************************************************************************
 */
  uint16_t em_read_8(uint16_t em_addr)
{
    uint8_t res;
    /// opcode
    em_rd(&res, em_addr, 1);

    return res;
}


/**
 ****************************************************************************************
 * @brief Read a 16bits value in EM
 *
 * @param[in] em_addr  Exchange memory address
 *
 * @return 16bits value
 *
 ****************************************************************************************
 */
  uint16_t em_read_16(uint16_t em_addr)
{
    uint16_t res;
    /// opcode
    em_rd(&res, em_addr, 2);

    return common_btohs(res);
}

/**
 ****************************************************************************************
 * @brief Read a 32bits value in EM
 *
 * @param[in] em_addr  Exchange memory address
 *
 * @return 32bits value
 *
 ****************************************************************************************
 */
 uint32_t em_read_32(uint16_t em_addr)
{
    uint32_t res;
    /// opcode
    em_rd(&res, em_addr, 4);

    return common_btohl(res);
}



#endif //BLE_EMB_PRESENT
/// @} EM_BUF
