/**
 ****************************************************************************************
 *
 * @file lld_pdu.c
 *
 * @brief Implementation of the functions for adv/acl/llcp transmission/reception
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLDPDU
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwble_config.h"
#include "lowlevel.h"
#include "lld.h"
#include "lld_pdu.h"

#include "kernel_event.h"
#include "kernel_mem.h"
#include "lld_util.h"
#include "llcontrl.h"
#include "llc_util.h"
#include "llc_llcp.h"
#include "llm.h"
#include "llm_util.h"
#include "ea.h"
#include "reg_ble_em_tx_desc.h"
#include "reg_ble_em_rx_desc.h"
#include "reg_ble_em_cs.h"
#if (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
#include "lld_wlcoex.h"
#endif //(RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
#if(BLE_AUDIO)
#include "audio.h"
#endif //BLE_AUDIO

#include "rwble.h"
/*
 * DEFINES
 ****************************************************************************************
 */
#define HALF_SLOT_SIZE_BOUNDARY 0xFFFE

//Base index for the data descriptor
#define DATA_BASE_INDEX (BLE_TX_DESC_CNTL + BLE_TX_DESC_ADV -1)

#define UNPK(msgstr)                                                        \
    (llcp_pdu_unpk_func_t)(lld_pdu_##msgstr##_unpk)

#define UNPK_AL(msgstr)                                                   \
    (llcp_pdu_unpk_func_t)lld_pdu_cntl_aligned_unpk

#define UNPK_NONE()  NULL

/// Status returned by generic packer-unpacker
enum lld_pdu_pack_status
{
    /// Pack succeed
    LLC_PDU_PACK_OK,
    /// Pack with wrong format
    LLC_PDU_PACK_WRONG_FORMAT,
    /// Unknown PDU to pack
    LLC_PDU_PACK_UNKNOWN,
};

#if (BLE_CENTRAL || BLE_PERIPHERAL)
static void lld_pdu_cntl_aligned_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t *param);
#if !(BLE_QUALIF)
static void lld_pdu_llcp_con_param_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param);
static void lld_pdu_llcp_con_param_rsp_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param);
static void lld_pdu_llcp_length_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param);
static void lld_pdu_llcp_length_rsp_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param);
#if (BLE_2MBPS)
static void lld_pdu_llcp_phy_upd_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param);
#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)
static uint8_t lld_pdu_pack(uint8_t* p_data, uint8_t* p_length, const char* format);

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// PDU unpack function pointer type definition
typedef void (*llcp_pdu_unpk_func_t)(uint16_t pdu_ptr, uint8_t parlen, uint8_t *param);


// Information of received PDU
struct lld_pdu_rx_info
{
    /// list header
    struct common_list_hdr hdr;
    /// RX Descriptor index
    uint8_t rx_hdl;
    /// indicate that rx_buffer has been freed
    bool    free;
    /// Connection handle
    uint16_t conhdl;
    /// received status field
    uint16_t status;
    /// packet PDU length
    uint8_t length;
    /// Channel index used
    uint8_t  channel;
    /// RSSI value
    uint8_t  rssi;
    /// True: Audio packet
    uint8_t  audio;
};

/// PDU Packer/Unpacker descriptor structure
struct lld_pdu_pack_desc
{
    /// LLCP parameters length
    uint8_t                 pdu_len;
    /// Parameters format string
    void*                   pack_fmt;
    /// PDU unpacking handler
    llcp_pdu_unpk_func_t    unpack_func;
};

/// ADV packets descriptors
const struct lld_pdu_pack_desc lld_pdu_adv_pk_desc_tab[] =
{
    [LL_ADV_CONN_UNDIR       ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
    [LL_ADV_CONN_DIR         ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
    [LL_ADV_NONCONN_UNDIR    ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
    [LL_SCAN_REQ             ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
    [LL_SCAN_RSP             ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
    [LL_CONNECT_REQ          ] = { (LLCP_CON_REQ_LEN - (2*BD_ADDR_LEN))  , "4B3BBHHHH5BB"     , UNPK_NONE() },
    [LL_ADV_DISC_UNDIR       ] = { LLCP_CON_REQ_LEN                      , "B"                , UNPK_NONE() },
};

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/// LLCP packets descriptors
const struct lld_pdu_pack_desc lld_pdu_llcp_pk_desc_tab[] =
{
    [LLCP_CONNECTION_UPDATE_IND_OPCODE] = {LLCP_CON_UPD_IND_LEN             , "BBHHHHH"       ,  UNPK_AL(llcp_con_up_ind       ) },
    [LLCP_CHANNEL_MAP_IND_OPCODE]       = {LLCP_CH_MAP_REQ_LEN              , "B5BH"          ,  UNPK_AL(llcp_channel_map_ind  ) },
    [LLCP_TERMINATE_IND_OPCODE]         = {LLCP_TERM_IND_LEN                , "BB"            ,  UNPK_AL(llcp_terminate_ind    ) },
    [LLCP_ENC_REQ_OPCODE]               = {LLCP_ENC_REQ_LEN                 , "B8B2B8B4B"     ,  UNPK_AL(llcp_enc_req          ) },
    [LLCP_ENC_RSP_OPCODE]               = {LLCP_ENC_RSP_LEN                 , "B8B4B"         ,  UNPK_AL(llcp_enc_rsp          ) },
    [LLCP_START_ENC_REQ_OPCODE]         = {LLCP_ST_ENC_REQ_LEN              , "B"             ,  UNPK_AL(llcp_start_enc_req    ) },
    [LLCP_START_ENC_RSP_OPCODE]         = {LLCP_ST_ENC_RSP_LEN              , "B"             ,  UNPK_AL(llcp_start_enc_rsp    ) },
    [LLCP_UNKNOWN_RSP_OPCODE]           = {LLCP_UNKN_RSP_LEN                , "BB"            ,  UNPK_AL(llcp_unknown_rsp      ) },
    [LLCP_FEATURE_REQ_OPCODE]           = {LLCP_FEAT_REQ_LEN                , "B8B"           ,  UNPK_AL(llcp_feats_req        ) },
    [LLCP_FEATURE_RSP_OPCODE]           = {LLCP_FEAT_RSP_LEN                , "B8B"           ,  UNPK_AL(llcp_feats_rsp        ) },
    [LLCP_PAUSE_ENC_REQ_OPCODE]         = {LLCP_PA_ENC_REQ_LEN              , "B"             ,  UNPK_AL(llcp_pause_enc_req    ) },
    [LLCP_PAUSE_ENC_RSP_OPCODE]         = {LLCP_PA_ENC_RSP_LEN              , "B"             ,  UNPK_AL(llcp_pause_enc_rsp    ) },
    [LLCP_VERSION_IND_OPCODE]           = {LLCP_VERS_IND_LEN                , "BBHH"          ,  UNPK_AL(llcp_vers_ind         ) },
    [LLCP_REJECT_IND_OPCODE]            = {LLCP_REJ_IND_LEN                 , "BB"            ,  UNPK_AL(llcp_reject_ind       ) },
#if !(BLE_QUALIF)
    [LLCP_SLAVE_FEATURE_REQ_OPCODE]     = {LLCP_SLAVE_FEATURE_REQ_LEN       , "B8B"           ,  UNPK_AL(llcp_slave_feature_req) },
    [LLCP_CONNECTION_PARAM_REQ_OPCODE]  = {LLCP_CON_PARAM_REQ_LEN           , "BHHHHBHHHHHHH" ,  UNPK(   llcp_con_param_req    ) },
    [LLCP_CONNECTION_PARAM_RSP_OPCODE]  = {LLCP_CON_PARAM_RSP_LEN           , "BHHHHBHHHHHHH" ,  UNPK(   llcp_con_param_rsp    ) },
    [LLCP_REJECT_IND_EXT_OPCODE]        = {LLCP_REJECT_IND_EXT_LEN          , "BBB"           ,  UNPK_AL(llcp_reject_ind_ext   ) },
    [LLCP_PING_REQ_OPCODE]              = {LLCP_PING_REQ_LEN                , "B"             ,  UNPK_AL(llcp_ping_req         ) },
    [LLCP_PING_RSP_OPCODE]              = {LLCP_PING_RSP_LEN                , "B"             ,  UNPK_AL(llcp_ping_rsp         ) },
    [LLCP_LENGTH_REQ_OPCODE]            = {LLCP_LENGTH_REQ_LEN              , "BHHHH"         ,  UNPK(   llcp_length_req       ) },
    [LLCP_LENGTH_RSP_OPCODE]            = {LLCP_LENGTH_RSP_LEN              , "BHHHH"         ,  UNPK(   llcp_length_rsp       ) },
    #if (BLE_2MBPS)
    [LLCP_PHY_REQ_OPCODE]              = {LLCP_PHY_REQ_LEN                  , "BBB"           ,  UNPK_AL(llcp_phy_req          ) },
    [LLCP_PHY_RSP_OPCODE]              = {LLCP_PHY_RSP_LEN                  , "BBB"           ,  UNPK_AL(llcp_phy_rsp          ) },
    [LLCP_PHY_UPD_IND_OPCODE]          = {LLCP_PHY_UPD_REQ_LEN              , "BBBH"          ,  UNPK(   llcp_phy_upd_req      ) },
    #endif // (BLE_2MBPS)
#endif //#if !(BLE_QUALIF)
};
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Add an element as last on the list.
 *
 * @param list           Pointer to the list structure
 * @param list_hdr       Pointer to the header to add at the end of the list
 *
 ****************************************************************************************
 */
__INLINE void lld_pdu_push_back(struct common_list *list, struct common_list_hdr *list_hdr)
{
    // check if list is empty
    if (list->first == NULL)
    {
        // list empty => pushed element is also head
        list->first = list_hdr;
    }
    else
    {
        // list not empty => update next of last
        list->last->next = list_hdr;
    }

    // add element at the end of the list
    list->last = list_hdr;
    list_hdr->next = NULL;
}
#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Extract the first element of the list.
 * @param list           Pointer to the list structure
 * @return The pointer to the element extracted, and NULL if the list is empty.
 ****************************************************************************************
 */

__INLINE struct common_list_hdr *lld_pdu_pop_front(struct common_list *list)
{
    struct common_list_hdr *element;

    // check if list is empty
    element = list->first;
    if (element != NULL)
    {
        // The list isn't empty : extract the first element
        list->first = list->first->next;

        if(list->first == NULL)
        {
            list->last = list->first;
        }
    }
    return element;
}
/**
 ****************************************************************************************
 * @brief Add an element as first on the list.
 *
 * @param list           Pointer to the list structure
 * @param list_hdr       Pointer to the header to add at the beginning of the list
 ****************************************************************************************
 */
__INLINE void lld_pdu_push_front(struct common_list *list, struct common_list_hdr *list_hdr)
{
    // check if list is empty
    if (list->first == NULL)
    {
        // list empty => pushed element is also head
        list->last = list_hdr;
    }

    // add element at the beginning of the list
    list_hdr->next = list->first;
    list->first = list_hdr;
}


/**
 ****************************************************************************************
 * @brief Called in IRQ context, it fragments the buffer if needed allocates descriptor(s)
 * for the fragmentation, link the descriptor(s) in the TX prog queue and set correctly the
 * length,LLID
 *
 * @param[in]  evt                   Event Pointer on which transmission happen
 * @param[in]  element_to_be_sent    Pointer on ACL data TX element structure to be sent
 * @param[out] nb_packet_prg         Give the number of packet programmed
 * @param[in]  nb_tx_desc_available  Give the number of free descriptor
 * @param[in]  phy                   Current phy used to convert in time the size
 * @param[in]  encrypted             Inform if the link is encrypted
 * @param[out] nb_chunks_sent        Return the number of chunk for the current data buffer
 *
 * Return true if the buffer has been pushed to be transmitted
 ****************************************************************************************
 */
__INLINE bool lld_pdu_send_packet(struct lld_evt_tag *evt, struct lld_pdu_data_tx_tag *element_to_be_sent, uint8_t *nb_packet_prg, uint16_t nb_tx_desc_available,uint8_t phy, bool encrypted)
{
    int16_t temp_length = (int16_t)element_to_be_sent->length;
    uint16_t temp_data_ptr_val = 0;
    uint16_t temp_buff_idx_val = 0;
    bool first_chunk = true;
    uint8_t default_length = evt->evt.conn.eff_max_tx_size;
    uint8_t nb_fragment = 1;

    /**
     * Phase 1 compute the minimum size to be transmitted
     */
    /**
     *Check if the effective transmit time is enough to send only one packet
     */

    // Current packet time
    uint16_t max_tx_time  = BLE_PREAMBLE_TIME;
    uint16_t def_pkt_size = BLE_ACCESS_CODE_SIZE + BLE_HEADER_SIZE + BLE_CRC_SIZE;
    uint16_t byte_size    = temp_length;
    // multiplication time factor for  duration computation
    uint8_t  phy_factor;

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Check if encryption is on
    if(encrypted)
    {
        #if(BLE_AUDIO)
        // If audio mode and the encryption mode is AES with MIC add the MIC length
        if(ble_crypt_mode_getf(evt->conhdl) == AUDIO_ENC_MODE_0)
        #endif//(BLE_AUDIO)
        {
            def_pkt_size += BLE_MIC_SIZE;
        }
    }
    #endif //(BLE_CENTRAL || BLE_PERIPHERAL)

    byte_size += def_pkt_size;

    // Get the current tx rate
    if(phy == LLD_CS_RATE_2MBPS)
    {
        // bit_duration = (bytes*8) / 2
        phy_factor = 2;
    }
    else
    {
        // bit_duration = (bytes*8)
        phy_factor = 3;
    }

    max_tx_time += (byte_size << phy_factor);

    if (evt->evt.conn.eff_max_tx_time < max_tx_time)
    {
        /**
         * Convert the packet duration in BYTE length
         * Calculate Time allocated for the data:effective tx time(us) - preamble(always 8us)-((AA+HEADER+CRC+MIC_IF_PRESENT)(us)/rate)
         * aligned to the rate (1-> 1MBPS, 2->2MBPS))
         * and Convert available time in byte according to the rate
         * */
        // Check how many chunks should be allocated according to the transmit effective time
        default_length = ((evt->evt.conn.eff_max_tx_time - BLE_PREAMBLE_TIME - (def_pkt_size<<phy_factor)) >>phy_factor);

        /**
         *Check if the effective transmit size is enough to send only one packet
         */

        if (default_length > evt->evt.conn.eff_max_tx_size)
        {
            default_length = evt->evt.conn.eff_max_tx_size ;
        }
    }
    /**
     * **********************************************************
     * Patch to solve the AES half-world align issue
     * **********************************************************
     */
    //If the data size to send generates a fragmentation, align the size on half-word boundary
    //And use the new tx effective size to fragment
    if(temp_length > default_length)
    {
        default_length &= HALF_SLOT_SIZE_BOUNDARY;
    }

    // Compute the number of chunks
    nb_fragment +=  ((temp_length-1) / default_length);

    if( nb_fragment > nb_tx_desc_available)
    // If not enough descriptor(s) available to send all the chunks
    {
        return(false);
    }

    *nb_packet_prg = nb_fragment;
    /**
     * Phase 2 fragmentation
     */
    for (;;)
    {
        /**
         * Phase 2 Get a new descriptor
         */
        struct em_desc_node* desc_node = (struct em_desc_node *)em_buf_tx_desc_alloc();
        if(first_chunk)
        {
            first_chunk = false;
            /**
             * Phase 1 Compute the information for the 1st chunk
             */
            // Format the data packet: set the length, llid and md
            desc_node->llid = ((element_to_be_sent->pb_bc_flag & BLE_TXLLID_MASK)==LLID_CONTINUE)?LLID_CONTINUE:LLID_START;

            // Get the pointer value on the data
            temp_data_ptr_val = element_to_be_sent->buf->buf_ptr;
            // Get the buffer index
            temp_buff_idx_val = element_to_be_sent->buf->idx;

        }
        else
        {
            /**
             * Phase 2 Compute the length for the chunk
             */
            desc_node->llid  = LLID_CONTINUE;

            temp_data_ptr_val = (temp_data_ptr_val + default_length) & BLE_TXDATAPTR_MASK;
        }

        if (temp_length > default_length)
        {
            desc_node->length = default_length;
        }
        else
        {
            desc_node->length = temp_length;
        }

        //Fill all the information before pushing the descriptor
        desc_node->buffer_idx   = temp_buff_idx_val;
        desc_node->buffer_ptr   = temp_data_ptr_val;

        /**
         * Phase 2 Compute the remaining size
         */
         temp_length -= default_length;

        // if it is the last chunk
        if(temp_length <= 0)
        {
            lld_pdu_data_tx_push(evt, desc_node, true, encrypted);
            break;
        }
        else
        {
            lld_pdu_data_tx_push(evt, desc_node, false, encrypted);
        }
    }
    return true;
}
/**
 ****************************************************************************************
 * @brief Unpack received PDU onto a LLCP FW structure.
 *
 * @param rx_hdl descriptor handle
 * @param len    length of the pdu
 * @param msg    message pointer to fill
 *
 * @return Status code of the pdu umpack
 ****************************************************************************************
 */
static struct llc_llcp_recv_ind* lld_pdu_llcp_unpack(uint8_t rx_hdl, uint8_t len, struct llc_llcp_recv_ind *msg)
{
    uint16_t rx_data_ptr = ble_rxdataptr_get(rx_hdl);

    // load pdu data
    em_rd(&msg->pdu.opcode, rx_data_ptr, 1);

    // Check opcode
    if (msg->pdu.opcode >= LLCP_OPCODE_MAX_OPCODE)
    {
        msg->status = COMMON_ERROR_UNKNOWN_LMP_PDU;
    }
    else if (len != lld_pdu_llcp_pk_desc_tab[msg->pdu.opcode].pdu_len)
    {
        msg->status = COMMON_ERROR_INVALID_LMP_PARAM; // incorrect length
    }
    else
    {
        msg->status = COMMON_ERROR_NO_ERROR;

        // Unpack the PDU
        lld_pdu_llcp_pk_desc_tab[msg->pdu.opcode].unpack_func(rx_data_ptr+1, len-1 /* op-code already extracted */, (uint8_t *) &msg->pdu.opcode);
    }

    return msg;
}


/* PDU unpacking functions
 ****************************************************************************************/
static void lld_pdu_cntl_aligned_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t *param)
{
    // Copy over params (if any)
    if (parlen != 0)
    {
        // Parameters are aligned, just copy them (op code already copied so not needed)
        em_rd(param+1, pdu_ptr, parlen);
    }
}

#if !(BLE_QUALIF)
/**
 ******************************************************************************************
 * @brief GENERAL COMMENT FOR llcp_..._pdu_unpk : LMP PDU param extraction function
 *
 * @param[in] pdu      Pointer to PDU buffer, without the 1 or two opcode bytes.
 * @param[in] parlen   Length of left over pdu params.
 * @param[in] param    Pointer to kernel message param position for direct copy of pdu params
 *
 ******************************************************************************************
 */
static void lld_pdu_llcp_con_param_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param)
{
    //the first two params of the struct have been filled before this fcn call
    struct llcp_con_param_req * s = (struct llcp_con_param_req *)param;
    /// minimum value of connInterval
    s->interval_min = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum value of connInterval
    s->interval_max = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// connSlaveLatency value
    s->latency = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// connSupervisionTimeout value
    s->timeout = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// preferred periodicity
    s->pref_period = em_read_8(pdu_ptr);
    pdu_ptr++;
    /// ReferenceConnEventCount
    s->ref_con_event_count = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset0
    s->offset0 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset1
    s->offset1 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset2
    s->offset2 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset3
    s->offset3 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset4
    s->offset4 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset5
    s->offset5 = em_read_16(pdu_ptr);

}

/**
 ******************************************************************************************
 * @brief GENERAL COMMENT FOR llcp_..._pdu_unpk : LMP PDU param extraction function
 *
 * @param[in] pdu      Pointer to PDU buffer, without the 1 or two opcode bytes.
 * @param[in] parlen   Length of left over pdu params.
 * @param[in] param    Pointer to kernel message param position for direct copy of pdu params
 *
 ******************************************************************************************
 */
static void lld_pdu_llcp_con_param_rsp_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param)
{
    //the first two params of the struct have been filled before this fcn call
    struct llcp_con_param_rsp * s = (struct llcp_con_param_rsp *)param;
    /// minimum value of connInterval
    s->interval_min = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum value of connInterval
    s->interval_max = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// connSlaveLatency value
    s->latency = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// connSupervisionTimeout value
    s->timeout = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// preferred periodicity
    s->pref_period = em_read_8(pdu_ptr);
    pdu_ptr++;
    /// ReferenceConnEventCount
    s->ref_con_event_count = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset0
    s->offset0 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset1
    s->offset1 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset2
    s->offset2 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset3
    s->offset3 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset4
    s->offset4 = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// Offset5
    s->offset5 = em_read_16(pdu_ptr);
}

/**
 ******************************************************************************************
 * @brief GENERAL COMMENT FOR llcp_..._pdu_unpk : LMP PDU param extraction function
 *
 * @param[in] pdu      Pointer to PDU buffer, without the 1 or two opcode bytes.
 * @param[in] parlen   Length of left over pdu params.
 * @param[in] param    Pointer to kernel message param position for direct copy of pdu params
 *
 ******************************************************************************************
 */
static void lld_pdu_llcp_length_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param)
{
    //the first two params of the struct have been filled before this func call
    struct llcp_length_req * s = (struct llcp_length_req *)param;
    /// maximum rx size
    s->max_rx_octets = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum rx time
    s->max_rx_time = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum tx size
    s->max_tx_octets = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum tx time
    s->max_tx_time = em_read_16(pdu_ptr);
}

/**
 ******************************************************************************************
 * @brief GENERAL COMMENT FOR llcp_..._pdu_unpk : LMP PDU param extraction function
 *
 * @param[in] pdu      Pointer to PDU buffer, without the 1 or two opcode bytes.
 * @param[in] parlen   Length of left over pdu params.
 * @param[in] param    Pointer to kernel message param position for direct copy of pdu params
 *
 ******************************************************************************************
 */
static void lld_pdu_llcp_length_rsp_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param)
{
    //the first two params of the struct have been filled before this fcn call
    struct llcp_length_rsp * s = (struct llcp_length_rsp *)param;
    /// maximum rx size
    s->max_rx_octets = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum rx time
    s->max_rx_time = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum tx size
    s->max_tx_octets = em_read_16(pdu_ptr);
    pdu_ptr += 2;
    /// maximum tx time
    s->max_tx_time = em_read_16(pdu_ptr);
}

#if (BLE_2MBPS)
/**
 ******************************************************************************************
 * @brief GENERAL COMMENT FOR llcp_..._pdu_unpk : LMP PDU param extraction function
 *
 * @param[in] pdu      Pointer to PDU buffer, without the 1 or two opcode bytes.
 * @param[in] parlen   Length of left over pdu params.
 * @param[in] param    Pointer to kernel message param position for direct copy of pdu params
 *
 ******************************************************************************************
 */
static void lld_pdu_llcp_phy_upd_req_unpk(uint16_t pdu_ptr, uint8_t parlen, uint8_t * param)
{
    //the first two params of the struct have been filled before this func call
    struct llcp_phy_upd_req * s = (struct llcp_phy_upd_req *)param;
    /// master to slave phy
    s->m_to_s_phy = em_read_8(pdu_ptr);
    pdu_ptr += 1;
    /// Slave to master phy
    s->s_to_m_phy = em_read_8(pdu_ptr);
    pdu_ptr +=1;
    /// Instant
    s->instant = em_read_16(pdu_ptr);
}
#endif // (BLE_2MBPS)
#endif // !(BLE_QUALIF)

/**
 ******************************************************************************************
 * @brief Generic LLCP pdu pack function
 *
 * @param[in] buf      Pointer to PDU buffer
 * @param[in] p_len   Length of left over pdu params.
 *
 ******************************************************************************************
 */
static uint8_t lld_pdu_llcp_pack(uint8_t* buf, uint8_t* p_len)
{
    uint8_t status = LLC_PDU_PACK_UNKNOWN;
    const struct lld_pdu_pack_desc* desc = NULL;
    uint8_t code = ((*buf & LLCP_OPCODE_MASK) >> LLCP_OPCODE_POS);

    // Find LLCP descriptor
    ASSERT_INFO((code < LLCP_OPCODE_MAX_OPCODE), code, code);

    desc = &lld_pdu_llcp_pk_desc_tab[code];

    // Check if LLCP desc found
    if(desc != NULL)
    {
        if(desc->pack_fmt != NULL)
        {
            // Pack the parameters
            status = lld_pdu_pack(buf, p_len, desc->pack_fmt);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// Extract length of an array from a format string
static uint16_t read_array_size(char **fmt_cursor)
{
    // Read size
    uint16_t size = 0;

    // Sanity check
    ASSERT_ERR(fmt_cursor);

    // Convert unit
    size = (*(*fmt_cursor)++) - '0';

    while(((*(*fmt_cursor)) >= '0') && ((*(*fmt_cursor)) <= '9'))
    {
        // Convert tens
        size = 10 * size + ((*(*fmt_cursor)++) - '0');
    }

    // Return the read size
    return (size);
}

/**
 ****************************************************************************************
 * @brief Pack the LLCP SW structure to a PDU air data
 *
 * @param p_data    PDU data
 * @param p_length  PDU Length
 * @param format    Format string of the PDU
 *
 * @return packing status
 ****************************************************************************************
 */
static uint8_t lld_pdu_pack(uint8_t* p_data, uint8_t* p_length, const char* format)
{
    uint8_t status = LLC_PDU_PACK_OK;
    uint8_t* p_in = p_data;
    uint8_t* p_out = p_data;
    char* cursor = (char*) format;

    ASSERT_ERR(format != NULL);

    while((*cursor != '\0') && (status == LLC_PDU_PACK_OK))
    {
        uint16_t nb = 0;

        // Check if the new field is an array (starting with a number)
        if((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = read_array_size(&cursor);
        }

        // Parse the format string
        switch (*cursor++)
        {
            case ('B'): // Byte
            {
                // Copy data and Move pointers
                *p_out++ = *p_in++;

                // For arrays only
                if(nb > 1)
                {
                    // Copy bytes
                    memcpy(p_out, p_in, nb-1);

                    // Move pointers
                    p_out += (nb-1);
                    p_in += (nb-1);
                }
            }
            break;

            case ('H'): // Short Word
            {
                // Align data buffer to a 16-bits address
                uint16_t *short_word = (uint16_t *)COMMON_ALIGN2_HI((uint32_t)p_in);

                // Copy data
                common_write16p(p_out, *short_word);

                // Move pointers
                p_in = (uint8_t *)(short_word + 1);
                p_out += 2;
            }
            break;

            case ('L'): // Long Word
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)COMMON_ALIGN4_HI((uint32_t)p_in);

                // Copy data
                common_write32p(p_out, *long_word);

                // Move pointers
                p_in = (uint8_t *)(long_word + 1);
                p_out += 4;
            }
            break;

            default:
            {
                // data format error
                status = LLC_PDU_PACK_WRONG_FORMAT;
            }
            break;
        }
    }

    if(status == LLC_PDU_PACK_OK)
    {
        *p_length = (uint16_t)(p_out - p_data);
    }
    else
    {
        ASSERT_INFO(0, status, 0);
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Flush all the packets pending for transmission in the specified list
 *
 * @param[in] list   List of buffers that have to be flushed
 *
 * @return The number of TX buffers that have been flushed
 *
 ****************************************************************************************
 */
static uint8_t lld_pdu_tx_flush_list(struct common_list *list)
{
    uint8_t tx_cnt = 0;

    // Go through the list to flush the data
    while (1)
    {
        // Pop the first descriptor from the list
        struct em_desc_node *txnode = NULL;

        GLOBAL_INT_DIS();
        txnode = (struct em_desc_node*)common_list_pop_front(list);
        GLOBAL_INT_RES();
        // If we reach the end of the list, then we exit the loop
        if (txnode == NULL)
            break;

        // Increment the TX counter only if this is a data buffer
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        if ((txnode->idx > DATA_BASE_INDEX) && (txnode->idx < BLE_TX_DESC_CNT))
        {
            tx_cnt++;

            // Free the buffer
            GLOBAL_INT_DIS();
            em_buf_tx_free(txnode);
            GLOBAL_INT_RES();
        }
        else if ((txnode->idx < BLE_TX_BUFF_CNTL) || (txnode->idx >= BLE_TX_DESC_CNT))
        {
            kernel_free(txnode);
        }
        #endif //(BLE_CENTRAL || BLE_PERIPHERAL)
    }

    return (tx_cnt);
}




bool lld_pdu_check(struct lld_evt_tag *evt)
{
    // Get first deferred element
    struct lld_pdu_rx_info *info = NULL;
    struct lld_pdu_data_tx_tag *tx_tofree = NULL;
    struct common_list tmp_pdu_list;
    common_list_init(&tmp_pdu_list);
    // Get first programmed tx node
    struct em_desc_node *txnode;
    uint16_t conhdl = evt->conhdl;
    uint8_t tx_cnt = 0;
    bool elt_deleted = false;



    // ***********************************************************************************
    // ******* TX Check
    // ***********************************************************************************

    GLOBAL_INT_DIS();
    txnode = (struct em_desc_node *)common_list_pick(&evt->tx_prog);
    GLOBAL_INT_RES();

    while (txnode)
    {
        // Check if packet has been transmitted and acknowledged
        if (ble_txdone_getf(txnode->idx))
        {
            struct em_desc_node *next;

            GLOBAL_INT_DIS();
            next = (struct em_desc_node *)(txnode->hdr.next);
            //Pop the acknowledged tx packet
            txnode = (struct em_desc_node *)common_list_pop_front(&evt->tx_prog);
            if (!common_list_is_empty(&evt->tx_acl_tofree))
            {
                tx_tofree = (struct lld_pdu_data_tx_tag *)common_list_pop_front(&evt->tx_acl_tofree);
                if (tx_tofree)
                {
                    kernel_free(tx_tofree);
                }
            }
            GLOBAL_INT_RES();

            // clear next pointer
            ble_nextptr_setf(txnode->idx, 0);

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            /**
             * LLCP tx confirmation
             */
            if(txnode->idx < BLE_TX_DESC_CNTL)
            {
                struct llcp_pdu_tag *llcp_elt = (struct llcp_pdu_tag *)txnode;

                evt->evt.conn.tx_prog_pkt_cnt--;
                GLOBAL_INT_DIS();
                if(common_list_is_empty(&evt->tx_llcp_pdu_rdy))
                {
                    // no more LLCP packet to transmit
                    LLD_EVT_FLAG_RESET(evt, WAITING_TXPROG);
                }
                kernel_free(txnode);
                GLOBAL_INT_RES();
                // inform that LLCP packet has been acknowledged
                llc_pdu_llcp_tx_ack_defer(conhdl, llcp_elt->opcode);
            }
            /**
             * ACL tx confirmation
             */
            else if (txnode->idx > DATA_BASE_INDEX)
            {

                // If the all the chunks have been acknowledged
                if(em_buf_tx_free(txnode))
                {
                    //Buffer is sent, increase the tx counter
                    tx_cnt++;
                }
            }
            /**
             * ADV,SCAN_REQ and CONNECT_REQ tx confirmation
             */
            else
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
            {
                tx_cnt++;
            }

            txnode = next;
        }
        else
        {
            // Escape from the loop
            break;
        }
    }

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // check if it's a connection handle
    if((conhdl != LLD_ADV_HDL) && (tx_cnt > 0))
    {
        GLOBAL_INT_DIS();
        ASSERT_INFO(evt->evt.conn.tx_prog_pkt_cnt >= tx_cnt, conhdl, tx_cnt);
        evt->evt.conn.tx_prog_pkt_cnt -= tx_cnt;
        GLOBAL_INT_RES();

        llc_pdu_acl_tx_ack_defer(conhdl, tx_cnt);
    }
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)


    // ***********************************************************************************
    // ******* RX Check
    // ***********************************************************************************
    GLOBAL_INT_DIS();
    info = (struct lld_pdu_rx_info *)common_list_pop_front(&lld_evt_env.rx_pkt_deferred);
    GLOBAL_INT_RES();

    // load all elements in rx queue
    while (info != NULL)
    {
        // check if descriptor is for current connection handle
        if(conhdl == info->conhdl)
        {
            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            // check if it's a connection handle
            if(info->conhdl != LLD_ADV_HDL)
            {
                llc_pdu_defer(info->conhdl, info->status, info->rssi, info->channel, info->length);
            }
            else
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
            #if (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)
            {
                elt_deleted = llm_pdu_defer(info->status, info->rx_hdl, tx_cnt);
            }
            #endif // (BLE_OBSERVER || BLE_CENTRAL || BLE_PERIPHERAL)

            // free deferred structure
            kernel_free(info);
        }
        else
        {
            // put element in temp list
            common_list_push_back(&tmp_pdu_list, &(info->hdr));
        }

        GLOBAL_INT_DIS();
        info = (struct lld_pdu_rx_info *)common_list_pop_front(&lld_evt_env.rx_pkt_deferred);

        // no more elements, copy temp list to defered list
        if(info == NULL)
        {
            if(!common_list_is_empty(&tmp_pdu_list))
            {
                common_list_merge(&lld_evt_env.rx_pkt_deferred, &tmp_pdu_list);
            }
        }
        GLOBAL_INT_RES();
    }
    return(elt_deleted);
}

void lld_pdu_tx_loop(struct lld_evt_tag *evt)
{
    struct common_list *acl_rdy = &evt->tx_acl_rdy;

    GLOBAL_INT_DIS();
    // Sanity checks
    ASSERT_ERR(common_list_is_empty(&evt->tx_prog));
    ASSERT_ERR(!common_list_is_empty(acl_rdy));

    // Link the last descriptor with the first descriptor
    ble_nextptr_setf(((struct em_desc_node *)(acl_rdy->last))->idx,
                     REG_BLE_EM_TX_DESC_ADDR_GET(((struct em_desc_node *)(acl_rdy->first))->idx));

    GLOBAL_INT_RES();
}


void lld_pdu_data_tx_push(struct lld_evt_tag *evt , struct em_desc_node* txnode, bool can_be_freed, bool encrypted)
{
    // Get list of data packets ready for programming
    struct common_list *list = &evt->tx_prog;

    uint8_t idx = txnode->idx;

    // Sanity check: TX node should be valid
    ASSERT_ERR(txnode);

    #if (BLE_EM_PRESENT)
    // Reset the Control Field of the TX descriptor
    ble_txcntl_set(idx, 0);
    //Fill all the information before pushing the descriptor
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    // Check if encryption is on
    if(encrypted)
    {
        #if (BLE_AUDIO)
        // If audio mode and the encryption mode is AES with MIC add the MIC length
        if(ble_crypt_mode_getf(evt->conhdl) == AUDIO_ENC_MODE_0)
        #endif//(BLE_AUDIO)
        {
            txnode->length += BLE_MIC_SIZE;
        }
    }
    #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
    // As the MD, SN and NESN mgt is done by HW we can set the value instead of setf
    ble_txphce_set(idx,(txnode->length << BLE_TXLEN_LSB) | txnode->llid );
    ble_txdataptr_set(idx, txnode->buffer_ptr);

    // if it is the last chunk
    if(can_be_freed)
    {
        ble_txdle_set(idx, BLE_FREEBUFF_BIT | txnode->buffer_idx);
        evt->evt.conn.tx_prog_pkt_cnt ++;
    }
    #else
    #endif //(BLE_EM_PRESENT)


    // Link the new descriptor in exchange memory
    if (list->first != NULL)
    {
        ble_nextptr_setf(((struct em_desc_node *)(list->last))->idx,
                                                    REG_BLE_EM_TX_DESC_ADDR_GET(txnode->idx));
    }

    // Push the descriptor at the end of the TX list
    lld_pdu_push_back(list, (struct common_list_hdr *)txnode);
}
#if (BLE_PERIPHERAL || BLE_CENTRAL)
bool lld_pdu_data_send(void *param)
{
    struct lld_pdu_data_tx_tag *data_tx = (struct lld_pdu_data_tx_tag*)kernel_malloc(sizeof(struct lld_pdu_data_tx_tag), KERNEL_MEM_ENV);
    uint8_t status = false;
    if(data_tx)
    {
        struct hci_acl_data_tx* data_to_tx = (struct hci_acl_data_tx*)param;
        // Get associated BLE event environment
        struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(llc_env[data_to_tx->conhdl]->elt);
        // Get list of data packets ready for programming
        struct common_list *list = &evt->tx_acl_rdy;

        data_tx->buf        = data_to_tx->buf;
        data_tx->conhdl     = data_to_tx->conhdl;
        data_tx->length     = data_to_tx->length;
        data_tx->pb_bc_flag = data_to_tx->pb_bc_flag;
        data_tx->idx        = BLE_TX_DESC_CNT;
        GLOBAL_INT_DIS();
        // Push the llcp pdu allocated at the end of the TX llcp pending list
        common_list_push_back(list, &data_tx->hdr);
        GLOBAL_INT_RES();
        status = true;
    }
    return (status);
}
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

void lld_pdu_tx_push(struct ea_elt_tag *elt, struct em_desc_node *txnode)
{
    // Get associated BLE event environment
    struct lld_evt_tag *evt = LLD_EVT_ENV_ADDR_GET(elt);
    // Get list of data packets ready for programming
    struct common_list *list = &evt->tx_acl_rdy;

    // Sanity check: TX node should be valid
    ASSERT_ERR(txnode);

    #if (BLE_EM_PRESENT)
    // Reset the Control Field of the TX descriptor
    ble_txcntl_set(txnode->idx, 0);
    #else
    #endif //(BLE_EM_PRESENT)

    // Disable the interrupts as the list can be modified under interrupt
    GLOBAL_INT_DIS();

    // Link the new descriptor in exchange memory
    if (!common_list_is_empty(list))
    {
        ble_nextptr_setf(((struct em_desc_node *)(list->last))->idx,
                                                    REG_BLE_EM_TX_DESC_ADDR_GET(txnode->idx));
    }

    // Push the descriptor at the end of the TX list
    common_list_push_back(list, (struct common_list_hdr *)txnode);

    // Restore the interrupts
    GLOBAL_INT_RES();
}

void lld_pdu_tx_prog(struct lld_evt_tag *evt)
{
    uint16_t conhdl                 = evt->conhdl;
    struct common_list *acl_rdy         = &evt->tx_acl_rdy;

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    struct common_list *llcp_pdu_rdy    = &evt->tx_llcp_pdu_rdy;
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    struct common_list *tx_prog         = &evt->tx_prog;
    uint16_t tx_add                 = 0;
    bool send_acl                   = true;
    struct em_desc_node *txnode = NULL;

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    /**
     * LLCP management
     */
    // Check if LLCP is pending to be sent and no LLCP sent wait acknowledgment
    if (!common_list_is_empty(llcp_pdu_rdy) && ble_txdone_getf(conhdl))
    {
        // Pick the 1st llcp pdu in TX llcp pending list
        struct llcp_pdu_tag *llcp_elt = (struct llcp_pdu_tag *)common_list_pick(llcp_pdu_rdy);
        // Check if we are in state where the LLCP authorization is needed
        uint8_t enc_state = LLC_UTIL_ENC_STATE_GET(conhdl);
        bool llcp_pdu_authorized = true;
        // The llcp buffer is linked with the conhdl
        uint8_t length;
        uint8_t *param ;
        uint8_t status;
        // Check if we have the correct authorization
        if(enc_state & LLC_ENC_TX_FLOW_OFF)
        {
            struct llcp_pdu_tag *next;

            //Parse the llcp list to find the allowed
            while (llcp_elt)
            {
                // Set the next element to the current
                next = llcp_elt;

                #if (BLE_TESTER)
                if(llcp_elt->opcode == LLCP_OPCODE_DEBUG)
                {
                    llcp_pdu_authorized = true;
                }
                else
                #endif // BLE_TESTER
                // Pause encryption state
                if(enc_state & LLC_ENC_PAUSE_PENDING)
                {
                    if(!(llc_llcp_get_autorize(llcp_elt->opcode) & LLC_LLCP_PAUSE_ENC_AUTHZED))
                    {
                        llcp_pdu_authorized= false;
                    }
                    else
                    {
                        llcp_pdu_authorized = true;
                    }
                }
                // Start encryption state
                else
                {
                    if(!(llc_llcp_get_autorize(llcp_elt->opcode) & LLC_LLCP_START_ENC_AUTHZED))
                    {
                        llcp_pdu_authorized = false;
                    }
                    else
                    {
                        llcp_pdu_authorized = true;
                    }
                }
                //LLCP not authorized
                if(!llcp_pdu_authorized)
                {
                    llcp_elt = ( struct llcp_pdu_tag *)next->hdr.next;
                }
                else //LLCP authorized
                {
                    break;
                }
            } // while (llcp_elt)
        } // if(enc_state & LLC_ENC_TX_FLOW_OFF)


        if(llcp_pdu_authorized)
        {
            //Extract the LLCP to program
            common_list_extract(llcp_pdu_rdy, &llcp_elt->hdr, 0);

            // The llcp buffer is linked with the conhdl
            param = (uint8_t*)llcp_elt->ptr;
            #if (BLE_TESTER)
            if(llcp_elt->opcode == LLCP_OPCODE_DEBUG)
            {
                length = llcp_elt->pdu_length;
                status = LLC_PDU_PACK_OK;
            }
            else
            #endif // BLE_TESTER
            {
                status = lld_pdu_llcp_pack(param, &length);
            }

            // Check packing status and if the LLCP is authorized to be sent during start or pause encrypt
            if(status == LLC_PDU_PACK_OK)
            {
                em_wr(param, ble_txdataptr_get(llcp_elt->idx), length);

                //Free temporary LLCP
                kernel_free(llcp_elt->ptr);


                if(LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_TX))
                {
                    #if(BLE_AUDIO)
                    // If audio mode and the encryption mode is AES with MIC add the MIC length
                    if(ble_crypt_mode_getf(conhdl) == AUDIO_ENC_MODE_0)
                    #endif//(BLE_AUDIO)
                    {
                        length += MIC_LEN;
                    }
                }

                ble_txlen_setf(conhdl, length);
                // Set LLID to 0x3 -> LLCP
                ble_txllid_setf(conhdl, BLE_TXLLID_MASK);
                //Clear Tx done and next pointer
                ble_txcntl_pack(evt->conhdl, 0, 0);

                if (!common_list_is_empty(tx_prog))
                {
                    // Chain the HW descriptors
                    ble_nextptr_setf(((struct em_desc_node *)tx_prog->last)->idx,
                            REG_BLE_EM_TX_DESC_ADDR_GET(llcp_elt->idx));
                }
                // Add element in the list
                common_list_push_back(tx_prog, &llcp_elt->hdr);
                evt->evt.conn.tx_prog_pkt_cnt++;
            }
        }// if(llcp_pdu_authorized)
    }
    #endif //  (BLE_CENTRAL || BLE_PERIPHERAL)

    /**
     * ACL management
     */
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // If connected link and flow off
    if((evt->conhdl < BLE_CONNECTION_MAX) && LLC_UTIL_ENC_STATE_IS_SET(conhdl, LLC_ENC_TX_FLOW_OFF))
    {
        send_acl = false;
    }
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    if (!common_list_is_empty(acl_rdy) && (send_acl))
    {
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        struct lld_pdu_data_tx_tag *txacl = NULL;
        if((evt->conhdl < BLE_CONNECTION_MAX) )
        {
            uint16_t nb_tx_desc_available   = common_list_size(&em_buf_env.tx_desc_free);
            uint8_t total_packet_prog       = 0;
            uint8_t phy                     = ble_txrate_getf(evt->conhdl);
            bool encrypted                  = (LLC_UTIL_ENC_STATE_IS_SET(evt->conhdl, LLC_ENC_TX) != 0)? true: false;

            txacl = (struct lld_pdu_data_tx_tag *)lld_pdu_pop_front(acl_rdy);
            while(txacl)
            {
                uint8_t nb_packet_prog = 0;
                // Check if the transmit max effective time is respected
                send_acl = lld_pdu_send_packet(evt, txacl, &nb_packet_prog, nb_tx_desc_available, phy, encrypted);
                // If the packet cannot be sent it should be fragmented
                if(!send_acl)
                {
                    lld_pdu_push_front(acl_rdy,&txacl->hdr);
                    break;
                }
                total_packet_prog       += nb_packet_prog;
                nb_tx_desc_available    -= nb_packet_prog;

                lld_pdu_push_back(&evt->tx_acl_tofree, &txacl->hdr);

                if(total_packet_prog > BLE_NB_MAX_PACKET_PROG)
                {
                    //Like not all the packet has been moved in the programmed queue do not free the ready queue
                    send_acl = false;
                    break;
                }
                txacl = (struct lld_pdu_data_tx_tag *)lld_pdu_pop_front(acl_rdy);
            }
        }
        else
        #endif //  (BLE_CENTRAL || BLE_PERIPHERAL)
        {
            common_list_merge(tx_prog, acl_rdy);
        }
        // Transmit effective time not respected, do not send the acl packet(s)
        if(send_acl != false)
        {
            // Reset ready list
            common_list_init(acl_rdy);
        }
    }

    // Program the control structure with the first tx descriptor pointer
    if (!common_list_is_empty(tx_prog))
    {
        txnode = (struct em_desc_node *)(tx_prog->first);

        // Some ACKed data packets could not have been handled, get first descriptor with TX Done = 0
        while (txnode)
        {
            if (!ble_txdone_getf(txnode->idx))
            {
                tx_add = REG_BLE_EM_TX_DESC_ADDR_GET(txnode->idx);
                break;
            }

            txnode = (struct em_desc_node *)(txnode->hdr.next);
        }

        // check if there is still something to program
        if(!txnode)
        {
            tx_add = 0;
        }

        #if (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
        ble_cntl_pack(conhdl,RW_BLE_PTI_PRIO_AUTO, RWIP_COEX_GET(CON_DATA, TXBSY), RWIP_COEX_GET(CON_DATA, RXBSY),RWIP_COEX_GET(CON_DATA, DNABORT), ble_format_getf(conhdl));
        #endif //(RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    }
	#if (RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    else
    {
    	ble_cntl_pack(conhdl,RW_BLE_PTI_PRIO_AUTO, RWIP_COEX_GET(CON, TXBSY), RWIP_COEX_GET(CON, RXBSY),RWIP_COEX_GET(CON, DNABORT), ble_format_getf(conhdl));
    }
	#endif //(RW_BLE_WLAN_COEX) || (RW_BLE_MWS_COEX)
    // No empty packet to be chained, so chain the user data
    ble_txdescptr_set(conhdl, tx_add);
}


void lld_pdu_tx_flush(struct lld_evt_tag *evt)
{
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    uint8_t nb_of_pkt_flushed;

    // Free all memory allocated for the LLCP management
    // Clear the pending LLCP

    GLOBAL_INT_DIS();
    struct common_list *list = &evt->tx_llcp_pdu_rdy;
    while(!common_list_is_empty(list))
    {
        struct llcp_pdu_tag *llcp_elt = (struct llcp_pdu_tag *)common_list_pop_front(list);
        kernel_free(llcp_elt->ptr);
        kernel_free(llcp_elt);
    }
    GLOBAL_INT_RES();

    // Flush TX data - Programmed and then Ready lists
    nb_of_pkt_flushed = lld_pdu_tx_flush_list(&evt->tx_prog);
    nb_of_pkt_flushed += lld_pdu_tx_flush_list(&evt->tx_acl_rdy);
    nb_of_pkt_flushed += lld_pdu_tx_flush_list(&evt->tx_acl_tofree);

    // if the number of packet flushed is not NULL send a number of packets
    if (nb_of_pkt_flushed > 0)
    {
        llc_common_nb_of_pkt_comp_evt_send(evt->conhdl, nb_of_pkt_flushed);
    }
    #else
    // Flush TX data - Programmed and then Ready lists
    lld_pdu_tx_flush_list(&evt->tx_prog);
    lld_pdu_tx_flush_list(&evt->tx_acl_rdy);
    #endif //(BLE_CENTRAL || BLE_PERIPHERAL)
}


uint8_t lld_pdu_adv_pack(uint8_t code, uint8_t* buf, uint8_t* p_len)
{
    uint8_t status = LLC_PDU_PACK_UNKNOWN;
    const struct lld_pdu_pack_desc* desc = NULL;

    // Find LLCP descriptor
    ASSERT_INFO((code < LL_ADV_END), code, code);

    desc = &lld_pdu_adv_pk_desc_tab[code];

    // Check if LLCP desc found
    if(desc != NULL)
    {
        if(desc->pack_fmt != NULL)
        {
            // Pack the parameters
            status = lld_pdu_pack(buf, p_len, desc->pack_fmt);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}


#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles the LLCP PDU
 *
 * @param info   PDU reception Info
 * @param length PDU length
 ****************************************************************************************
 */
static void lld_pdu_llcp_rx_handler(struct lld_pdu_rx_info* info, uint8_t length)
{
    #if (BLE_TESTER)
    // check if pass through mechanism is enabled or not
    if(llc_env[info->conhdl]->llcp_pass_through_enable)
    {
        uint16_t rx_data_ptr = ble_rxdataptr_get(info->rx_hdl);
        // Generate a DBG Meta Event which contains the LLCP PDU

        // allocate the DBG Meta event message
        struct hci_dbg_ble_tst_llcp_recv_evt *event = KERNEL_MSG_ALLOC(HCI_DBG_EVT, info->conhdl, 0, hci_dbg_ble_tst_llcp_recv_evt);
        // gets event subcode
        event->subcode = HCI_DBG_BLE_TST_LLCP_RECV_EVT_SUBCODE;
        // gets connection handle
        event->conhdl = info->conhdl;

        event->length = length;
        // load pdu data
        em_rd(event->data, rx_data_ptr, length);

        // send it to the host
        hci_send_2_host(event);
    }
    else
    #endif // (BLE_TESTER)
    {
        kernel_task_id_t llc_taskid      = KERNEL_BUILD_ID(TASK_LLC, info->conhdl);

        // Allocate message pdu handler
        struct llc_llcp_recv_ind *msg = KERNEL_MSG_ALLOC(LLC_LLCP_RECV_IND, llc_taskid, llc_taskid, llc_llcp_recv_ind);

        // unpack the message
        lld_pdu_llcp_unpack(info->rx_hdl, length, msg);

        // check if message can be handled or not
        llc_llcp_recv_handler(llc_taskid, msg->status, &(msg->pdu), true);

        // And send the message
        kernel_msg_send(msg);
    }
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)


void lld_pdu_rx_handler(struct lld_evt_tag *evt, uint8_t nb_rx_desc)
{
    uint8_t rx_hdl = em_buf_rx_current_get();
	
   // printf("hearder = 0x%x,rx status = 0x%x,nb_rx_desc = %d\r\n",ble_rxphce_get(rx_hdl),ble_rxstat_get(rx_hdl),nb_rx_desc);

    while(nb_rx_desc--)
    {
        if(!LLD_EVT_FLAG_GET(evt, WAITING_EOEVT_TO_DELETE))
        {
            struct lld_pdu_rx_info* info = kernel_malloc(sizeof(struct lld_pdu_rx_info), KERNEL_MEM_KERNEL_MSG);
            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            uint16_t header = ble_rxphce_get(rx_hdl);
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)

            info->rx_hdl = rx_hdl;
            info->conhdl = evt->conhdl;
            info->status = ble_rxstat_get(rx_hdl);
            #if(HW_AUDIO)
            ble_rxchass_unpack(rx_hdl, &info->audio, &info->channel, &info->rssi);
            #else // !(HW_AUDIO)
            info->audio = 0;
          //  ble_rxchass_unpack(rx_hdl, &info->channel, &info->rssi);
					
					  ble_rxchass_unpack(rx_hdl, &info->audio, &info->channel, &info->rssi);
            #endif // (HW_AUDIO)
            info->free   = false;
       //     ASSERT_INFO(ble_rxlinklbl_getf(rx_hdl) == evt->conhdl, ble_rxlinklbl_getf(rx_hdl), evt->conhdl);

            #if (BLE_PERIPHERAL)
            // If we are waiting for the acknowledgment, and it is received, enable the slave latency
            if (LLD_EVT_FLAG_GET(evt, WAITING_ACK) && !(info->status & BLE_NESN_ERR_BIT))
            {
                // We received the acknowledgment
                LLD_EVT_FLAG_RESET(evt, WAITING_ACK);
            }
            #endif //(BLE_PERIPHERAL)

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            // check if it's a connection handle
            if(info->conhdl != LLD_ADV_HDL)
            {
                // retrieve PDU pdu_len
                info->length = GETF(header, BLE_RXLEN);

                if(LLD_EVT_FLAG_GET(evt, WAITING_SYNC) && ((info->status & BLE_SYNC_ERR_BIT) == 0))
                {
                    LLD_EVT_FLAG_RESET(evt, WAITING_SYNC);

                    // mark that first sync has been received
                    SETF(llc_env[info->conhdl]->llc_status, LLC_STAT_SYNC_FOUND, true);
                }

                // check if pdu packet should be ignored
                if (((info->status & (BLE_MIC_ERR_BIT | BLE_CRC_ERR_BIT | BLE_LEN_ERR_BIT | BLE_TYPE_ERR_BIT | BLE_SYNC_ERR_BIT | BLE_SN_ERR_BIT | BLE_RXTIMEERR_BIT)) == 0)
                    && (info->length > 0))
                {
                    // retrieve pdu LLID
                    uint8_t llid = GETF(header, BLE_RXLLID);

                    // If RX encryption is enabled, remove the MIC length from the PCU length
                    if (LLC_UTIL_ENC_STATE_IS_SET(info->conhdl, LLC_ENC_RX)
                            #if(BLE_AUDIO)
                            && (ble_crypt_mode_getf(info->conhdl) == AUDIO_ENC_MODE_0)
                            #endif // (BLE_AUDIO)
                            )
                    {
                        ASSERT_INFO(info->length > MIC_LEN, info->length, header);
                        info->length -= MIC_LEN;
                    }

                    switch(llid)
                    {
                        case LLID_CNTL:
                        {
                            // LLCP packet
                            lld_pdu_llcp_rx_handler(info, info->length);

                            // Free the received descriptor
                            em_buf_rx_free(info->rx_hdl);
                            info->free = true;
                        } break;
                        case LLID_START:
                        case LLID_CONTINUE:
                        {
                            // ACL packet
                            // builds the message id associated to the conhdl
                            kernel_task_id_t llc_taskid      = KERNEL_BUILD_ID(TASK_LLC, info->conhdl);
                            // allocates the message to send - The message is first sent to LLC task for further
                            // security checks
                            struct llc_data_ind *ll_data = KERNEL_MSG_ALLOC(LLC_DATA_IND, llc_taskid, llc_taskid, llc_data_ind);
                            // gets the conhdl associated to the task
                            ll_data->conhdl              = info->conhdl;
                            // gets the length
                            ll_data->length              = info->length;
                            // sets the packet boundary flag
                            ll_data->pb_bc_flag          = llid;
                            // gets the pointer on the descriptor
                            ll_data->rx_hdl              = info->rx_hdl;
                            // send the message
                            kernel_msg_send(ll_data);
                        } break;
                        default:
                        {
                            // Free the received descriptor
                            em_buf_rx_free(info->rx_hdl);
                            info->free = true;
                        } break;
                    }
                }
                else
                {
                    // Free the received descriptor
                    em_buf_rx_free(info->rx_hdl);
                    info->free = true;
                }
            }
            else
            #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
            {
                // handle in in the defer
            }

            // Push the element at the end of the deferred list
            common_list_push_back(&lld_evt_env.rx_pkt_deferred, &info->hdr);
        }
        else
        {
            // Free the received descriptor
            em_buf_rx_free(rx_hdl);
        }
        // Go to the next descriptor
        rx_hdl = em_buf_rx_next(rx_hdl);
    }

    // Move the current RX buffer
    em_buf_rx_current_set(rx_hdl);
}

/// @} LLDPDU
