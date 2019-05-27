/**
 ****************************************************************************************
 *
 * @file llc_ch_asses.c
 *
 * @brief Link layer channel assessment functions declaration
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCCHASSES
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>
#include "common_endian.h"
#include "common_utils.h"
#include "rwip_config.h"
#if (BLE_CHNL_ASSESS)
#include "llm_util.h"
#include "llcontrl.h"
#include "llc_ch_asses.h"
#include "reg_ble_em_rx_desc.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * PRIVATE FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void llc_ch_assess_local(uint16_t conhdl, uint16_t status, int8_t rssi, uint8_t channel)
{
    //Gets environment link to the conhdl
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];

    /************** START ASSESS ALGO ***************/
    /*
     * The algorithm will use a continuous standard deviation computation
     */
    // If we detect a sync error several case are possible
    if(status & (uint16_t)BLE_SYNC_ERR_BIT)
    {
        // Case where we have a noiser close to the device
        if(rssi > llm_util_ch_assess_get_rssi_noise())
        {
            llc_env_ptr->chnl_assess.rcvd_quality[channel] += LLD_CH_ASSES_SYNC_ERR_HIGH_RSSI;
        }
        else
        {
            // Case where the latency is used
            if(!llc_env_ptr->chnl_assess.latency_en)
            {
                llc_env_ptr->chnl_assess.rcvd_quality[channel] += LLD_CH_ASSES_SYNC_ERR_LOW_RSSI_NO_LATENCY;
            }
            // If synchronization not detected and no RSSI is low do nothing
        }
    }
    else if(status & (uint16_t)BLE_CRC_ERR_BIT)
    {
        // CRC error detected
        llc_env_ptr->chnl_assess.rcvd_quality[channel] += LLD_CH_ASSES_CRC_ERR;
    }
    else
    {
        // Synchronization detected without error
        llc_env_ptr->chnl_assess.rcvd_quality[channel] += LLD_CH_ASSES_SYNC_FOUND_NO_CRC_ERR;
    }
    /*
     * The lower and upper limits are set to reduce the latency in case of the channel quality change
     */
    if(llc_env_ptr->chnl_assess.rcvd_quality[channel] < llm_util_ch_assess_get_lower_limit())
    {
        llc_env_ptr->chnl_assess.rcvd_quality[channel] = llm_util_ch_assess_get_lower_limit();
    }
    else if(llc_env_ptr->chnl_assess.rcvd_quality[channel] > llm_util_ch_assess_get_upper_limit())
    {
        llc_env_ptr->chnl_assess.rcvd_quality[channel] = llm_util_ch_assess_get_upper_limit();
    }
}

uint8_t llc_ch_assess_get_local_ch_map(uint16_t conhdl, struct le_chnl_map *map, struct le_chnl_map *hostmap)
{
    uint8_t nb_ch_good = 0;
    // Gets environment link to the conhdl
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    // Set all channels as BAD
    memset(map, 0x00, LE_CHNL_MAP_LEN);

    for(int8_t channel = (LE_DATA_FREQ_LEN - 1); channel >= 0; channel--)
    {
        // check that host mapping is valid
        if(((hostmap->map[channel >> 3] & (uint8_t)((1 << (channel & 0x7)))) != 0)
            // and channel quality is good
            && (llc_env_ptr->chnl_assess.rcvd_quality[channel] > llm_util_ch_assess_get_lower_limit()))
        {
            // mark channel as good
            map->map[channel >> 3] |= (uint8_t)((1 << (channel & 0x7)));
            nb_ch_good++;
        }
    }

    return nb_ch_good;
}

struct le_chnl_map * llc_ch_assess_get_current_ch_map(uint16_t conhdl)
{
    return (&llc_env[conhdl]->ch_map);
}



void llc_ch_assess_reass_ch(uint16_t conhdl, struct le_chnl_map *map, struct le_chnl_map *hostmap, uint8_t nb_chgood)
{
    // Gets environment link to the conhdl
    struct llc_env_tag *llc_env_ptr = llc_env[conhdl];
    uint16_t nb_chan_reasses;
    int8_t i;
    uint8_t cursor = llc_env_ptr->chnl_assess.reassess_cursor;

    // ensure that we have at least 2 channels
    if(nb_chgood <= 2)
    {
        // increase drastically number of channel to increase chance of finding new good channels
        nb_chan_reasses = 10;
    }
    else
    {
        // only re-assess 25% more channels: ((N * 2^5) + 2^7 - 1) / 2^7 <==> ceil(N * 1.25)
        nb_chan_reasses = ((((uint16_t)nb_chgood) << 5) + 127) >> 7;
    }

    // reset reassess counter
    llc_env_ptr->chnl_assess.reassess_count = llm_util_ch_assess_get_reass_cnt();

    // check all frequencies
    for(i = (LE_DATA_FREQ_LEN - 1); (i >= 0) && (nb_chan_reasses > 0); i--)
    {
        // update cursor, to increase diversity, we try to enable channel into different frequency band
        cursor += 11;
        if(cursor >= LE_DATA_FREQ_LEN)
        {
            cursor -= LE_DATA_FREQ_LEN;
        }

        // check that channel currently disabled
        if(((map->map[cursor >> 3] & (uint8_t)((1 << (cursor & 0x7)))) == 0)
                // but allowed by host map
                && ((hostmap->map[cursor >> 3] & (uint8_t)((1 << (cursor & 0x7)))) != 0))
        {
            // re-assess channel
            llc_env_ptr->chnl_assess.rcvd_quality[cursor] = 0;

            // mark channel as good
            map->map[cursor >> 3] |= (uint8_t)((1 << (cursor & 0x7)));
            nb_chan_reasses--;
        }
    }

    // fill cursor with new value
    llc_env_ptr->chnl_assess.reassess_cursor = cursor;
}

#endif //#if (BLE_CHNL_ASSESS)

/// @} LLCCHASSES
