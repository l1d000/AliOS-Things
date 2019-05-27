/**
 ****************************************************************************************
 *
 * @file rwble_hl.c
 *
 * @brief Entry points the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>
#include "rwip_config.h"
#include "rwip.h"
#include "rwble_hl.h"

#include "gapm.h"
#include "gattm.h"
#if (BLE_PERIPHERAL || BLE_CENTRAL)
#include "attm.h"
#include "l2cm.h"
#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)


#if (BLE_PROFILES)
#include "prf.h"
#endif /* (BLE_PROFILES) */

#if(!BLE_EMB_PRESENT)
#include "em_buf.h"
#endif //(!BLE_EMB_PRESENT)

#ifdef BLE_AUDIO_AM0_TASK
#include "am0_api.h"
#endif // BLE_AUDIO_AM0_TASK
/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
/*   rwble_hl_reset 函数会被gapm_task.c中函数调用，修改其为指针传递
			gapm_call_flash_list.gapm_rwble_hl_reset();
*/


void rwble_hl_init(void)
{
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (BLE_ATTS)
    /* Initialize database */
    attm_init(false);
    #endif /* (BLE_ATTS) */
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    // Initialize GAP
    gapm_init(false);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize GATT
    gattm_init(false);
    // Initialize L2CAP
    l2cm_init(false);
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    #ifdef BLE_AUDIO_AM0_TASK
    // Initialize Audio Mode 0
    am0_task_init(false);
    #endif // BLE_AUDIO_AM0_TASK
}


/**
 ****************************************************************************************
 * @brief Initialize the host (reset requested)
 *
 ****************************************************************************************
 */
void rwble_hl_reset(void)
{
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    #if (BLE_ATTS)
    /* Initialize database */
    attm_init(true);
    #endif /* (BLE_ATTS) */
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    // initialize GAP
    gapm_init(true);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize GATT & ATT
    gattm_init(true);
    // Initialize L2CAP
    l2cm_init(true);
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    #ifdef BLE_AUDIO_AM0_TASK
    // Initialize Audio Mode 0
    am0_task_init(true);
    #endif // BLE_AUDIO_AM0_TASK
}

/// @} RWBLE
