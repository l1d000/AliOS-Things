/**
****************************************************************************************
*
* @file lld_wlcoex.c
*
* @brief WLAN/MWS Coexistence mailbox functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup LLDWLCOEX
* @{
****************************************************************************************
*/

/*
 * INCLUDES
 ****************************************************************************************
 */
#include <stdbool.h>            // boolean definitions
#include <stdint.h>             // boolean definitions

#include "rwble_config.h"      // stack configuration
#include "lld_wlcoex.h"         // WL coexistence definitions
#include "reg_blecore.h"        // ble core registers

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */
#if (RW_BLE_WLAN_COEX)
void lld_wlcoex_set(bool en)
{
    if (en)
    {
        ble_coexifcntl0_set(ble_coexifcntl0_get() | BLE_SYNCGEN_EN_BIT | BLE_WLANCOEX_EN_BIT);
    }
    else
    {
        ble_coexifcntl0_set(ble_coexifcntl0_get() & ~(BLE_SYNCGEN_EN_BIT | BLE_WLANCOEX_EN_BIT));
    }
}
#endif // RW_BLE_WLAN_COEX

#if (RW_BLE_MWS_COEX)
void lld_mwscoex_set(bool en)
{
    if (en)
    {
        ble_coexifcntl0_set(ble_coexifcntl0_get() | BLE_SYNCGEN_EN_BIT /*| BLE_MWSWCI_EN_BIT*/ | BLE_MWSCOEX_EN_BIT);
    }
    else
    {
        ble_coexifcntl0_set(ble_coexifcntl0_get() & ~(BLE_SYNCGEN_EN_BIT | BLE_MWSWCI_EN_BIT | BLE_MWSCOEX_EN_BIT));
    }
}
#endif // RW_BLE_MWS_COEX

///@} LDWLCOEX
