/**
 ****************************************************************************************
 *
 * @file rwble.c
 *
 * @brief Entry points the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2015
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
#include "rwble.h"

#include "common_version.h"
#if HCI_PRESENT
#include "hci.h"
#endif //HCI_PRESENT
#include "kernel_event.h"
#include "kernel_timer.h"
#include "em_buf.h"
#include "ea.h"
#include "lld.h"
#include "llcontrl.h"
#include "llm.h"
#include "dbg.h"
#include "ea.h"
#include "reg_blecore.h"

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT
#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
#include "audio.h"
#if defined (PLF_LE_AUDIO_PATH)
#include "leaudio.h"
#endif
#endif /*((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))*/
#include "ble_pub.h"
#include "arm_arch.h"
#include "ps_debug_pub.h"


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the diagnostic port.
 ****************************************************************************************
 */
static void rwble_diagport_init(void)
{
    #if (NVDS_SUPPORT)
    uint8_t diag_cfg[NVDS_LEN_DIAG_BLE_HW];
    uint8_t length = NVDS_LEN_DIAG_BLE_HW;

    // Read diagport configuration from NVDS
    if(nvds_get(NVDS_TAG_DIAG_BLE_HW, &length, diag_cfg) == NVDS_OK)
    {
        ble_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
    }
    else
    #endif // NVDS_SUPPORT
    {
        ble_diagcntl_set(0);
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void rwble_init(void)
{
    // Initialize buffers
    em_buf_init();
    // Initialize the Link Layer Driver
    lld_init(false);
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize the Link Layer Controller
    llc_init();
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */
    // Initialize the Link Layer Manager
    llm_init(false);
    // Initialize diagport - for test and debug only
    rwble_diagport_init();

    #if BLE_HOST_PRESENT
    // Signal the completion of the boot process to the host layers
    llm_ble_ready();
    #endif //BLE_HOST_PRESENT
    #if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
    audio_init(false);
    #endif /*#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))*/
}

void rwble_reset(void)
{
    uint32_t seed;

    // Disable interrupts before reset procedure is completed
    GLOBAL_INT_DIS();
    //Sample the base time count
    ble_samp_setf(1);
    while (ble_samp_getf());
    seed = ble_basetimecnt_get();
    seed += ble_finetimecnt_get();
    seed += ble_bdaddrl_get();
    //Init the random seed
    common_random_init(seed);

    // Reset the BLE core
    lld_core_reset();
    
    // init the link layer driver
    lld_init(true);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Reset the link layer controller
    llc_reset();
    #endif /* #if (BLE_CENTRAL || BLE_PERIPHERAL) */

    // init the link layer manager
    llm_init(true);

    // Initialize Descriptors and buffers
    em_buf_init();
    
    #if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
    audio_init(true);
    #endif /*#if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))*/

    // Restore interrupts after reset procedure is completed
    GLOBAL_INT_RES();
}

#if (DEEP_SLEEP)
bool rwble_sleep_check(void)
{
    return(common_list_is_empty(&lld_evt_env.elt_prog));
}
#endif

bool rwble_activity_ongoing_check(void)
{
    bool status = true;
    if(common_list_is_empty(&lld_evt_env.elt_prog) && common_list_is_empty(&lld_evt_env.elt_wait)  && common_list_is_empty(&lld_evt_env.elt_deferred))
    {
        status = false;
    }
    return(status );
}


void rwble_version(uint8_t* fw_version, uint8_t* hw_version)
{
    // FW version
    *(fw_version+3) = RWBT_SW_VERSION_MAJOR;
    *(fw_version+2) = RWBT_SW_VERSION_MINOR;
    *(fw_version+1) = RWBT_SW_VERSION_BUILD;
    *(fw_version)   = RWBT_SW_VERSION_SUB_BUILD;

    // HW version 
    *(hw_version+3) = ble_typ_getf();
    *(hw_version+2) = ble_rel_getf();
    *(hw_version+1) = ble_upg_getf();
    *(hw_version)   = ble_build_getf();
}

void rwble_isr(void)
{
	//printf("rwble_isr \r\n");
    // Loop until no more interrupts have to be handled
    while (1)
    {
        // Check BLE interrupt status and call the appropriate handlers
        uint32_t irq_stat = ble_intstat_get();
			
        if (irq_stat == 0)
            break;
        
        #if DEEP_SLEEP
        #if !BT_DUAL_MODE
        // End of sleep interrupt
        if (irq_stat & BLE_SLPINTSTAT_BIT)
        {

            #if CFG_USE_BLE_PS
            if(ble_ps_enabled())
            {
                sctrl_rf_wakeup();
                ble_switch_rf_to_ble();
            }
            #endif

            DBG_SWDIAG(BLE_ISR, SLPINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_SLPINTACK_BIT);
			// Handle wake-up
            rwip_wakeup();	
            {
                rwip_prevent_sleep_clear(RW_SLEEP_EVENT_ONGOING);
            }
            DBG_SWDIAG(BLE_ISR, SLPINT, 0);
        }

        // Slot interrupt
        if (irq_stat & BLE_CSCNTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, CSCNTINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_CSCNTINTACK_BIT);

            // Handle end of wake-up
            rwip_wakeup_end();

            GLOBAL_INT_DIS();
            ble_deep_sleep = 0;
            GLOBAL_INT_RES();

            // Try to schedule immediately
            ea_finetimer_isr();

            DBG_SWDIAG(BLE_ISR, CSCNTINT, 0);
        }
        #endif //!BT_DUAL_MODE
        #endif //DEEP_SLEEP

        // Event target interrupt
        if (irq_stat & BLE_FINETGTIMINTSTAT_BIT)
        {
            //PS_DEBUG_UP_OUT;
            ble_request_rf_by_isr();            
            DBG_SWDIAG(BLE_ISR, FINETGTIMINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_FINETGTIMINTACK_BIT);

            ea_finetimer_isr();

            DBG_SWDIAG(BLE_ISR, FINETGTIMINT, 0);
            //ble_send_msg(BLE_MSG_POLL);
        }
        #if ((BLE_CENTRAL || BLE_PERIPHERAL) && (BLE_AUDIO))
        // Audio channel 0 interrupt
        if (irq_stat & BLE_AUDIOINT0STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO0INT, 1);
            ble_intack_clear(BLE_AUDIOINT0STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_0);
            DBG_SWDIAG(BLE_ISR2, AUDIO0INT, 0);
        }
        // Audio channel 1 interrupt
        if (irq_stat & BLE_AUDIOINT1STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO1INT, 1);
            ble_intack_clear(BLE_AUDIOINT1STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_1);
            DBG_SWDIAG(BLE_ISR2, AUDIO1INT, 0);
        }
        // Audio channel 2 interrupt
        if (irq_stat & BLE_AUDIOINT2STAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, AUDIO2INT, 1);
            ble_intack_clear(BLE_AUDIOINT2STAT_BIT);
            audio_evt_processed(AUDIO_CHANNEL_2);
            DBG_SWDIAG(BLE_ISR2, AUDIO2INT, 0);
        }
        #endif
        // End of event interrupt
        if (irq_stat & BLE_EVENTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, EVENTINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_EVENTINTSTAT_BIT);

            if (irq_stat & BLE_RXINTSTAT_BIT)
            {
                ble_intack_clear(BLE_RXINTSTAT_BIT);
                irq_stat &= ~BLE_RXINTSTAT_BIT;
            }
            lld_evt_end_isr(false);
            
            DBG_SWDIAG(BLE_ISR, EVENTINT, 0);

            //PS_DEBUG_UP_TRIGER;
            ble_release_rf_by_isr();
        }

        // AFPM interrupt
        if (irq_stat & BLE_EVENTAPFAINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, EVENTAPFMINT, 1);
            // Clear the interrupt
            ble_intack_clear(BLE_EVENTAPFAINTSTAT_BIT);
            if (irq_stat & BLE_RXINTSTAT_BIT)
            {
                ble_intack_clear(BLE_RXINTSTAT_BIT);
                irq_stat &= ~BLE_RXINTSTAT_BIT;
            }

            lld_evt_end_isr(true);
					
            DBG_SWDIAG(BLE_ISR2, EVENTAPFMINT, 0);
        }

        // Rx interrupt
        if (irq_stat & BLE_RXINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, RXINT, 1);
            ble_intack_clear(BLE_RXINTSTAT_BIT);	
            lld_evt_rx_isr();
            DBG_SWDIAG(BLE_ISR, RXINT, 0);
        }

        // SW interrupt
        if (irq_stat & BLE_SWINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR2, SWINT, 1);
            // Clear the interrupt
            ble_intack_clear(BLE_SWINTSTAT_BIT);

            ea_sw_isr();
            DBG_SWDIAG(BLE_ISR2, SWINT, 0);
        }

        #if (!BT_DUAL_MODE)
        // General purpose timer interrupt
        if (irq_stat & BLE_GROSSTGTIMINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, GROSSTGTIMINT, 1);

            // Clear the interrupt
            ble_intack_clear(BLE_GROSSTGTIMINTACK_BIT);

            lld_evt_timer_isr();

            DBG_SWDIAG(BLE_ISR, GROSSTGTIMINT, 0);
        }
        #endif //(!BT_DUAL_MODE)

        // End of encryption interrupt
        if (irq_stat & BLE_CRYPTINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, CRYPTINT, 1);

            ble_intack_clear(BLE_CRYPTINTSTAT_BIT);

            lld_crypt_isr();

            DBG_SWDIAG(BLE_ISR, CRYPTINT, 0);
        }

        // Error interrupt
        if (irq_stat & BLE_ERRORINTSTAT_BIT)
        {
            DBG_SWDIAG(BLE_ISR, ERRORINT, 1);

            ASSERT_INFO(0, ble_errortypestat_get(), 0);

            // Clear the interrupt
            ble_intack_clear(BLE_ERRORINTSTAT_BIT);
            DBG_SWDIAG(BLE_ISR, ERRORINT, 0);
        }

        ble_send_msg(BLE_MSG_POLL);
    }
}

/// @} RWBLE
