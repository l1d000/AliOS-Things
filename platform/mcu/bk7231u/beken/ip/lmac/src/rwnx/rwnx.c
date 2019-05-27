/**
 ****************************************************************************************
 *
 * @file rwnx.c
 *
 * @brief This file contains the implementation of the nX-MAC platform APIs.
 *
 * Copyright (C) RivieraWaves 2011-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup MACSW
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "version.h"
#include "ke_event.h"
#include "mm.h"
#include "hal_machw.h"
#include "rxl_cntrl.h"
#include "txl_cntrl.h"
#include "ps.h"
#include "rwnx.h"

#if (NX_TX_FRAME)
#include "vif_mgmt.h"
#endif //(NX_TX_FRAME)

#include "me.h"
#include "include.h"
#include "rw_pub.h"
#include "arm_arch.h"
#include "mem_pub.h"
#include "ke_timer.h"
#include "mm_timer.h"
#include "target_util_pub.h"
#include "power_save_pub.h"
#include "rw_platf_pub.h"

/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */
#if NX_POWERSAVE
struct rwnx_env_tag rwnx_env;
#endif

RW_CONNECTOR_T g_rwnx_connector = {NULLPTR};
uint32_t *rwnx_bak_reg_ptr;
uint32_t rwnx_reseting = false;

extern int g_set_channel_postpone_num;
extern void ps_env_flush(void);

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function performs all the initializations of the MAC SW.
 *
 * It first initializes the heap, then the message queues and the events. Then if required
 * it initializes the trace.
 *
 ****************************************************************************************
 */
void rwnxl_init(void)
{
    #if NX_POWERSAVE
    rwnx_env.hw_in_doze = false;
    #endif
    
    rw_ieee80211_init();    
    me_init();// UMAC initialization;
    mm_init();// LMAC initialization;
    ke_init();// Mac kernel initialization

    rwm_msdu_init();
}

void rwnxl_register_connector(RW_CONNECTOR_T *intf)
{
    g_rwnx_connector = *intf;
}

void rwnxl_backup_reg(void)
{
	uint32_t reg, val, i, j;
	uint32_t *bak = rwnx_bak_reg_ptr;

	for(i = 0; i <= (0x154 >> 2) + 1; i ++)
	{
		reg = 0xc0000000 + (i << 2);
		val = REG_READ(reg);
		bak[i] = val;
	}

    #if (CFG_SOC_NAME != SOC_BK7231)
    bak[i ++] = REG_READ(0xc0000700);
    #endif

	bak[i ++] = REG_READ(0xc0008048);
	
	bak[i ++] = REG_READ(0xc000806c);
	bak[i ++] = REG_READ(0xc0008070);
	bak[i ++] = REG_READ(0xc0008074);
	bak[i ++] = REG_READ(0xc0008078);
	bak[i ++] = REG_READ(0xc000807c);
	bak[i ++] = REG_READ(0xc0008080);
	bak[i ++] = REG_READ(0xc0008084);
	bak[i ++] = REG_READ(0xc0008088);
	bak[i ++] = REG_READ(0xc000808c);
	
	bak[i ++] = REG_READ(0xc00080a4);
	bak[i ++] = REG_READ(0xc00080a8);
	bak[i ++] = REG_READ(0xc0008160);
	bak[i ++] = REG_READ(0xc0008164);
	bak[i ++] = REG_READ(0xc0008168);

    ASSERT(i <= RW_BAK_REG_LEN);
}


void rwnxl_restore_reg(void)
{
	uint32_t reg, val, i;
	uint32_t *bak = rwnx_bak_reg_ptr;

	for(i = 0; i <= (0x154 >> 2) + 1; i ++)
	{
		val = bak[i];
		reg = 0xc0000000 + (i << 2);
		REG_WRITE(reg, val);
	}

    #if (CFG_SOC_NAME != SOC_BK7231)
    REG_WRITE(0xc0000700, bak[i ++]);
    #endif

	REG_WRITE(0xc0008048, bak[i ++]);	
	
	REG_WRITE(0xc000806c, bak[i ++]);	
	REG_WRITE(0xc0008070, bak[i ++]);	
	REG_WRITE(0xc0008074, bak[i ++]);	
	REG_WRITE(0xc0008078, bak[i ++]);	
	REG_WRITE(0xc000807c, bak[i ++]);	
	REG_WRITE(0xc0008080, bak[i ++]);	
	REG_WRITE(0xc0008084, bak[i ++]);	
	REG_WRITE(0xc0008088, bak[i ++]);	
	REG_WRITE(0xc000808c, bak[i ++]);
	
	REG_WRITE(0xc00080a4, bak[i ++]);	
	REG_WRITE(0xc00080a8, bak[i ++]);
	
	REG_WRITE(0xc0008160, bak[i ++]);	
	REG_WRITE(0xc0008164, bak[i ++]);	
	REG_WRITE(0xc0008168, bak[i ++]);	

    ASSERT(i <= RW_BAK_REG_LEN);

}

void rwnxl_violence_reset_patch(void)
{
    rwnx_bak_reg_ptr = (uint32_t*)os_malloc(RW_BAK_REG_LEN * sizeof(uint32_t));
    if(!rwnx_bak_reg_ptr) {
        os_printf("bakup mac reg failed with no memery!!\r\n");
        return;
    }
          
	rwnxl_backup_reg();
	
	hal_machw_stop();

	rwnxl_restore_reg();

    os_free(rwnx_bak_reg_ptr);
	rwnx_bak_reg_ptr = 0;
}

void rwnxl_set_reseting_flag(uint32_t flag)
{
    rwnx_reseting = flag;
}

uint32_t rwnxl_get_reseting_flag(void)
{
    return rwnx_reseting;
}

/**
 ****************************************************************************************
 * @brief This function performs all the initializations of the MAC SW.
 *
 * It first initializes the heap, then the message queues and the events. Then if required
 * it initializes the trace.
 *
 ****************************************************************************************
 */
void mm_sta_keep_alive_time_reset(void);
extern void mm_bcn_flush(void);
void rwnxl_reset_handle(int dummy)
{    
    GLOBAL_INT_DECLARATION();

    // Do the full reset procedure with interrupt disabled
    GLOBAL_INT_DISABLE();
    rwnxl_set_reseting_flag(true);
    // Clear the reset event
    ke_evt_clear(KE_EVT_RESET_BIT);

    mm_timer_save_before_soft_reset();
    ke_timer_save_before_soft_reset();

    // Reset the MAC HW (this will reset the PHY too)
    hal_machw_reset();

	rwxl_reset_patch();

    // Reset the RX path
    rxl_reset();

    // Reset the TX path
    txl_reset();

    // Reset the MM
    mm_reset();

    #if (NX_TX_FRAME)
    // Push postponned internal frames
    vif_mgmt_reset();
    #endif //(NX_TX_FRAME)

    mm_bcn_flush();
    mm_sta_keep_alive_time_reset();

    #if CFG_USE_STA_PS
    ps_env_flush();
    #endif

    if(g_set_channel_postpone_num)
    {
        rw_msg_set_channel(g_set_channel_postpone_num, PHY_CHNL_BW_20, NULL);

        g_set_channel_postpone_num = 0;
    }

    mm_timer_restore_after_soft_reset();
    ke_timer_restore_after_soft_reset();

    rwnxl_set_reseting_flag(false);
    GLOBAL_INT_RESTORE();
}

void rwnxl_reset_evt(int dummy)
{
#if NX_POWERSAVE
    CHECK_OPERATE_RF_REG_IF_IN_SLEEP();
#endif

    rwnxl_reset_handle(dummy);

#if NX_POWERSAVE
    CHECK_OPERATE_RF_REG_IF_IN_SLEEP_END();
#endif
}

#if NX_POWERSAVE
static UINT32 nxmac_timer_saved;

void rwnxl_set_nxmac_timer_value(void)
{
    nxmac_timers_int_un_mask_set(nxmac_timer_saved);
}

bool rwnxl_get_status_in_doze(void)
{
    return rwnx_env.hw_in_doze;
}

void rwnxl_ask_idle(void)
{
    uint32_t v_tmp = 0;
    uint32_t i_tmp = 0;
    
    // Store the current HW state to recover it at wake-up
    rwnx_env.prev_hw_state = nxmac_current_state_getf();

    // Ask HW to go to IDLE
    if (nxmac_current_state_getf() != HW_IDLE)
    {
        nxmac_next_state_setf(HW_IDLE);
        while(1)
        {
            if(nxmac_status_idle_interrupt_getf() == 1)
                break;
            else
            {
                i_tmp++;
                if(i_tmp > 1000)
                {
                    i_tmp = 0;
                    v_tmp =nxmac_state_cntrl_get();
                    PS_WPRT(" s:0x%x\r\n",v_tmp);
                }
            }
        }
        nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);
    }
}

bool rwnxl_if_idle(void)
{
    return  (nxmac_current_state_getf() == HW_IDLE);
}

#if CFG_USE_STA_PS
#if PS_WAKEUP_MOTHOD_RW
/**
 ****************************************************************************************
 * @brief This function performs the required checks prior to go to DOZE mode
 *
 * @return true if the CPU can be put in sleep, false otherwise.
 *
 ****************************************************************************************
 */
bool rwnxl_sleep(IDLE_FUNC wait_func,IDLE_FUNC do_func)
{
    bool cpu_sleep = false;
    uint32_t v_tmp = 0;
    uint32_t i_tmp = 0,y_tmp = 0;

    if(rwnx_env.hw_in_doze)
    {
        power_save_forbid_trace(PS_FORBID_IN_DOZE);
        goto sleep_exit;
    }

    do
    {
        // Check if some kernel processing is ongoing
        if (ke_evt_get() != 0)
        {
            power_save_forbid_trace(PS_FORBID_KEEVT_ON);
            break;
        }
        
        if(!bmsg_is_empty())
        {
            power_save_forbid_trace(PS_FORBID_BMSG_ON);
            break;
        }

        // Check if TX path allows sleeping
        if (!txl_sleep_check())
        {
            power_save_forbid_trace(PS_FORBID_TXING);
            break;
        }

        if(power_save_if_sleep_first())
        {
            nxmac_timer_saved = nxmac_timers_int_un_mask_get();
        }
#if 1
        if(nxmac_timer_saved != nxmac_timers_int_un_mask_get())
        {
            PS_PRT("---ps mxmac tim:%x saved:%x\r\n",nxmac_timers_int_un_mask_get(),
                nxmac_timer_saved);
            nxmac_timer_saved = nxmac_timers_int_un_mask_get();
        }     
#endif
        nxmac_timers_int_un_mask_set(nxmac_timer_saved & ~(0x3ff));

        // Check the HW timers
        if (!hal_machw_sleep_check())
        {
            power_save_forbid_trace(PS_FORBID_HW_TIMER);
#if 1
            nxmac_timers_int_un_mask_set(nxmac_timer_saved);
#endif
            break;
        }
        
#if 1

        nxmac_timers_int_event_clear(0x3ff);
#endif
      
        cpu_sleep = true;

        // Store the current HW state to recover it at wake-up
        rwnx_env.prev_hw_state = nxmac_current_state_getf();

        // Ask HW to go to IDLE
        if (nxmac_current_state_getf() != HW_IDLE)
        {
            nxmac_next_state_setf(HW_IDLE);
            while(1) 
            {
                if(nxmac_status_idle_interrupt_getf() == 1)
                    break;
                else
                {
                    wait_func();
                    i_tmp++;
                    if(i_tmp > 1000)
                    {
                        i_tmp = 0;
                        v_tmp =nxmac_state_cntrl_get();
                        PS_WPRT("s s:0x%x c:0x%x\r\n",v_tmp,power_save_get_sleep_count());
                        y_tmp ++;
                        if(y_tmp > 50)
                        {
                            y_tmp = 0;
                            PS_WPRT("idle rec\r\n");
                            //long time wait idle interrupt,recover!
                            hal_machw_disable_int();
                            rwnxl_reset_handle(0);
                            PS_WPRT("idle rec over\r\n");
                        }
                        nxmac_next_state_setf(HW_IDLE);
                    }
                }
            }
            //while (nxmac_status_idle_interrupt_getf() != 1);
            nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);
        }

        do_func();
		
        // Ask HW to go to DOZE
        rwnx_env.hw_in_doze = true;
        nxmac_next_state_setf(HW_DOZE);
    } while(0);

sleep_exit:
    return (cpu_sleep);
}


/**
 ****************************************************************************************
 * @brief This function performs the wake up from DOZE mode.
 *
 ****************************************************************************************
 */
void rwnxl_wakeup(IDLE_FUNC wait_func)
{
	UINT32		reg;
    uint32_t v_tmp = 0;
    uint32_t i_tmp = 0,y_tmp = 0;

    if (rwnx_env.hw_in_doze)
    {
        // Start the Wake-Up from doze procedure
        nxmac_wake_up_from_doze_setf(1);
        rwnx_env.hw_in_doze = false;

        // Wait for idle interrupt
        //while (nxmac_status_idle_interrupt_getf() != 1);
        while(1) 
        {
            if(nxmac_status_idle_interrupt_getf() == 1)
                break;
            else
            {
                wait_func();
                i_tmp++;
                if(i_tmp > 1000)
                {
                    i_tmp = 0;
                    v_tmp =nxmac_state_cntrl_get();
                    PS_WPRT(" w s:0x%x c:0x%x\r\n",v_tmp,power_save_get_sleep_count());
                    y_tmp ++;
                    if(y_tmp > 10000)
                    {
                    y_tmp = 0;
                    os_printf(" w s:0x%x c:0x%x\r\n",v_tmp,power_save_get_sleep_count());
                    }
                }
            }
        }
        nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);

        // Move back to the previous state
        if (rwnx_env.prev_hw_state != HW_IDLE)
        {
            nxmac_next_state_setf(rwnx_env.prev_hw_state);
            while (nxmac_current_state_getf() != rwnx_env.prev_hw_state);
        }

        // Wake-Up from doze procedure is done
        nxmac_wake_up_from_doze_setf(0);

#if 1
		nxmac_timers_int_un_mask_set(nxmac_timer_saved);
#endif      
    }
}
#endif

static UINT8 prev_mac_state = 0;

void wifi_mac_state_set_idle(void)
{
    uint32_t v_tmp = 0;
    uint32_t i_tmp = 0,y_tmp = 0;

    prev_mac_state = nxmac_current_state_getf();
    // Ask HW to go to IDLE
    if (nxmac_current_state_getf() != HW_IDLE)
    {
        nxmac_next_state_setf(HW_IDLE);
        while(1) 
            {
                if(nxmac_status_idle_interrupt_getf() == 1)
                    break;
                else
                    {
                i_tmp++;
                if(i_tmp > 1000)
                {
                    i_tmp = 0;
                    v_tmp =nxmac_state_cntrl_get();
                    y_tmp ++;
                    if(y_tmp > 50)
                    {
                        y_tmp = 0;
                        //long time wait idle interrupt,recover!
                        hal_machw_disable_int();
                        rwnxl_reset_handle(0);
                    }
                    nxmac_next_state_setf(HW_IDLE);
                }
                }
            }
        //while (nxmac_status_idle_interrupt_getf() != 1);
        nxmac_gen_int_ack_clear(NXMAC_IDLE_INTERRUPT_BIT);
    }
}

void wifi_mac_state_set_active(void)
{
    if (nxmac_current_state_getf() != HW_ACTIVE)
    {
    nxmac_next_state_setf(HW_ACTIVE);
    while (nxmac_current_state_getf() != HW_ACTIVE);
    }
}

void wifi_mac_state_set_prev(void)
{
    if (nxmac_current_state_getf() != prev_mac_state)
    {
    nxmac_next_state_setf(prev_mac_state);
    while (nxmac_current_state_getf() != prev_mac_state);
    }
}

#endif

#endif

/// @}
