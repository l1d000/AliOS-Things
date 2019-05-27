/**
 ****************************************************************************************
 *
 * @file kernel_event.c
 *
 * @brief This file contains the event handling primitives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup EVT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"  // stack configuration

#include "architect.h"         // architecture
#include <stdint.h>       // standard integer definition
#include <stddef.h>       // standard definition
#include <string.h>       // memcpy defintion

#include "common_math.h"      // maths definitions
#include "kernel_event.h"     // kernel event

#include "ble_pub.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Format of an event callback function
typedef void (*p_callback_t)(void);


/*
 * STRUCTURES DEFINTIONS
 ****************************************************************************************
 */

/// KE EVENT environment structure
struct kernel_event_env_tag
{
    /// Event field
    uint32_t event_field;

    /// Callback table
    p_callback_t callback[KERNEL_EVENT_MAX];
};


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// KE EVENT environment
static struct kernel_event_env_tag kernel_event_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void kernel_event_init(void)
{
    memset(&kernel_event_env, 0, sizeof(kernel_event_env));
}

uint8_t kernel_event_callback_set(uint8_t event_type, void (*p_callback)(void))
{
    uint8_t status = KERNEL_EVENT_CAPA_EXCEEDED;

    ASSERT_INFO((event_type < KERNEL_EVENT_MAX) && (p_callback != NULL), event_type, p_callback);

    if(event_type < KERNEL_EVENT_MAX)
    {
        // Store callback
        kernel_event_env.callback[event_type] = p_callback;

        // Status OK
        status = KERNEL_EVENT_OK;
    }

    return (status);
}

void kernel_event_set(uint8_t event_type)
{
    ASSERT_INFO((event_type < KERNEL_EVENT_MAX), event_type, 0);

    GLOBAL_INT_DIS();

    if(event_type < KERNEL_EVENT_MAX)
    {
        // Set the event in the bit field
        kernel_event_env.event_field |= (1 << event_type);
    }

    GLOBAL_INT_RES();

    ble_send_msg(BLE_MSG_POLL);
}

void kernel_event_clear(uint8_t event_type)
{
    ASSERT_INFO((event_type < KERNEL_EVENT_MAX), event_type, 0);

    GLOBAL_INT_DIS();

    if(event_type < KERNEL_EVENT_MAX)
    {
        // Set the event in the bit field
        kernel_event_env.event_field &= ~(1 << event_type);
    }

    GLOBAL_INT_RES();
}

uint8_t kernel_event_get(uint8_t event_type)
{
    uint8_t state = 0;

    ASSERT_INFO((event_type < KERNEL_EVENT_MAX), event_type, 0);

    GLOBAL_INT_DIS();

    if(event_type < KERNEL_EVENT_MAX)
    {
        // Get the event in the bit field
        state = (kernel_event_env.event_field >> event_type) & (0x1);
    }

    GLOBAL_INT_RES();

    return state;
}

uint32_t kernel_event_get_all(void)
{
    return kernel_event_env.event_field;
}

void kernel_event_flush(void)
{
    kernel_event_env.event_field = 0;
}

void kernel_event_schedule(void)
{
    uint8_t hdl;

    // Get the volatile value
    uint32_t field = kernel_event_env.event_field;

    while (field) // Compiler is assumed to optimize with loop inversion
    {
        // Find highest priority event set
        hdl = 32 - (uint8_t) common_clz(field) - 1;

        // Sanity check
        ASSERT_INFO(hdl < KERNEL_EVENT_MAX, hdl, field);

        if(kernel_event_env.callback[hdl] != NULL)
        {
            // Execute corresponding handler
            (kernel_event_env.callback[hdl])();
        }
        else
        {
            ASSERT_ERR(0);
        }

        // Update the volatile value
        field = kernel_event_env.event_field;
    }
}


///@} KERNEL_EVT
