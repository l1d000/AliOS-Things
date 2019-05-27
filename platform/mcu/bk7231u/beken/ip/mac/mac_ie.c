/**
 ****************************************************************************************
 *
 * @file mac_ie.c
 *
 * @brief MAC Information Elements related API definitions.
 *
 * Copyright (C) RivieraWaves 2011-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_MAC_IE
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
// minimum default inclusion directive
#include "mac_ie.h"
// for memcmp and memmove
#include <string.h>
// for ASSERT_ERR
#include "arch.h"
// for management frame related constants
#include "mac_frame.h"
#include "co_utils.h"
#include "include.h"
#include "uart_pub.h"
#include "ieee802_11_defs.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint32_t mac_ie_find(uint32_t addr,
                     uint16_t buflen,
                     uint8_t ie_id)
{
    uint32_t end = addr + buflen;
    // loop as long as we do not go beyond the frame size
    while (addr < end)
    {
        // check if the current IE is the one looked for
        if (ie_id == co_read8p(addr))
        {
            // the IE id matches and it is not OUI IE, return the pointer to this IE
            return addr;
        }
        // move on to the next IE
        addr += co_read8p(addr + MAC_INFOELT_LEN_OFT) + MAC_INFOELT_INFO_OFT;
    }

    // sanity check: the offset can not be greater than the length
    if(addr != end)
    {
    	os_printf("mac_ie_find_assert\r\n");
    }

    return 0;
}


uint32_t mac_vsie_find(uint32_t addr,
                       uint16_t buflen,
                       uint8_t const *oui,
                       uint8_t ouilen)
{
    uint32_t end = addr + buflen;

    // loop as long as we do not go beyond the frame size
    while (addr < end)
    {
        // check if the current IE is the one looked for
        if (co_read8p(addr) == MAC_ELTID_OUI)
        {
            // check if the OUI matches the one we are looking for
            if (co_cmp8p(addr + MAC_INFOELT_INFO_OFT, oui, ouilen))
            {
                // the OUI matches, return the pointer to this IE
                return addr;
            }
        }
        // move on to the next IE
        addr += co_read8p(addr + MAC_INFOELT_LEN_OFT) + MAC_INFOELT_INFO_OFT;
    }

    // sanity check: the offset can not be greater than the length
    ASSERT_ERR(addr == end);

    return 0;
}

uint8_t *mac_vendor_ie_find(unsigned int oui, 
							uint8_t oui_type, 
							uint8_t *ies, 
							int len)
{
    struct ieee80211_vendor_ie *ie;
    uint8_t *pos = ies, *end = ies + len;
    int ie_oui;

    while (pos < end) {
        pos = (uint8_t *)mac_ie_find((uint32_t)pos,
                               end - pos, WLAN_EID_VENDOR_SPECIFIC);
        if (!pos) {
            return NULL;
        }

        ie = (struct ieee80211_vendor_ie *)pos;

        if (ie->len < sizeof(*ie)) {
            goto cont;
        }

        ie_oui = ie->oui[0] << 16 | ie->oui[1] << 8 | ie->oui[2];
        if (ie_oui == oui && ie->oui_type == oui_type) {
            return pos;
        }
cont:
        pos += 2 + ie->len;
    }
	
    return NULL;
}
							
//eof

