/**
 ****************************************************************************************
 *
 * @file hci_util.c
 *
 * @brief HCI module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)

#include <string.h>          // string manipulation
#include "common_error.h"        // error definition
#include "common_utils.h"        // common utility definition
#include "common_endian.h"       // common endianess definition
#include "common_math.h"         // common math definition

#include "hci_int.h"         // HCI internal definitions


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
/// Extract length of an array from a format string
static uint16_t hci_util_read_array_size(char **fmt_cursor)
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

/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t hci_util_pack(uint8_t* inout, uint16_t* inout_len, const char* format)
{
    uint8_t status = HCI_PACK_OK;
    uint8_t* p_in = inout;
    uint8_t* p_out = inout;
    uint8_t* p_in_end = inout + *inout_len;
    char* cursor = (char*) format;
    bool b_copy = (inout != NULL);

    ASSERT_ERR(format != NULL);

    while((*cursor != '\0') && (status == HCI_PACK_OK))
    {
        uint16_t nb = 0;

        // Check if the new field is an array (starting with a number)
        if((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = hci_util_read_array_size(&cursor);
        }

        // Parse the format string
        switch (*cursor++)
        {
            case ('B'): // Byte
            {
                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if((p_in + 1) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    *p_out = *p_in;
                }

                // Move pointers
                p_out++;
                p_in++;

                // For arrays only
                if(nb > 1)
                {
                    if(b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if((p_in + nb - 1) > p_in_end)
                        {
                            status = HCI_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Copy bytes
                        memcpy(p_out, p_in, nb-1);
                    }

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

                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if(((uint8_t *)(short_word + 1)) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    common_write16p(p_out, *short_word);
                }

                // Move pointers
                p_in = (uint8_t *)(short_word + 1);
                p_out += 2;
            }
            break;

            case ('L'): // Long Word
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)COMMON_ALIGN4_HI((uint32_t)p_in);

                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if(((uint8_t *)(long_word + 1)) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    common_write32p(p_out, *long_word);
                }

                // Move pointers
                p_in = (uint8_t *)(long_word + 1);
                p_out += 4;
            }
            break;

            default:
            {
                // data format error
                status = HCI_PACK_WRONG_FORMAT;
            }
            break;
        }
    }

    if(status == HCI_PACK_OK)
    {
        *inout_len = (uint16_t)(p_out - inout);
    }

    return (status);
}

uint8_t hci_util_unpack(uint8_t* out, uint8_t* in, uint16_t* out_len, uint16_t in_len, const char* format)
{
    uint8_t status = HCI_PACK_OK;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    char* cursor = (char*) format;
    bool b_copy = ((out != NULL) && (in != NULL));

    ASSERT_ERR(format != NULL);

    while((*cursor != '\0') && (status == HCI_PACK_OK))
    {
        uint16_t nb = 0;

        // Check if the new field is an array (starting with a number)
        if((*cursor >= '0') && (*cursor <= '9'))
        {
            nb = hci_util_read_array_size(&cursor);
        }

        // Parse the format string
        switch (*cursor++)
        {
            case ('B'): // Byte
            {
                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if((p_in + 1) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if((p_out + 1) > p_out_end)
                    {
                        status = HCI_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    *p_out = *p_in;
                }

                // Move pointers
                p_out++;
                p_in++;

                // For arrays only
                if(nb > 1)
                {
                    if(b_copy)
                    {
                        // Check if enough space in input buffer to read
                        if((p_in + nb - 1) > p_in_end)
                        {
                            status = HCI_PACK_IN_BUF_OVFLW;
                            break;
                        }

                        // Check if enough space in out buffer to write
                        if((p_out + nb - 1) > p_out_end)
                        {
                            status = HCI_PACK_OUT_BUF_OVFLW;
                            break;
                        }

                        // Copy bytes
                        memcpy(p_out, p_in, nb-1);
                    }

                    // Move pointers
                    p_out += (nb-1);
                    p_in += (nb-1);
                }
            }
            break;

            case ('H'): // Short Word
            {
                // Align data buffer to a 16-bits address
                uint16_t *short_word = (uint16_t *)COMMON_ALIGN2_HI((uint32_t)p_out);

                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if((p_in + 2) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if(((uint8_t *)(short_word + 1)) > p_out_end)
                    {
                        status = HCI_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    *short_word = common_read16p(p_in);
                }

                // Move pointers
                p_out = (uint8_t *)(short_word + 1);
                p_in += 2;
            }
            break;

            case ('L'): // Long Word
            {
                // Align data buffer to a 32-bits address
                uint32_t *long_word = (uint32_t *)COMMON_ALIGN4_HI((uint32_t)p_out);

                if(b_copy)
                {
                    // Check if enough space in input buffer to read
                    if((p_in + 4) > p_in_end)
                    {
                        status = HCI_PACK_IN_BUF_OVFLW;
                        break;
                    }

                    // Check if enough space in out buffer to write
                    if(((uint8_t *)(long_word + 1)) > p_out_end)
                    {
                        status = HCI_PACK_OUT_BUF_OVFLW;
                        break;
                    }

                    // Copy data
                    *long_word = common_read32p(p_in);
                }

                // Move pointers
                p_out = (uint8_t *)(long_word + 1);
                p_in += 4;
            }
            break;

            default:
            {
                // data format error
                status = HCI_PACK_WRONG_FORMAT;
            }
            break;
        }
    }

    // Check a potential mismatch between the theoretical (measured) input length and the given input length
    if(p_in > p_in_end)
    {
        status = HCI_PACK_IN_BUF_OVFLW;
    }

    // Return the total size needed for unpacked parameters
    *out_len = (uint16_t)(p_out - out);

    return (status);
}
#endif //(HCI_TL_SUPPORT)



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */



#endif //HCI_PRESENT

/// @} HCI
