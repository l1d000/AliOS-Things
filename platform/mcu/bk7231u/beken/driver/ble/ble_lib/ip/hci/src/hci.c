/**
 ****************************************************************************************
 *
 * @file hci.c
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
#include "common_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "kernel_msg.h"          // kernel message declaration
#include "kernel_task.h"         // kernel task definition
#include "kernel_event.h"        // kernel event definition
#include "kernel_mem.h"          // kernel memory definition
#include "kernel_timer.h"        // kernel timer definition

#if(BLE_HOST_PRESENT && BLE_EMB_PRESENT && HCI_TL_SUPPORT)
#include "gapm.h"            // use to check if embedded host is enabled or not
#endif // (BLE_HOST_PRESENT && BLE_EMB_PRESENT && HCI_TL_SUPPORT)

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
 * CONSTANTS DEFINITIONS
 ****************************************************************************************
 */

/// Default event mask
static const struct evt_mask hci_def_evt_msk =  {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00}};

/// Reserved event mask
static const struct evt_mask hci_rsvd_evt_msk = {{0x00, 0x60, 0x04, 0x00, 0xF8, 0x07, 0x40, 0x02}};

#if BT_EMB_PRESENT
/// This table is used to check the length of the parameters of HCI_SetEventFilter command
static const uint8_t hci_evt_filter_size[3][3] =
{// type 0  1  2
        {1, 2, 3},  // Condition = 0
        {1, 8, 9},  // Condition = 1
        {1, 8, 9}   // Condition = 2
};
#endif //BT_EMB_PRESENT

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI environment context
struct hci_env_tag hci_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Check if the event to be sent to the host is masked or not
 *
 * @param[in] msg  Pointer to the message containing the event
 *
 * @return true id the message has to be filtered out, false otherwise
 *****************************************************************************************
 */
static bool hci_evt_mask_check(struct kernel_msg *msg)
{
    bool masked = false;
    uint8_t evt_code;

    switch(msg->id)
    {
        case HCI_LE_EVENT:
        {
            // LE meta event
            evt_code = HCI_LE_META_EVT_CODE;
        }
        break;
        case HCI_EVENT:
        {
            // Get event code
            evt_code = msg->src_id;
        }
        break;

        default:
        {
            // Cannot be masked
            return false;
        }
    }

    // Check if this event is maskable
    if(evt_code < HCI_MAX_EVT_MSK_PAGE_1_CODE)
    {
        uint8_t index = evt_code - 1;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk.mask[index/8] & (1<<(index - ((index/8)*8)))) == 0x00);
    }
    else if(evt_code < HCI_MAX_EVT_MSK_PAGE_2_CODE)
    {
        // In this area the evt code is in the range [EVT_MASK_CODE_MAX<evt_code<HCI_MAX_EVT_MSK_CODE]
        // The index should be < EVT_MASK_CODE_MAX to avoid evt_msk_page_2.mask array overflow
        uint8_t index = evt_code - EVT_MASK_CODE_MAX;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk_page_2.mask[index/8] & (1<<(index - ((index/8)*8)))) == 0x00);
    }

    return masked;
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if BT_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Check if a connection request event has to be forwarded to the host or not
 * This function also takes the required decisions according to the auto_accept parameter
 *
 * @param[in] bdaddr    BDADDR contained in the inquiry result event
 * @param[in] class     Class of device contained in the inquiry result event
 * @param[in] link_type Type of link that is requested (asynchronous or synchronous)
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_con_check(struct bd_addr *bdaddr, struct devclass   *classofdev, uint8_t link_type, uint8_t link_id)
{
    uint8_t filtered = false;
    uint8_t auto_accept = DO_NOT_AUTO_ACCEPT_CONNECTION;

    /* Check if a Connection type is present (start from last to first)           */
    for (int i = HCI_FILTER_NB - 1 ; i >= 0 ; i--)
    {
        struct hci_evt_filter_tag *filter = &hci_env.evt_filter[i];

        // If this filter is a ConnectionSetupFilter
        if ((filter->in_use == true) && (filter->type == CONNECTION_FILTER_TYPE))
        {
            // There is at least one connection filter set, so now we should reject by default unless
            // the connection request matches one of the filters
            auto_accept = 0xFF;

            // If the condition is a All Device type
            if (filter->condition == ALL_FILTER_CONDITION_TYPE)
            {
                // This connection will not be rejected
                auto_accept = filter->auto_accept;
                break;
            }
            // If the condition is a ClassOfDevice type
            else if (filter->condition == CLASS_FILTER_CONDITION_TYPE)
            {
                struct devclass class_masked;
                struct classofdevcondition *cond = &filter->param.device_class;

                // Remove don't care bit of Class Of Device
                class_masked.A[0] = classofdev->A[0] & cond->class_mask.A[0];
                class_masked.A[1] = classofdev->A[1] & cond->class_mask.A[1];
                class_masked.A[2] = classofdev->A[2] & cond->class_mask.A[2];

                // Check if any device class is allowed or the Class of the filter matches the Class of the Inquiry result
                if (    (cond->classofdev.A[0] == 0 && cond->classofdev.A[1] == 0 && cond->classofdev.A[2] == 0)
                     || !memcmp(&class_masked, &cond->classofdev, sizeof(struct devclass)))
                {
                    // This connection will not be rejected
                    auto_accept = filter->auto_accept;
                    break;
                }
            }
            /* If this filter is a ConnectionFilter                                     */
            else if (filter->condition == BD_ADDR_FILTER_CONDITION_TYPE)
            {
                // Check if the BdAddr of the filter matches the BdAddr of the Connect req
                if (!memcmp(bdaddr, &filter->param.bdaddr, sizeof(struct bd_addr)))
                {
                    // This connection will not be rejected
                    auto_accept = filter->auto_accept;
                    break;
                }
            }
        }
    }

    if ((auto_accept == ACCEPT_CONNECTION_SLAVE) || (auto_accept == ACCEPT_CONNECTION_MASTER))
    {
        // Auto accept the connection request
        filtered = true;

        // If this is an ACL link
        if (link_type == ACL_TYPE)
        {
            struct hci_accept_con_req_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, KERNEL_BUILD_ID(TASK_LC, link_id), HCI_ACCEPT_CON_REQ_CMD_OPCODE, hci_accept_con_req_cmd);

            // Fill-in the parameter structure
            if (auto_accept == ACCEPT_CONNECTION_SLAVE)
            {
                cmd->role = ACCEPT_REMAIN_SLAVE;
            }
            else
            {
                cmd->role = ACCEPT_SWITCH_TO_MASTER;
            }
            cmd->bd_addr = *bdaddr;

            // Send the message
            kernel_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_ACCEPT_CON_REQ_CMD_OPCODE;
        }
        #if (MAX_NB_SYNC > 0)
        // If this is a Synchronous Link (SCO or eSCO)
        else
        {
            struct hci_accept_sync_con_req_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, KERNEL_BUILD_ID(TASK_LC, link_id), HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE, hci_accept_sync_con_req_cmd);

            // Fill in parameter structure
            cmd->bd_addr = *bdaddr;
            cmd->tx_bw = SYNC_BANDWIDTH_DONT_CARE;
            cmd->rx_bw = SYNC_BANDWIDTH_DONT_CARE;
            cmd->max_lat = SYNC_DONT_CARE_LATENCY;
            cmd->vx_set = hci_env.voice_settings;
            cmd->retx_eff = SYNC_RE_TX_DONT_CARE;
            cmd->pkt_type = 0xFFFF;   /// All packet type

            // Send the message
            kernel_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE;
        }
        #endif // (MAX_NB_SYNC > 0)
    }
    else if (auto_accept != DO_NOT_AUTO_ACCEPT_CONNECTION)
    {
        // This Device does not match a filter => filtered and rejected
        filtered = true;

        // If this is an ACL link
        if (link_type == ACL_TYPE)
        {
            struct hci_reject_con_req_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, KERNEL_BUILD_ID(TASK_LC, link_id), HCI_REJECT_CON_REQ_CMD_OPCODE, hci_reject_con_req_cmd);
            cmd->bd_addr = *bdaddr;
            cmd->reason = COMMON_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR;
            kernel_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_REJECT_CON_REQ_CMD_OPCODE;
        }
        #if (MAX_NB_SYNC > 0)
        // If this is a Synchronous Link (SCO or eSCO)
        else
        {
            struct hci_reject_sync_con_req_cmd *cmd = KERNEL_MSG_ALLOC(HCI_COMMAND, KERNEL_BUILD_ID(TASK_LC, link_id), HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE, hci_reject_sync_con_req_cmd);
            cmd->bd_addr = *bdaddr;
            cmd->reason = COMMON_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR;
            kernel_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE;
        }
        #endif // (MAX_NB_SYNC > 0)
    }

    return(filtered);
}

/**
 ****************************************************************************************
 * @brief Check if an inquiry result event has to be forwarded to the host or not
 *
 * @param[in] bdaddr  BDADDR contained in the inquiry result event
 * @param[in] class   Class of device contained in the inquiry result event
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_inq_check(struct bd_addr *bdaddr, struct devclass *classofdev)
{
    int     i;
    uint8_t filtered = false;  // By default the event is not filtered

    // Check if an inquiry filter type is present
    for (i = 0; i < HCI_FILTER_NB; i++)
    {
        struct hci_evt_filter_tag *filter = &hci_env.evt_filter[i];

        // If this filter is an InquiryFilter
        if ((filter->in_use == true) && (filter->type == INQUIRY_FILTER_TYPE))
        {
            // There is at least one inquiry filter set, so now we should filter unless
            // the inquiry result matches one of the filters
            filtered = true;

            // If the condition is a ClassOfDevice type
            if (filter->condition == CLASS_FILTER_CONDITION_TYPE)
            {
                struct devclass class_masked;
                struct classofdevcondition *cond = &filter->param.device_class;

                // Remove don't care bit of Class Of Device
                class_masked.A[0] = classofdev->A[0] & cond->class_mask.A[0];
                class_masked.A[1] = classofdev->A[1] & cond->class_mask.A[1];
                class_masked.A[2] = classofdev->A[2] & cond->class_mask.A[2];

                // Check if the Class of the filter match the Class of the Inquiry result
                if (!memcmp(&class_masked.A[0], &cond->classofdev.A[0], sizeof(struct devclass)))
                {
                    // This InquiryResult must NOT be filtered
                    filtered = false;
                    break;
                }
            }
            // If this filter is a BDADDR type
            else if (filter->condition == BD_ADDR_FILTER_CONDITION_TYPE)
            {
                // Check if the BdAddr of the filter match the BdAddr of the Inquiry res
                if (!memcmp(bdaddr, &filter->param.bdaddr, sizeof(struct bd_addr)))
                {
                    // This InquiryResult must NOT be filtered
                    filtered = false;
                    break;
                }
            }
        }
    }

    return(filtered);
}

/**
 ****************************************************************************************
 * @brief Check if an event has to be forwarded to the host or not
 *
 * @param[in] msg  Pointer to the event message to be transmitted to the host
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_check(struct kernel_msg *msg)
{
    uint8_t filtered = false;

    switch(msg->id)
    {
        case HCI_EVENT:
        {
            // Get event code
            uint8_t evt_code = msg->src_id;
            uint8_t *param = (uint8_t *)msg->param;

            switch(evt_code)
            {
                // InquiryResult Event
                case HCI_INQ_RES_EVT_CODE:
                case HCI_INQ_RES_WITH_RSSI_EVT_CODE:
                case HCI_EXT_INQ_RES_EVT_CODE:
                {
                    struct devclass *classofdev;
                    // Retrieve the information required for the filtering from the PDU
                    struct bd_addr * bdaddr = (struct bd_addr *)&param[1];
                    if(evt_code == HCI_INQ_RES_EVT_CODE)
                    {
                        struct hci_inq_res_evt *evt = (struct hci_inq_res_evt *) param;
                        classofdev = (struct devclass *)&evt->class_of_dev;
                    }
                    else
                    {
                        struct hci_ext_inq_res_evt *evt = (struct hci_ext_inq_res_evt *) param;
                        classofdev = (struct devclass *)&evt->class_of_dev;
                    }

                    // Check if the event has to be filtered or not
                    filtered = hci_evt_filter_inq_check(bdaddr, classofdev);
                }
                break;
                // Connection Request Event
                case HCI_CON_REQ_EVT_CODE:
                {
                    // Retrieve the information required for the filtering from the PDU
                    struct bd_addr * bdaddr = (struct bd_addr *)&param[0];
                    struct devclass *classofdev = (struct devclass *)&param[6];
                    uint8_t link_type = param[9];

                    // Check if the event has to be filtered or not
                    filtered = hci_evt_filter_con_check(bdaddr, classofdev, link_type, msg->dest_id);
                }
                break;
                // Connection Complete Event
                case HCI_CON_CMP_EVT_CODE:
                {
                    // Check if a connection was auto-rejected
                    if(hci_env.auto_reject)
                    {
                        filtered = true;
                        hci_env.auto_reject = false;
                    }
                }
                break;
                default:
                {
                    // Nothing to do
                }
            }
        }
        break;

        case HCI_CMD_STAT_EVENT:
        {
            // Filter CS event associated to the current auto-accept command
            if(msg->src_id == hci_env.auto_accept_opcode)
            {
                filtered = true;
                hci_env.auto_accept_opcode = 0x0000;

                if(msg->src_id == HCI_REJECT_CON_REQ_CMD_OPCODE)
                {
                    hci_env.auto_reject = true;
                }
            }
        }
        break;

        default:
        {
            // Not an event
        }
        break;
    }

    return filtered;
}

/**
 ****************************************************************************************
 * @brief Reset the list of event filters
 *
 * @return Status
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_reset(void)
{
    for (int i = 0 ; i < HCI_FILTER_NB; i++)
    {
        hci_env.evt_filter[i].in_use = false;
    }
    return (COMMON_ERROR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Allocate an event filter structure
 *
 * @return A pointer to the allocated filter, if any, NULL otherwise
 *****************************************************************************************
 */
static struct hci_evt_filter_tag *hci_evt_filter_alloc(void)
{
    int i;
    struct hci_evt_filter_tag *evt_filter;

    for (i = 0; i < HCI_FILTER_NB; i++)
    {
        if (hci_env.evt_filter[i].in_use == false)
        {
            break;
        }
    }
    if (i < HCI_FILTER_NB)
    {
        evt_filter = &hci_env.evt_filter[i];
        evt_filter->in_use = true;
    }
    else
    {
        evt_filter = NULL;
    }

    return(evt_filter);
}

/**
 ****************************************************************************************
 * @brief Add an inquiry event filter
 *
 * @param[in] condition  Filter condition type
 * @param[in] param      Pointer to the condition parameters
 *
 * @return The status of the filter addition
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_inq_add(struct inq_res_filter const * filter)
{
    uint8_t status = COMMON_ERROR_INVALID_HCI_PARAM;
    struct hci_evt_filter_tag *evt_filter;

    switch (filter->cond_type)
    {
        case ALL_FILTER_CONDITION_TYPE:
        {
            // Remove all Inquiry type
            for (int i = 0 ; i < HCI_FILTER_NB ; i++)
            {
                if (hci_env.evt_filter[i].type == INQUIRY_FILTER_TYPE)
                {
                    hci_env.evt_filter[i].in_use = false;
                }
            }

            status = COMMON_ERROR_NO_ERROR;
        }
        break;

        case CLASS_FILTER_CONDITION_TYPE:
        case BD_ADDR_FILTER_CONDITION_TYPE:
        {
            // Add the new filter
            evt_filter = hci_evt_filter_alloc();
            if (evt_filter != NULL)
            {
                evt_filter->type = INQUIRY_FILTER_TYPE;
                evt_filter->condition = filter->cond_type;
                if (filter->cond_type == CLASS_FILTER_CONDITION_TYPE)
                {
                    struct classofdevcondition *dev_class = &evt_filter->param.device_class;
                    // Store the mask
                    dev_class->class_mask = filter->cond.cond_1.class_of_dev_msk;

                    // Store the class, masked with the class mask
                    dev_class->classofdev.A[0] = filter->cond.cond_1.class_of_dev.A[0] & filter->cond.cond_1.class_of_dev_msk.A[0];
                    dev_class->classofdev.A[1] = filter->cond.cond_1.class_of_dev.A[1] & filter->cond.cond_1.class_of_dev_msk.A[1];
                    dev_class->classofdev.A[2] = filter->cond.cond_1.class_of_dev.A[2] & filter->cond.cond_1.class_of_dev_msk.A[2];
                }
                else
                {
                    evt_filter->param.bdaddr = filter->cond.cond_2.bd_addr;
                }
                status = COMMON_ERROR_NO_ERROR;
            }
            else
            {
                status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
            }
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
    }
    return(status);
}


/**
 ****************************************************************************************
 * @brief Add a connection event filter
 *
 * @param[in] condition  Filter condition type
 * @param[in] param      Pointer to the condition parameters
 *
 * @return The status of the filter addition
 *****************************************************************************************
 */
static uint8_t hci_evt_filter_con_add(struct con_set_filter const * filter)
{
    uint8_t status = COMMON_ERROR_NO_ERROR;
    struct hci_evt_filter_tag *evt_filter;

    switch (filter->cond_type)
    {
        case ALL_FILTER_CONDITION_TYPE:
        {
            uint8_t auto_accept = filter->cond.cond_0.auto_accept;
            // Check auto_accept parameter
            if ((auto_accept >= DO_NOT_AUTO_ACCEPT_CONNECTION) && (auto_accept <= ACCEPT_CONNECTION_MASTER))
            {
                // Remove all Connection type
                for (int i = 0 ; i < HCI_FILTER_NB ; i++)
                {
                    if (hci_env.evt_filter[i].type == CONNECTION_FILTER_TYPE)
                    {
                        hci_env.evt_filter[i].in_use = false;
                    }
                }
                // Add the new filter
                evt_filter = hci_evt_filter_alloc();
                if (evt_filter != NULL)
                {
                    evt_filter->type = CONNECTION_FILTER_TYPE;
                    evt_filter->condition = ALL_FILTER_CONDITION_TYPE;
                    evt_filter->auto_accept = auto_accept;
                }
                else
                {
                    status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
                }
            }
        }
        break;

        case CLASS_FILTER_CONDITION_TYPE:
        case BD_ADDR_FILTER_CONDITION_TYPE:
        {
            uint8_t auto_accept = filter->cond.cond_1.auto_accept;
            // Check auto_accept parameter
            if ((auto_accept >= DO_NOT_AUTO_ACCEPT_CONNECTION) && (auto_accept <= ACCEPT_CONNECTION_MASTER))
            {
                // Remove all Connection type with ALL_FILTER_CONDITION_TYPE set
                for (int i = 0; i < HCI_FILTER_NB ; i++)
                {
                    if ((hci_env.evt_filter[i].in_use == true) &&
                        (hci_env.evt_filter[i].type == CONNECTION_FILTER_TYPE) &&
                        (hci_env.evt_filter[i].condition == ALL_FILTER_CONDITION_TYPE))
                    {
                        hci_env.evt_filter[i].in_use = false;
                    }
                }
                // Add the new filter
                evt_filter = hci_evt_filter_alloc();
                if (evt_filter != NULL)
                {
                    evt_filter->type = CONNECTION_FILTER_TYPE;
                    evt_filter->condition = filter->cond_type;
                    evt_filter->auto_accept = auto_accept;
                    if (filter->cond_type == CLASS_FILTER_CONDITION_TYPE)
                    {
                        struct classofdevcondition *dev_class = &evt_filter->param.device_class;
                        // Store the mask
                        dev_class->class_mask = filter->cond.cond_1.class_of_dev_msk;

                        // Store the class, masked with the class mask
                        dev_class->classofdev.A[0] = filter->cond.cond_1.class_of_dev.A[0] & filter->cond.cond_1.class_of_dev_msk.A[0];
                        dev_class->classofdev.A[1] = filter->cond.cond_1.class_of_dev.A[1] & filter->cond.cond_1.class_of_dev_msk.A[1];
                        dev_class->classofdev.A[2] = filter->cond.cond_1.class_of_dev.A[2] & filter->cond.cond_1.class_of_dev_msk.A[2];
                    }
                    else
                    {
                        evt_filter->param.bdaddr = filter->cond.cond_2.bd_addr;
                    }
                }
                else
                {
                    status = COMMON_ERROR_MEMORY_CAPA_EXCEED;
                }
            }
            else
            {
                status = COMMON_ERROR_INVALID_HCI_PARAM;
            }
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
    }

    return(status);
}
#endif //BT_EMB_PRESENT


/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_init(void)
{
    memset(&hci_env, 0, sizeof(hci_env));

    // Initialize event mask
    hci_evt_mask_set(&hci_def_evt_msk, HCI_PAGE_DFT);

    #if (HCI_TL_SUPPORT)
    // Reset the HCI
    hci_tl_init(false);
    #endif //(HCI_TL_SUPPORT)
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    hci_fc_init();
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)
}

void hci_reset(void)
{
    memset(&hci_env, 0, sizeof(hci_env));

    // Initialize event mask
    hci_evt_mask_set(&hci_def_evt_msk, HCI_PAGE_DFT);

    #if (HCI_TL_SUPPORT)
    // Reset the HCI
    hci_tl_init(true);
    #endif //(HCI_TL_SUPPORT)

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    hci_fc_init();
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
void hci_send_2_host(void *param)
{
    struct kernel_msg *msg = kernel_param2msg(param);

    if(   hci_evt_mask_check(msg)
    #if BT_EMB_PRESENT
       || hci_evt_filter_check(msg)
    #endif //BT_EMB_PRESENT
      )
    {
        // Free the kernel message space
        kernel_msg_free(msg);
        return;
    }

    #if BLE_HOST_PRESENT
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    // check if communication is performed over embedded host
    if(gapm_is_embedded_host())
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    {
        kernel_task_id_t dest = TASK_BLE_NONE;
        uint8_t hl_type = HL_UNDEF;

        // The internal destination first depends on the message type (command, event, data)
        switch(msg->id)
        {
            case HCI_CMD_STAT_EVENT:
            case HCI_CMD_CMP_EVENT:
            {
                if(msg->src_id != HCI_NO_OPERATION_CMD_OPCODE)
                {
                    // Find a command descriptor associated to the command opcode
                    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(msg->src_id);

                    // Check if the command is supported
                    if(cmd_desc != NULL)
                    {
                        hl_type = (cmd_desc->dest_field & HCI_CMD_DEST_HL_MASK) >> HCI_CMD_DEST_HL_POS;
                    }
                }
                else
                {
                    hl_type = HL_MNG;
                }
            }
            break;
            case HCI_EVENT:
            {
                // Find an event descriptor associated to the event code
                const struct hci_evt_desc_tag* evt_desc = hci_look_for_evt_desc(msg->src_id);

                // Check if the event is supported
                if(evt_desc != NULL)
                {
                    hl_type = (evt_desc->dest_field & HCI_EVT_DEST_HL_MASK) >> HCI_EVT_DEST_HL_POS;
                }
            }
            break;
            case HCI_LE_EVENT:
            {
                uint8_t subcode = *((uint8_t *)kernel_msg2param(msg));

                // Find an LE event descriptor associated to the LE event subcode
                const struct hci_evt_desc_tag* evt_desc = hci_look_for_le_evt_desc(subcode);

                // Check if the event is supported
                if(evt_desc != NULL)
                {
                    hl_type = (evt_desc->dest_field & HCI_EVT_DEST_HL_MASK) >> HCI_EVT_DEST_HL_POS;
                }
            }
            break;
            #if (HCI_BLE_CON_SUPPORT)
            case HCI_ACL_DATA_RX:
            {
                hl_type = HL_DATA;
            }
            break;
            #endif // (HCI_BLE_CON_SUPPORT)

            default:
            {
                // Nothing to do
            }
            break;
        }

        // Find the higher layers destination task
        switch(hl_type)
        {
            case HL_MNG:
            {
                // Build the destination task ID
                dest = TASK_GAPM;
            }
            break;

            #if (HCI_BLE_CON_SUPPORT)
            case HL_CTRL:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KERNEL_BUILD_ID(TASK_GAPC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;

            case HL_DATA:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KERNEL_BUILD_ID(TASK_L2CC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;
            #endif //(HCI_BLE_CON_SUPPORT)
            #if (BLE_AUDIO)
            case HL_AM0:
            {
                // Build the destination task ID
                dest = TASK_AM0;
            }
            break;
            #endif // BLE_AUDIO
            default:
            {
                ASSERT_INFO(0, hl_type, 0);
            }
            break;
        }

        // Check it the destination has been found
        if(dest != TASK_BLE_NONE)
        {
            // Send the command to the internal destination task associated to this command
            msg->dest_id = dest;
            kernel_msg_send(param);
        }
        else
        {
            ASSERT_INFO(0, msg->id, msg->src_id);

            // Free message to avoid memory leak
            kernel_msg_free(msg);
        }
    }
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    else
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    #endif //BLE_HOST_PRESENT
    #if (HCI_TL_SUPPORT)
    {
        // Send the HCI message over TL
        hci_tl_send(msg);
    }
    #endif //(HCI_TL_SUPPORT)
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if BLE_HOST_PRESENT
void hci_send_2_controller(void *param)
{
    struct kernel_msg *msg = kernel_param2msg(param);

    #if (HCI_TL_SUPPORT && !BLE_EMB_PRESENT)
    // Send the HCI message over TL
    hci_tl_send(msg);
    #else //(HCI_TL_SUPPORT)
    #if BLE_EMB_PRESENT

    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    // check if communication is performed over embedded host
    if(gapm_is_embedded_host())
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    {
        kernel_task_id_t dest = TASK_BLE_NONE;
        uint8_t ll_type = LL_UNDEF;

        // The internal destination first depends on the message type (command, event, data)
        switch(msg->id)
        {
            case HCI_COMMAND:
            {
                // Find a command descriptor associated to the command opcode
                const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(msg->src_id);

                // Check if the command is supported
                if(cmd_desc != NULL)
                {
                    ll_type = (cmd_desc->dest_field & HCI_CMD_DEST_LL_MASK) >> HCI_CMD_DEST_LL_POS;
                }
            }
            break;
            #if (HCI_BLE_CON_SUPPORT)
            case HCI_ACL_DATA_TX:
            {
                ll_type = BLE_CTRL;
            }
            break;
            #endif // (HCI_BLE_CON_SUPPORT)

            default:
            {
                // Nothing to do
            }
            break;
        }

        switch(ll_type)
        {
            case MNG:
            case BLE_MNG:
            {
                // Build the destination task ID
                dest = TASK_LLM;
            }
            break;

            #if (HCI_BLE_CON_SUPPORT)
            case CTRL:
            case BLE_CTRL:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KERNEL_BUILD_ID(TASK_LLC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;
            #endif //(HCI_BLE_CON_SUPPORT)
            case DBG:
            {
                dest = TASK_DBG;
            }break;
            default:
            {
                // Nothing to do
            }
            break;
        }

        // Check it the destination has been found
        if(dest != TASK_BLE_NONE)
        {
            // Send the command to the internal destination task associated to this command
            msg->dest_id = dest;
            kernel_msg_send(param);
        }
        else
        {
            ASSERT_INFO(0, msg->id, msg->src_id);

            // Free message to avoid memory leak
            kernel_msg_free(msg);
        }
    }
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    else
    {
        // receiving a message from internal host is not expected at all
        ASSERT_ERR(0);
        // Free message to avoid memory leak
        kernel_msg_free(msg);
    }
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    #endif //BLE_EMB_PRESENT
    #endif //(HCI_TL_SUPPORT)
}
#endif //BLE_HOST_PRESENT

#if  (BT_EMB_PRESENT)
void hci_bt_acl_bdaddr_register(uint8_t link_id, struct bd_addr* bd_addr)
{
    ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);
    ASSERT_INFO(hci_env.bt_acl_con_tab[link_id].state == HCI_BT_ACL_STATUS_NOT_ACTIVE, hci_env.bt_acl_con_tab[link_id].state, 0);

    // Store BD address
    memcpy(&hci_env.bt_acl_con_tab[link_id].bd_addr.addr[0], &bd_addr->addr[0], sizeof(struct bd_addr));

    // Link ID associated with BD address
    hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_BD_ADDR;
}

void hci_bt_acl_conhdl_register(uint8_t link_id)
{
    ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);
    ASSERT_INFO(hci_env.bt_acl_con_tab[link_id].state == HCI_BT_ACL_STATUS_BD_ADDR, hci_env.bt_acl_con_tab[link_id].state, 0);

    // Link ID associated with BD address AND connection handle
    hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_BD_ADDR_CONHDL;
}

void hci_bt_acl_bdaddr_unregister(uint8_t link_id)
{
    ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);

    // Link ID associated with BD address
    hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_NOT_ACTIVE;
}
#endif //(BT_EMB_PRESENT)

uint8_t hci_evt_mask_set(struct evt_mask const *evt_msk, uint8_t page)
{
    uint8_t i;

    switch(page)
    {
        case HCI_PAGE_0:
        case HCI_PAGE_1:
        {
            ASSERT_INFO(page == HCI_PAGE_DFT,page,page);
        }break;
        case HCI_PAGE_2:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk_page_2.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);
        }break;
        case HCI_PAGE_DFT:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);

            // ensure that reserved bit are set
            for (i = 0; i < EVT_MASK_LEN; i++)
            {
                hci_env.evt_msk.mask[i] |= hci_rsvd_evt_msk.mask[i];
            }
        }break;
        default:
        {
            ASSERT_ERR(0);
        }break;
    }
    return COMMON_ERROR_NO_ERROR;
}

#if (BT_EMB_PRESENT)
uint8_t hci_evt_filter_add(struct hci_set_evt_filter_cmd const *param)
{
    uint8_t status = COMMON_ERROR_INVALID_HCI_PARAM;

    // Perform the requested action according to the filter type
    switch (param->filter_type)
    {
        case CLEAR_ALL_FILTER_TYPE:
        {
            // Reset all Filters
            status = hci_evt_filter_reset();
        }
        break;

        case INQUIRY_FILTER_TYPE:
        {
            // Add inquiry event filter
            status = hci_evt_filter_inq_add(&param->filter.inq_res);
        }
        break;

        case CONNECTION_FILTER_TYPE:
        {
            // Add connection event filter
            status = hci_evt_filter_con_add(&param->filter.con_set);
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
    }

    return (status);
}

#if (MAX_NB_SYNC > 0)
uint16_t hci_voice_settings_get(void)
{
    return(hci_env.voice_settings);
}

uint8_t hci_voice_settings_set(uint16_t voice_settings)
{
    uint8_t status = COMMON_ERROR_UNSUPPORTED;

    // Check if the requested voice settings are supported
    switch (voice_settings & AIR_COD_MSK)
    {
        case AIR_COD_CVSD:
        case AIR_COD_MULAW:
        case AIR_COD_ALAW:
        case AIR_COD_TRANS:
        {
            hci_env.voice_settings = voice_settings;
            status = COMMON_ERROR_NO_ERROR;
        }
        break;
        default:
            break;
    }

    return(status);
}
#endif // (MAX_NB_SYNC > 0)
#endif //(BT_EMB_PRESENT)

#endif //(HCI_PRESENT)

/// @} HCI
