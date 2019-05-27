/**
 ****************************************************************************************
 *
 * @file h4tl.c
 *
 * @brief H4 UART Transport Layer source code.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup H4TL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "include.h"
#include "rwip_config.h"      // stack configuration
#if (H4TL_SUPPORT)

#include <string.h>           // in order to use memset
#include "common_endian.h"        // endian-ness definition
#include "common_utils.h"         // stack common utility definitions
#include "common_error.h"         // error definition
#include "common_hci.h"           // HCI definitions
#include "h4tl.h"             // hci External Interface definition
#include "kernel_mem.h"           // kernel memory
#include "kernel_msg.h"           // kernel event
#include "kernel_event.h"         // kernel event definition
#include "rwip.h"             // rw bt core interrupt
#if (HCI_TL_SUPPORT)
#include "hci.h"              // hci definition
#endif //(HCI_TL_SUPPORT))
#if (AHI_TL_SUPPORT)
#include "ahi.h"              // Application Host Interface definition
#include "gapm_task.h"        // GAP Manager Task
#include "gapm.h"             // GAP Manager Native API
#endif // (AHI_TL_SUPPORT)
#if defined(CFG_AUDIO_AOAHI)
#include "aoahi.h"
#endif // defined(CFG_AUDIO_AOAHI)
#include "RomCallFlash.h"
#include "uart_pub.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Size of the RX Buffer that can both receive the header and valid packet to exit from out of sync mode
#if (AHI_TL_SUPPORT)
#define RX_TMP_BUFF_SIZE     AHI_RESET_MSG_LEN 
#elif (HCI_TL_SUPPORT)
#define RX_TMP_BUFF_SIZE     HCI_RESET_MSG_LEN 
#endif // (AHI_TL_SUPPORT) or (HCI_TL_SUPPORT)
/*
 * ENUMERATIONS DEFINTION
 ****************************************************************************************
 */

///H4TL RX states
enum H4TL_STATE_RX {
	H4TL_STATE_RX_IDLE,
	///H4TL RX Start State - receive message type
	H4TL_STATE_RX_START,
	///H4TL RX Header State - receive message header
	H4TL_STATE_RX_HDR,
	///H4TL RX Header State - receive (rest of) message payload
	H4TL_STATE_RX_PAYL,
	///H4TL RX Out Of Sync state - receive message type
	H4TL_STATE_RX_OUT_OF_SYNC
};

///H4TL TX states
enum H4TL_STATE {
	///H4TL TX Start State - when packet is ready to be sent
	H4TL_STATE_TX_ONGOING,
	///H4TL TX Done State - TX ended with no error
	H4TL_STATE_TX_DONE,
	///H4TL TX In Idle State - No TX Pending
	H4TL_STATE_TX_IDLE,
};

/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// H4TL Environment context structure
struct h4tl_env_tag {
	/// pointer to External interface api
	const struct rwip_eif_api* ext_if;
	///Pointer to space reserved for received payload.
	uint8_t *curr_payl_buff;

	uint8_t *cur_buff;
	///TX callback for indicating the end of transfer
	void (*tx_callback)(void);
	/// Ensure that array is 32bits alligned
	/// Latest received message header, or used to receive a message alowing to exit from out of sync
	uint8_t rx_buf[RX_TMP_BUFF_SIZE];

	///Rx state - can be receiving message type, header, payload or error
	uint8_t rx_state;
	///Tx state - either transmitting, done or error.
	uint8_t tx_state;
	///Latest received message type: CMD/EVT/ACL.
	uint8_t rx_type;
	/// Transport layer type
	uint8_t tl_type;

	uint8_t tx_len;

	uint8_t rx_len;

	uint8_t *cur_rx_idx;

	uint8_t cur_tx_idx;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI table for correspondence between External Interface message type and header length.
static const uint8_t h4tl_msgtype2hdrlen[] = {
#if (HCI_TL_SUPPORT)
		[HCI_CMD_MSG_TYPE] = HCI_CMD_HDR_LEN,
		[HCI_ACL_MSG_TYPE] = HCI_ACL_HDR_LEN,
#if BT_EMB_PRESENT
#if (VOICE_OVER_HCI)
				[HCI_SYNC_MSG_TYPE] = HCI_SYNC_HDR_LEN,
#endif // BT_EMB_PRESENT
#endif // (VOICE_OVER_HCI)
				[HCI_EVT_MSG_TYPE] = HCI_EVT_HDR_LEN,
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
				[AHI_KERNEL_MSG_TYPE] = AHI_KERNEL_MSG_HDR_LEN,
#if defined(AOAHI_TL_SUPPORT)
				[AOAHI_AUDIO_MSG_TYPE] = AOAHI_HEADER_LEN,
#endif // defined(AOAHI_TL_SUPPORT)
#endif // (AHI_TL_SUPPORT)
			};

/// H4TL environment context
static struct h4tl_env_tag h4tl_env[H4TL_TYPE_MAX];
extern   uint32_t  ble_dut_flag;

/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
static void h4tl_rx_cmd_hdr_extract(struct h4tl_env_tag* env,
		struct hci_cmd_hdr * p_hdr);
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
static void h4tl_rx_acl_hdr_extract(struct h4tl_env_tag* env,
		struct hci_acl_hdr * p_hdr);
#endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
static void h4tl_rx_evt_hdr_extract(struct h4tl_env_tag* env, struct hci_evt_hdr * p_hdr);
#endif //((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
static void h4tl_cmd_hdr_rx_evt_handler(void);
static void h4tl_cmd_pld_rx_evt_handler(void);
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#endif // (HCI_TL_SUPPORT)
static void h4tl_read_start(struct h4tl_env_tag* env);
static void h4tl_read_hdr(struct h4tl_env_tag* env, uint8_t len);
static void h4tl_read_payl(struct h4tl_env_tag* env, uint16_t len);
static void h4tl_read_next_out_of_sync(struct h4tl_env_tag* env);
static void h4tl_out_of_sync(struct h4tl_env_tag* env);
static bool h4tl_out_of_sync_check(struct h4tl_env_tag* env);
static void h4tl_tx_done(struct h4tl_env_tag* env, uint8_t status);
static void h4tl_rx_done(struct h4tl_env_tag* env, uint8_t status);
static void h4tl_tx_evt_handler(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Local function : extracts command header components
 *
 * @param[in] env Environment of transport layer
 * @param[out]   p_hdr   Pointer to command header structure
 *****************************************************************************************
 */
static void h4tl_rx_cmd_hdr_extract(struct h4tl_env_tag* env,
		struct hci_cmd_hdr * p_hdr) {
	//extract command header:opcode, parameter length
	p_hdr->opcode = common_btohs(common_read16p(&(env->rx_buf[0])));
	p_hdr->parlen = env->rx_buf[HCI_CMD_OPCODE_LEN];
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Local function : extracts ACL header components
 *
 * @param[in] env Environment of transport layer
 * @param[out]   p_hdr   Pointer to ACL header structure
 *****************************************************************************************
 */
static void h4tl_rx_acl_hdr_extract(struct h4tl_env_tag* env,
		struct hci_acl_hdr * p_hdr) {
	// Extract ACL header: data length, connection handle and flags
	p_hdr->datalen = common_btohs(
			common_read16p(env->rx_buf + HCI_ACL_HDR_HDL_FLAGS_LEN));
	p_hdr->hdl_flags = common_btohs(common_read16p(env->rx_buf));
}
#endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
#if (BT_EMB_PRESENT)
#if (VOICE_OVER_HCI)
/**
 ****************************************************************************************
 * @brief Local function : extracts synchronous header components
 *
 * @param[in] env Environment of transport layer
 * @param[out]   p_hdr   Pointer to synchronous header structure
 *****************************************************************************************
 */
static void h4tl_rx_sync_hdr_extract(struct h4tl_env_tag* env, struct hci_sync_hdr * p_hdr)
{
	// Extract ACL header: data length, connection handle and flags
	p_hdr->data_total_len = *(env->rx_buf + HCI_SYNC_HDR_HDL_FLAGS_LEN);
	p_hdr->conhdl_flags = common_btohs(common_read16p(env->rx_buf));
}
#endif // (VOICE_OVER_HCI)
#endif // (BT_EMB_PRESENT)
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
/**
 ****************************************************************************************
 * @brief Local function : extracts event header components
 *
 * @param[in] env Environment of transport layer
 * @param[out]   p_hdr   Pointer to event header structure
 *****************************************************************************************
 */
static void h4tl_rx_evt_hdr_extract(struct h4tl_env_tag* env, struct hci_evt_hdr * p_hdr)
{
	//extract event header:code, parameter length
	p_hdr->code = env->rx_buf[0];
	p_hdr->parlen = env->rx_buf[HCI_EVT_CODE_LEN];
}
#endif //((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#endif // (HCI_TL_SUPPORT)
/**
 ******************************************************************************************
 * @brief Local function : places H4TL in RX_START state and sets the External Interface environment.
 *
 * @param[in] env Environment of transport layer
 ******************************************************************************************
 */
static void h4tl_read_start(struct h4tl_env_tag* env) {
	//Initialize External Interface in reception mode state
	//  env->rx_state = H4TL_STATE_RX_START;
	env->rx_state = H4TL_STATE_RX_IDLE;

	env->rx_len = 1;

	*env->cur_rx_idx = 0;
	//Set the External Interface environment to message type 1 byte reception

	env->ext_if->read(&(env->rx_buf[RX_TMP_BUFF_SIZE - 1]),
			H4TL_LOGICAL_CHANNEL_LEN, (rwip_eif_callback) &h4tl_rx_done, env);

	env->cur_buff = &(env->rx_buf[RX_TMP_BUFF_SIZE - 1]); //sean

    if(ble_dut_flag)
    {
        uart_clear_rxfifo();
    }
    else
    {
        while(uart_read_byte(UART2_PORT) != -1); // sean
    }

#if DEEP_SLEEP
	{
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
		uint8_t i;
		bool prevent_sleep = false;

		// check if one of transport layer is not in IDLE mode
		for(i =0; i < H4TL_TYPE_MAX; i++)
		{
			if(h4tl_env[i].rx_state != H4TL_STATE_RX_START)
			{
				prevent_sleep = true;
				break;
			}
		}

		if(!prevent_sleep)
#endif // ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
		{
			// No HCI reception is ongoing, so allow going to sleep
			//rwip_prevent_sleep_clear(RW_TL_RX_ONGOING);
			rwip_prevent_sleep_clear(RW_TL_RX_ONGOING);
		}
	}
#endif
}

/**
 ****************************************************************************************
 * @brief Local function : places H4TL in RX header state and sets the External Interface env.
 *
 * @param[in] env Environment of transport layer
 * @param[in] len Length of header to be received in the currently set buffer.
 *****************************************************************************************
 */
static void h4tl_read_hdr(struct h4tl_env_tag* env, uint8_t len) {
	//UART_PRINTF("h4tl_read_hdr len = %x\r\n",len);
	//change Rx state - wait for header next
	env->rx_state = H4TL_STATE_RX_HDR;

	env->rx_len = len;
	//set External Interface environment to header reception of len bytes
	env->ext_if->read(&env->rx_buf[0], len, (rwip_eif_callback) &h4tl_rx_done,
			env);

	env->cur_buff = &(env->rx_buf[0]);

	h4tl_rx_done(env, RWIP_EIF_STATUS_OK);

	//	h4tl_rx_done(env,RWIP_EIF_STATUS_OK);
#if DEEP_SLEEP
	// An HCI reception is ongoing
	//rwip_prevent_sleep_set(RW_TL_RX_ONGOING);
	rwip_prevent_sleep_set(RW_TL_RX_ONGOING);
#endif //DEEP_SLEEP
}

/**
 ******************************************************************************************
 * @brief Local function : places H4TL in RX payload state and request the External IF
 *
 * @param[in] env Environment of transport layer
 * @param[in] buf Buffer for payload reception
 * @param[in] len Length of payload to be received in the currently set buffer.
 ******************************************************************************************
 */
static void h4tl_read_payl(struct h4tl_env_tag* env, uint16_t len) {

	//UART_PRINTF("h4tl_read_payl len = %x\r\n",len);
	//change rx state to payload reception
	env->rx_state = H4TL_STATE_RX_PAYL;

    env->rx_len = len;
	//set External Interface environment to payload reception of len bytes
	env->ext_if->read(env->curr_payl_buff, len,
			(rwip_eif_callback) &h4tl_rx_done, env);

	env->cur_buff = env->curr_payl_buff;

	h4tl_rx_done(env, RWIP_EIF_STATUS_OK);

}

/**
 ******************************************************************************************
 * @brief Local function : places H4TL in RX_START_OUT_OF_SYNC state.
 *
 * @param[in] env Environment of transport layer
 ******************************************************************************************
 */
static void h4tl_read_next_out_of_sync(struct h4tl_env_tag* env) {
	//Set External Interface reception state to H4TL_STATE_RX_START_OUT_OF_SYNC
	env->rx_state = H4TL_STATE_RX_OUT_OF_SYNC;

	env->rx_len = 1;
	//Set the External Interface environment to 1 byte reception (at end of rx buffer)
	env->ext_if->read(&(env->rx_buf[RX_TMP_BUFF_SIZE - 1]),
			H4TL_LOGICAL_CHANNEL_LEN, (rwip_eif_callback) &h4tl_rx_done, env);
	
    env->cur_buff = &(env->rx_buf[RX_TMP_BUFF_SIZE - 1]); //zjw
}

/**
 *****************************************************************************************
 *@brief Static function handling External Interface out of synchronization detection.
 *
 * At External Interface reception, when packet indicator opcode of a command is not
 * recognized.
 *
 * @param[in] env Environment of transport layer
 *****************************************************************************************
 */
static void h4tl_out_of_sync(struct h4tl_env_tag* env) {
#if (HCI_TL_SUPPORT && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
#if(AHI_TL_SUPPORT)
	// check if communication is performed over embedded host
	if (!gapm_is_embedded_host())
#endif // (AHI_TL_SUPPORT && BLE_EMB_PRESENT)
	{
		// Allocate a message structure for hardware error event
		struct hci_hw_err_evt *evt =
				KERNEL_MSG_ALLOC(HCI_EVENT, 0, HCI_HW_ERR_EVT_CODE, hci_hw_err_evt);

		// Fill the parameter structure
		evt->hw_code = COMMON_ERROR_HW_UART_OUT_OF_SYNC;

		// Send the message
		hci_send_2_host(evt);
	}
#endif // (HCI_TL_SUPPORT && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
	// Initialize receive buffer
	memset(&(env->rx_buf[0]), 0, RX_TMP_BUFF_SIZE - 2);
	// keep last octet received
	env->rx_buf[RX_TMP_BUFF_SIZE - 2] = env->rx_buf[RX_TMP_BUFF_SIZE - 1];
	// Start reception of new packet ID
	//h4tl_read_next_out_of_sync(env);
	h4tl_read_start(env); // by zjw

#if DEEP_SLEEP
	// No HCI reception is ongoing, so allow going to sleep
	//rwip_prevent_sleep_clear(RW_TL_RX_ONGOING);
	rwip_prevent_sleep_clear(RW_TL_RX_ONGOING);
#endif // DEEP_SLEEP
}

/**
 ****************************************************************************************
 * @brief Check received byte in out of sync state
 *
 * This function is the algorithm to check that received byte stream in out of sync state
 * corresponds to HCI_reset command.
 *
 * Level of reception is incremented when bytes of HCI_reset_cmd are detected.
 *
 * @param[in] env Environment of transport layer
 *****************************************************************************************
 */
static bool h4tl_out_of_sync_check(struct h4tl_env_tag* env) {
	bool sync_ok = false;

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
	const uint8_t hci_reset_msg[HCI_RESET_MSG_LEN] = HCI_RESET_MSG_BUF;

	// check if valid HCI reset has been received
	if ((env->tl_type == H4TL_TYPE_HCI)
			&& (memcmp(&(hci_reset_msg[0]),
					&(env->rx_buf[RX_TMP_BUFF_SIZE - HCI_RESET_MSG_LEN]),
					HCI_RESET_MSG_LEN) == 0)) {
#if(AHI_TL_SUPPORT )
		// use an external host to control the controller
		gapm_set_embedded_host(false);
#endif // (AHI_TL_SUPPORT)
		// HCI processes the command
		hci_cmd_received(HCI_RESET_CMD_OPCODE, 0, NULL);
		sync_ok = true;
	}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
	// Check reset message
	if ((!sync_ok) && (env->tl_type == H4TL_TYPE_AHI)) {
		const uint8_t ahi_reset_msg[AHI_RESET_MSG_LEN] = AHI_RESET_MSG_BUF;

		// check if valid HCI reset has been received
		if (memcmp(&(ahi_reset_msg[0]),
				&(env->rx_buf[RX_TMP_BUFF_SIZE - AHI_RESET_MSG_LEN]),
				AHI_RESET_MSG_LEN) == 0) {
			// send reset message to application
			struct gapm_reset_cmd* reset =
					KERNEL_MSG_ALLOC(GAPM_RESET, TASK_GAPM, APP_MAIN_TASK, gapm_reset_cmd);
			reset->operation = GAPM_RESET;

#if(HCI_TL_SUPPORT  && BLE_EMB_PRESENT)
			// use an internal host to control the controller
			gapm_set_embedded_host(true);
#endif // (HCI_TL_SUPPORT && BLE_EMB_PRESENT)
			kernel_msg_send(reset);
			sync_ok = true;
		}
	}
	// check sync pattern
	if (!sync_ok) {
		const uint8_t ahi_sync_pattern[AHI_RESET_MSG_LEN] = AHI_RESET_MSG_BUF;

		// check if valid sync patternt has been received
		if (memcmp(&(ahi_sync_pattern[0]),
				&(env->rx_buf[RX_TMP_BUFF_SIZE - AHI_RESET_MSG_LEN]),
				AHI_RESET_MSG_LEN) == 0) {
			sync_ok = true;
		}
	}
#endif // (AHI_TL_SUPPORT)
	// sync not found, ensure that a packet will be received
	if (!sync_ok) {
		uint8_t i;

#if DEEP_SLEEP
		// An HCI reception is ongoing
		//rwip_prevent_sleep_set(RW_TL_RX_ONGOING);
		rwip_prevent_sleep_set(RW_TL_RX_ONGOING);
#endif // DEEP_SLEEP
		// shift received bytes left into rx buffer
		for (i = 0; i < (RX_TMP_BUFF_SIZE - 1); i++) {
			env->rx_buf[i] = env->rx_buf[i + 1];
		}
	}

	return sync_ok;
}

/**
 ****************************************************************************************
 * @brief Callback for TL to indicate the end of TX
 *
 * @param[in] env    Environment of transport layer
 * @param[in] status External Interface Tx status: ok or error.
 *****************************************************************************************
 */
static void h4tl_tx_done(struct h4tl_env_tag* env, uint8_t status) {
	// Sanity check: Transmission should always work
	ASSERT_ERR(status == RWIP_EIF_STATUS_OK);

	env->tx_state = H4TL_STATE_TX_DONE;

	// Defer the processing to ensure that it is done in background
	kernel_event_set(KERNEL_EVENT_H4TL_TX);
}

/**
 ****************************************************************************************
 * @brief Actions after External Interface TX.
 *****************************************************************************************
 */
static void h4tl_tx_evt_handler(void) {
	uint8_t i;

	// Clear the event
	kernel_event_clear(KERNEL_EVENT_H4TL_TX);

	for (i = 0; i < H4TL_TYPE_MAX; i++) {
		// Check if TX is done
		if (h4tl_env[i].tx_state == H4TL_STATE_TX_DONE) {
			// Go Back to Idle State
			h4tl_env[i].tx_state = H4TL_STATE_TX_IDLE;
			// Call callback
			if (h4tl_env[i].tx_callback != NULL) {
				h4tl_env[i].tx_callback();
			}
		}
	}

#if DEEP_SLEEP
	{
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
		bool on_going_tx = false;

		// check if one of transport layer is not in IDLE mode
		for(i =0; i < H4TL_TYPE_MAX; i++)
		{
			if(h4tl_env[i].tx_state != H4TL_STATE_TX_IDLE)
			{
				on_going_tx = true;
				break;
			}
		}

		if(!on_going_tx)
#endif // ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
		{
			// No HCI reception is ongoing, so allow going to sleep
			//rwip_prevent_sleep_clear(RW_TL_TX_ONGOING);
			rwip_prevent_sleep_clear(RW_TL_TX_ONGOING);
		}
	}
#endif
}

static void h4tl_recive_data(struct h4tl_env_tag* env) {

	uint8_t tmp_len = env->rx_len;

	uint8_t buf = 0;
	//UART_PRINTF("h4tl_recive_data rx_len = 0x%x,recive_data =  ",env->rx_len);	
	while (tmp_len--) {
        if(ble_dut_flag)
        {
            buf = Read_Uart_Buf();
        }
        else
        {
            buf = uart_read_byte(UART2_PORT); //fix 18/2/3 sean
        }
		//UART_PRINTF("%x ",buf);
		*env->cur_buff++ = buf; //Read_Uart_Buf();
	}
//UART_PRINTF("\r\n");

}

/**
 ****************************************************************************************
 * @brief Function called at each RX interrupt.
 *
 * @param[in] env    Environment of transport layer
 * @param[in] status External Interface RX status: ok or error
 *****************************************************************************************
 */
static void h4tl_rx_done(struct h4tl_env_tag* env, uint8_t status) {
	//UART_PRINTF("h4tl_rx_done  status = %x \r\n",status);

	if (env->rx_state == H4TL_STATE_RX_IDLE) {
		env->rx_state = H4TL_STATE_RX_START;
	}
	// detect that an event occurs on interface
	if (status != RWIP_EIF_STATUS_OK) {
		//detect External Interface RX error and handle accordingly
		if ((status == RWIP_EIF_STATUS_ERROR)
				|| (env->rx_state != H4TL_STATE_RX_START)) {
			// External Interface RX error -> enter in out of sync
			h4tl_out_of_sync(env);
		} else {
			// restart logical channel reception
			h4tl_read_start(env);
		}
	} else {
		//UART_PRINTF("01 env->rx_state = %x\r\n",env->rx_state);
		h4tl_recive_data(env);
		//check HCI state to see what was received
		switch (env->rx_state) {
		/* RECEIVE MESSAGE TYPE STATE*/
		case H4TL_STATE_RX_START: { //UART_PRINTF("02\r\n");
									// extract RX type
			env->rx_type = env->rx_buf[RX_TMP_BUFF_SIZE - 1];
			//UART_PRINTF("env->rx_type = 0x%x,RX_TMP_BUFF_SIZE = %x\r\n",env->rx_type,RX_TMP_BUFF_SIZE);
			// Check received packet indicator
			switch (env->rx_type) {
#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
			case HCI_CMD_MSG_TYPE: //0x01
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
			case HCI_ACL_MSG_TYPE: //0x02
#if BT_EMB_PRESENT
#if (VOICE_OVER_HCI)
			case HCI_SYNC_MSG_TYPE: // 0x03
#endif // (VOICE_OVER_HCI)
#endif //BT_EMB_PRESENT
#if BLE_HOST_PRESENT
			case HCI_EVT_MSG_TYPE: //  0x04
#endif //BLE_HOST_PRESENT
			{
				/*
				 if((env->tl_type != H4TL_TYPE_HCI)
				 #if(AHI_TL_SUPPORT && BLE_EMB_PRESENT)
				 // check if communication is performed over embedded host
				 || (gapm_is_embedded_host())
				 #endif // (AHI_TL_SUPPORT && BLE_EMB_PRESENT)
				 )
				 {
				 // Incorrect packet indicator -> enter in out of sync

				 h4tl_out_of_sync(env);
				 }
				 else
				 */
				{
					//UART_PRINTF("03\r\n");
					//change state to header reception
					h4tl_read_hdr(env, h4tl_msgtype2hdrlen[env->rx_type]);
				}
			}
				break;
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
#if defined(AOAHI_TL_SUPPORT)
				case AOAHI_AUDIO_MSG_TYPE:
#endif // defined(AOAHI_TL_SUPPORT)
			case AHI_KERNEL_MSG_TYPE: {
				/*      if((env->tl_type != H4TL_TYPE_AHI)
				 #if(HCI_TL_SUPPORT && BLE_EMB_PRESENT)
				 // check if communication is performed over embedded host
				 || (!gapm_is_embedded_host())
				 #endif // (AHI_TL_SUPPORT && BLE_EMB_PRESENT)
				 )
				 {
				 // Incorrect packet indicator -> enter in out of sync
				 h4tl_out_of_sync(env);
				 }
				 else
				 */
				{
					//change state to header reception
					h4tl_read_hdr(env, h4tl_msgtype2hdrlen[env->rx_type]);
				}
			}
				break;
#endif // (AHI_TL_SUPPORT)
			default: {
				// Incorrect packet indicator -> enter in out of sync
				h4tl_out_of_sync(env);
			}
				break;
			}
		}
			break;
			/* RECEIVE MESSAGE TYPE STATE END*/

			/* RECEIVE HEADER STATE*/
		case H4TL_STATE_RX_HDR: // 0x02
		{
			//UART_PRINTF("H4TL_STATE_RX_HDR \r\n");
			switch (env->rx_type) {
#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
			//Command Header reception
			case HCI_CMD_MSG_TYPE: {
				//UART_PRINTF("06 \r\n");
				// Defer the processing to ensure that it is done in background
				kernel_event_set(KERNEL_EVENT_H4TL_CMD_HDR_RX);
			}
				break;
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
			case HCI_ACL_MSG_TYPE: {
				struct hci_acl_hdr* p_hdr = (struct hci_acl_hdr*) env->rx_buf;

				// Extract the ACL header components
				h4tl_rx_acl_hdr_extract(env, p_hdr);

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
				// HCI allocate a buffer for data reception (HL rx or LL tx)
				env->curr_payl_buff = hci_acl_tx_data_alloc(p_hdr->hdl_flags,
						p_hdr->datalen);
#elif BLE_HOST_PRESENT
				// HCI allocate a buffer for data reception (HL rx or LL tx)
				env->curr_payl_buff = hci_acl_rx_data_alloc(p_hdr->hdl_flags, p_hdr->datalen);
#else
				ASSERT_ERR(0);
#endif //BLE_EMB_PRESENT/BLE_HOST_PRESENT
				// Check data length
				if (env->curr_payl_buff == NULL) {
					// Incorrect payload size -> enter in out of sync
					h4tl_out_of_sync(env);
				} else { //UART_PRINTF("07\r\n");
						 //change HCI rx state to payload reception
					h4tl_read_payl(env, p_hdr->datalen);
				}
			}
				break;
#endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
#if (BT_EMB_PRESENT)
#if (VOICE_OVER_HCI)
				case HCI_SYNC_MSG_TYPE:
				{
					struct hci_sync_hdr* p_hdr = (struct hci_sync_hdr*) env->rx_buf;

					// Extract the Synchronous header components
					h4tl_rx_sync_hdr_extract(env, p_hdr);

					// HCI allocate a buffer for data reception
					env->curr_payl_buff = hci_sync_tx_data_alloc(p_hdr->conhdl_flags, p_hdr->data_total_len);

					// Check data length
					if (env->curr_payl_buff == NULL)
					{
						// Incorrect payload size -> enter in out of sync
						h4tl_out_of_sync(env);
					}
					else
					{
						//change HCI rx state to payload reception
						h4tl_read_payl(env, p_hdr->data_total_len);
					}
				}
				break;
#endif // (VOICE_OVER_HCI)
#endif // (BT_EMB_PRESENT)
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
				//Event Header reception
				case HCI_EVT_MSG_TYPE:
				{
					struct hci_evt_hdr* p_hdr = (struct hci_evt_hdr*) env->rx_buf;

					// Extract the event header components
					h4tl_rx_evt_hdr_extract(env, p_hdr);

					// Check if the event has parameters
					if(p_hdr->parlen == 0)
					{
						// HCI processes the event
						hci_evt_received(p_hdr->code, p_hdr->parlen, NULL);

						//change hci rx state to message type reception
						h4tl_read_start(env);
					}
					else
					{
						// Allocate memory buffer for receiving the payload
						env->curr_payl_buff = (uint8_t*) kernel_malloc(p_hdr->parlen, KERNEL_MEM_KERNEL_MSG);

						if (env->curr_payl_buff != NULL)
						{
							//change HCI rx state to payload reception
							h4tl_read_payl(env, p_hdr->parlen);
						}
						else
						{
							// Problem in the RX buffer allocation -> enter in out of sync
							h4tl_out_of_sync(env);
						}
					}
				}
				break;
#endif //((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
			case AHI_KERNEL_MSG_TYPE: {
				struct ahi_kemsghdr * p_msg_hdr =
						(struct ahi_kemsghdr *) (&env->rx_buf[0]);
				struct kernel_msg* rx_msg;

				// retrieve destination task
				kernel_task_id_t dest = gapm_get_task_from_id(p_msg_hdr->dest_id);

				// destination task is unknown
				if (KERNEL_TYPE_GET(dest) == TASK_BLE_NONE) {
					// Allocate the kernel message to GAPM with source identifier = destination task ID
					rx_msg = kernel_param2msg(
							kernel_msg_alloc(GAPM_UNKNOWN_TASK_MSG, TASK_GAPM,
									p_msg_hdr->dest_id, p_msg_hdr->param_len));

					// Store identifier in parameter length field
					rx_msg->param_len = p_msg_hdr->id;
				}
				// destination task is known
				else {
					// Update source and dest task identifier with task numbers
					p_msg_hdr->src_id = gapm_get_task_from_id(
							p_msg_hdr->src_id);

					// Allocate the kernel message
					rx_msg = kernel_param2msg(
							kernel_msg_alloc(p_msg_hdr->id, dest, p_msg_hdr->src_id,
									p_msg_hdr->param_len));
				}

				//no params
				if (p_msg_hdr->param_len == 0) {
					// Send message directly
					kernel_msg_send(kernel_msg2param(rx_msg));

					// Restart a new packet reception
					h4tl_read_start(env);
				} else {
					env->curr_payl_buff = (uint8_t*) &(rx_msg->param[0]);
					// Start payload reception
					h4tl_read_payl(env, p_msg_hdr->param_len);
				}
			}
				break;
#if defined(AOAHI_TL_SUPPORT)
				case AOAHI_AUDIO_MSG_TYPE:
				{
					struct aoahi_pkt_hdr* p_hdr = (struct aoahi_pkt_hdr*) env->rx_buf;

					// Allocate a buffer for data reception
					env->curr_payl_buff = aoahi_audio_in_buf_alloc(p_hdr);

					// Check data length
					if (env->curr_payl_buff == NULL)
					{
						// Incorrect payload size -> enter in out of sync
						h4tl_out_of_sync(env);
					}
					else
					{
						//change HCI rx state to payload reception
						h4tl_read_payl(env, p_hdr->length);
					}
				}break;
#endif // defined(AOAHI_TL_SUPPORT)
#endif // (AHI_TL_SUPPORT)
			default: {
				ASSERT_INFO(0, env->rx_type, env->tl_type);
			}
				break;
			} //end switch
		}
			break;
			/* RECEIVE HEADER STATE END*/

			/* RECEIVE PAYLOAD STATE */
		case H4TL_STATE_RX_PAYL: {
			//UART_PRINTF("H4TL_STATE_RX_PAYL \r\n");
			switch (env->rx_type) {
#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
			case HCI_CMD_MSG_TYPE: {
				//UART_PRINTF("kernel_event_set(KERNEL_EVENT_H4TL_CMD_PLD_RX);\r\n");
				// Defer the processing to ensure that it is done in background
				kernel_event_set(KERNEL_EVENT_H4TL_CMD_PLD_RX);
			}
				break;
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
			case HCI_ACL_MSG_TYPE: {

#if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT))
				struct hci_acl_hdr* p_hdr = (struct hci_acl_hdr *) env->rx_buf;
				// HCI processes the data
				hci_acl_tx_data_received(p_hdr->hdl_flags, p_hdr->datalen,
						env->curr_payl_buff);
#elif (BLE_HOST_PRESENT && HCI_BLE_CON_SUPPORT)
				struct hci_acl_hdr* p_hdr = (struct hci_acl_hdr *) env->rx_buf;
				// HCI processes the data
				hci_acl_rx_data_received(p_hdr->hdl_flags, p_hdr->datalen, env->curr_payl_buff);
#else
				ASSERT_ERR(0);
#endif //BLE_EMB_PRESENT/BLE_HOST_PRESENT/BT_EMB_PRESENT
				//change hci rx state to message type reception - common to all types
				h4tl_read_start(env);
			}
				break;

#if (BT_EMB_PRESENT)
#if (VOICE_OVER_HCI)
				case HCI_SYNC_MSG_TYPE:
				{
					struct hci_sync_hdr* p_hdr = (struct hci_sync_hdr *) env->rx_buf;
					// HCI processes the data
					hci_sync_tx_data_received(p_hdr->conhdl_flags, p_hdr->data_total_len, env->curr_payl_buff);

					//change hci rx state to message type reception - common to all types
					h4tl_read_start(env);
				}
				break;
#endif // (VOICE_OVER_HCI)
#endif // (BT_EMB_PRESENT)
#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
				case HCI_EVT_MSG_TYPE:
				{
					struct hci_evt_hdr* p_hdr = (struct hci_evt_hdr *) env->rx_buf;

					// HCI processes the event
					hci_evt_received(p_hdr->code, p_hdr->parlen, env->curr_payl_buff);

					if (env->curr_payl_buff != NULL)
					{
						// Free payload buffer
						kernel_free(env->curr_payl_buff);

						// Clear current payload buffer pointer
						env->curr_payl_buff = NULL;
					}

					//change hci rx state to message type reception - common to all types
					h4tl_read_start(env);
				}
				break;
#endif //((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
			case AHI_KERNEL_MSG_TYPE: {
				// Send the kernel message
				kernel_msg_send(env->curr_payl_buff);

				// Restart a new packet reception
				h4tl_read_start(env);
			}
				break;
#if defined(AOAHI_TL_SUPPORT)
				case AOAHI_AUDIO_MSG_TYPE:
				{
					struct aoahi_pkt_hdr* p_hdr = (struct aoahi_pkt_hdr*) env->rx_buf;

					// Send the kernel message
					aoahi_audio_in_buf_send(p_hdr, env->curr_payl_buff);

					// Restart a new packet reception
					h4tl_read_start(env);
				}
				break;
#endif // defined(AOAHI_TL_SUPPORT)
#endif // (AHI_TL_SUPPORT)
			default: {
				ASSERT_INFO(0, env->rx_type, env->tl_type);
			}
				break;
			}
		}
			break;
			/* RECEIVE PAYLOAD STATE END*/

			/* RX OUT OF SYNC STATE */
		case H4TL_STATE_RX_OUT_OF_SYNC: {
			// Check received byte
			if (h4tl_out_of_sync_check(env)) {
				// sync found, return to normal mode
				h4tl_read_start(env);
			} else {
				// Start a new byte reception
				h4tl_read_next_out_of_sync(env);
			}
		}
			break;
			/* RX OUT OF SYNC STATE END*/

			/* DEFAULT STATE */
		default: {
			ASSERT_ERR(0);
		}
			break;
			/* DEFAULT END*/

		}
		/* STATE SWITCH END */
	}
}

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Actions after reception of HCI command header
 *****************************************************************************************
 */
static void h4tl_cmd_hdr_rx_evt_handler(void) {

	struct h4tl_env_tag* env = &(h4tl_env[H4TL_TYPE_HCI]);
	struct hci_cmd_hdr* p_hdr = (struct hci_cmd_hdr*) env->rx_buf;

	// Clear the event
	kernel_event_clear(KERNEL_EVENT_H4TL_CMD_HDR_RX);

	//UART_PRINTF("h4tl_cmd_pld_rx_evt_handler \r\n");
	// Extract the command header components
	h4tl_rx_cmd_hdr_extract(env, p_hdr);

	//UART_PRINTF("p_hdr->parlen = %x\r\n",p_hdr->parlen);
	// Check if the command has parameters
	if (p_hdr->parlen == 0) {
		// HCI processes the command
		hci_cmd_received(p_hdr->opcode, p_hdr->parlen, NULL);

		//change hci rx state to message type reception
		h4tl_read_start(env);
	} else {
		// Check received parameter size
		if (p_hdr->parlen > hci_cmd_get_max_param_size(p_hdr->opcode)) {
			// Incorrect header -> enter in out of sync
			h4tl_out_of_sync(env);
		} else {
			// Allocate memory buffer for receiving the payload
			env->curr_payl_buff = (uint8_t*) kernel_malloc(p_hdr->parlen,
					KERNEL_MEM_KERNEL_MSG);

			if (env->curr_payl_buff != NULL) {
				//change HCI rx state to payload reception
				h4tl_read_payl(env, p_hdr->parlen);

			} else {
				// Problem in the RX buffer allocation -> enter in out of sync
				h4tl_out_of_sync(env);
			}
		}
	}
}

/**
 ****************************************************************************************
 * @brief Actions after reception of HCI command payload
 *****************************************************************************************
 */
static void h4tl_cmd_pld_rx_evt_handler(void) {
	struct h4tl_env_tag* env = &(h4tl_env[H4TL_TYPE_HCI]);
	struct hci_cmd_hdr* p_hdr = (struct hci_cmd_hdr*) env->rx_buf;
	//UART_PRINTF("h4tl_cmd_pld_rx_evt_handler \r\n");
	// Clear the event
	kernel_event_clear(KERNEL_EVENT_H4TL_CMD_PLD_RX);

	// HCI processes the command
	hci_cmd_received(p_hdr->opcode, p_hdr->parlen, env->curr_payl_buff);

	if (env->curr_payl_buff != NULL) {
		// Free payload buffer
		kernel_free(env->curr_payl_buff);

		// Clear current payload buffer pointer
		env->curr_payl_buff = NULL;
	}

	//change hci rx state to message type reception - common to all types
	h4tl_read_start(env);
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#endif //(HCI_TL_SUPPORT)
/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void h4tl_init(uint8_t tl_type, const struct rwip_eif_api* eif,
		uint8_t *cur_read_idx) {


	// Store external interface API
	h4tl_env[tl_type].ext_if = eif;

	// Enable External Interface
	h4tl_env[tl_type].ext_if->flow_on();

	//initialize tx state
	h4tl_env[tl_type].tx_state = H4TL_STATE_TX_IDLE;
	h4tl_env[tl_type].tl_type = tl_type;

	//initialize rx buf idx
	h4tl_env[tl_type].cur_rx_idx = cur_read_idx; //(uint8_t *)(&cur_read_buf_idx);

	// Create HCI TX kernel event
	kernel_event_callback_set(KERNEL_EVENT_H4TL_TX, &h4tl_tx_evt_handler);



#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
	kernel_event_callback_set(KERNEL_EVENT_H4TL_CMD_HDR_RX,
			&h4tl_cmd_hdr_rx_evt_handler);
	kernel_event_callback_set(KERNEL_EVENT_H4TL_CMD_PLD_RX,
			&h4tl_cmd_pld_rx_evt_handler);
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#endif // (HCI_TL_SUPPORT)
	//start External Interface reception

	h4tl_read_start(&(h4tl_env[tl_type]));

}

void h4tl_write(uint8_t type, uint8_t *buf, uint16_t len,
		void (*tx_callback)(void)) {

	//UART_PRINTF("h4tl_write \r\n");
	uint8_t tl_type = 0;
#if DEEP_SLEEP
	// An HCI transmission is ongoing - The bit has to be set prior to call to write
	// as this function may call h4tl_tx_done immediately
	rwip_prevent_sleep_set(RW_TL_TX_ONGOING);
	//	rom_env.rwip_prevent_sleep_set(RW_TL_TX_ONGOING);
#endif // DEEP_SLEEP
	//pack event type message (External Interface header)
	buf -= HCI_TRANSPORT_HDR_LEN;
	*buf = type;

	// retrieve Transport Layer Type
	switch (type) {
#if (AHI_TL_SUPPORT)
#if defined(AOAHI_TL_SUPPORT)
	case AOAHI_AUDIO_MSG_TYPE:
#endif // defined(AOAHI_TL_SUPPORT)
	case AHI_KERNEL_MSG_TYPE: {
		tl_type = H4TL_TYPE_AHI;
	}
		break;
#endif // (AHI_TL_SUPPORT)
#if (HCI_TL_SUPPORT)
	case HCI_CMD_MSG_TYPE:
	case HCI_ACL_MSG_TYPE:
	case HCI_SYNC_MSG_TYPE:
	case HCI_EVT_MSG_TYPE: {
		tl_type = H4TL_TYPE_HCI;
	}
		break;
#endif // (HCI_TL_SUPPORT)
	default: {
		ASSERT_INFO(0, type, H4TL_TYPE_MAX);
	}
		break;
	}

	h4tl_env[tl_type].tx_callback = tx_callback;
	//go to start tx state
	h4tl_env[tl_type].tx_state = H4TL_STATE_TX_ONGOING;
	h4tl_env[tl_type].ext_if->write(buf, len + HCI_TRANSPORT_HDR_LEN,
			(rwip_eif_callback) &h4tl_tx_done, &(h4tl_env[tl_type]));
}

#if DEEP_SLEEP
void h4tl_start(void)
{
	uint8_t i;

	for (i = 0; i < H4TL_TYPE_MAX; i++)
	{
		// Enable External Interface flow
		h4tl_env[i].ext_if->flow_on();
	}
}

bool h4tl_stop(void)
{
	bool res = true;
	uint8_t i;
	for (i = 0; (i < H4TL_TYPE_MAX) && (res); i++)
	{
		// Disable External Interface flow
		res = h4tl_env[i].ext_if->flow_off();
	}

	// If Flow cannot be stopped on all interfaces, restart flow on interfaces stopped
	if(!res)
	{
		uint8_t j;
		for (j = 0; (j < i); j++)
		{
			// Enable External Interface flow
			h4tl_env[j].ext_if->flow_on();
		}
	}

	return true; //(res);
}
#endif //DEEP_SLEEP
#endif //(H4TL_SUPPORT)
/// @} H4TL
