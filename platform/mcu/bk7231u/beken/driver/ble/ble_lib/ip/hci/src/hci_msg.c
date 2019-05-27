/**
 ****************************************************************************************
 *
 * @file hci_msg.c
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
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

/*
 * If Transport layer is present, some commands/events parameters with special format (variable content) needs
 * specific function for packing or unpacking. The specific function may support packing only, unpacking only, or both.
 *
 * Function types:
 *   - pkupk => functions used to pack or unpack parameters (depending on the Host<->Controller direction supported)
 *   - upk   => functions used to unpack parameters
 *   - pk    => functions used to pack parameters
 *
 * The support of packing or unpacking depends on the Host<->Controller direction supported by each commands/event:
 *  - for commands supported in LE:
 *      - Split-Host configuration          -> command parameters are packed / return parameters are unpacked
 *      - Split-Emb or full configuration   -> command parameters are unpacked / return parameters are packed
 *  - for events supported in LE:
 *      - Split-Host configuration          -> event parameters are unpacked
 *      - Split-Emb or full configuration   -> event parameters are packed
 *  - for commands supported in BT only:
 *                                          -> command parameters are unpacked / return parameters are packed
 *  - for events supported in BT only:
 *                                          -> event parameters are packed
 */

#if (HCI_TL_SUPPORT)
static uint8_t hci_host_nb_cmp_pkts_cmd_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#if (BT_EMB_PRESENT)
static uint8_t hci_set_evt_filter_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_wr_stored_lk_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_wr_curr_iac_lap_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#if CSB_SUPPORT
static uint8_t hci_set_con_slv_bcst_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_con_slv_bcst_rec_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //CSB_SUPPORT
#if RW_MWS_COEX
static uint8_t hci_set_external_frame_config_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_set_mws_scan_freq_table_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_set_mws_pattern_config_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_get_mws_transport_layer_config_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //RW_MWS_COEX
#if RW_DEBUG
static uint8_t hci_dbg_bt_send_lmp_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //RW_DEBUG
#endif //(BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
static uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_dir_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#if (RW_DEBUG &&  (BLE_EMB_PRESENT || BT_EMB_PRESENT))
uint8_t hci_dbg_wr_mem_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
uint8_t hci_dbg_wr_flash_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
uint8_t hci_dbg_wr_par_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
uint8_t hci_dbg_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
uint8_t hci_dbg_assert_err_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //(RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
#if CRYPTO_UT
static uint8_t hci_dbg_test_crypto_func_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //CRYPTO_UT
#endif //(HCI_TL_SUPPORT)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// HCI command descriptors (OGF link control)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_lk_ctrl[] =
{
    #if BT_EMB_PRESENT
    CMD(INQ                        , BT_MNG         , 0   , PK_GEN_GEN, 5 , "3BBB"    , NULL  ),
    CMD(INQ_CANCEL                 , BT_MNG         , 0   , PK_GEN_GEN, 0 , NULL      , "B"   ),
    CMD(PER_INQ_MODE               , BT_MNG         , 0   , PK_GEN_GEN, 9 , "HH3BBB"  , "B"   ),
    CMD(EXIT_PER_INQ_MODE          , BT_MNG         , 0   , PK_GEN_GEN, 0 , NULL      , "B"   ),
    CMD(CREATE_CON                 , BT_MNG         , 0   , PK_GEN_GEN, 13, "6BHBBHB" , NULL  ),
    CMD(CREATE_CON_CANCEL          , BT_MNG         , 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(ACCEPT_CON_REQ             , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 7 , "6BB"     , NULL  ),
    CMD(REJECT_CON_REQ             , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 7 , "6BB"     , NULL  ),
    CMD(LK_REQ_REPLY               , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 22, "6B16B"   , "B6B" ),
    CMD(LK_REQ_NEG_REPLY           , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(PIN_CODE_REQ_REPLY         , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 23, "6BB16B"  , "B6B" ),
    CMD(PIN_CODE_REQ_NEG_REPLY     , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(CHG_CON_PKT_TYPE           , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 4 , "HH"      , NULL  ),
    CMD(AUTH_REQ                   , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 2 , "H"       , NULL  ),
    CMD(SET_CON_ENC                , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 3 , "HB"      , NULL  ),
    CMD(CHG_CON_LK                 , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 2 , "H"       , NULL  ),
    CMD(MASTER_LK                  , BT_BCST        , 0   , PK_GEN_GEN, 1 , "B"       , NULL  ),
    CMD(REM_NAME_REQ               , BT_MNG         , 0   , PK_GEN_GEN, 10, "6BBBH"   , NULL  ),
    CMD(REM_NAME_REQ_CANCEL        , BT_MNG         , 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(RD_REM_EXT_FEATS           , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 3 , "HB"      , NULL  ),
    CMD(RD_CLK_OFF                 , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 2 , "H"       , NULL  ),
    CMD(RD_LMP_HDL                 , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 2 , "H"       , "BHBL"),
    CMD(SETUP_SYNC_CON             , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 17, "HLLHHBH" , NULL  ),
    CMD(ACCEPT_SYNC_CON_REQ        , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 21, "6BLLHHBH", NULL  ),
    CMD(REJECT_SYNC_CON_REQ        , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 7 , "6BB"     , NULL  ),
    CMD(IO_CAP_REQ_REPLY           , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 9 , "6BBBB"   , "B6B" ),
    CMD(USER_CFM_REQ_REPLY         , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(USER_CFM_REQ_NEG_REPLY     , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(USER_PASSKEY_REQ_REPLY     , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 10, "6BL"     , "B6B" ),
    CMD(USER_PASSKEY_REQ_NEG_REPLY , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(REM_OOB_DATA_REQ_REPLY     , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 38, "6B16B16B", "B6B" ),
    CMD(REM_OOB_DATA_REQ_NEG_REPLY , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(IO_CAP_REQ_NEG_REPLY       , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 7 , "6BB"     , "B6B" ),
    CMD(ENH_SETUP_SYNC_CON         , BT_CTRL_CONHDL , 0   , PK_GEN_GEN, 59, "HLL5B5BHHLL5B5BHHBBBBBBBBHHB" , NULL),
    CMD(ENH_ACCEPT_SYNC_CON        , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 63, "6BLL5B5BHHLL5B5BHHBBBBBBBBHHB", NULL),
    CMD(REM_OOB_EXT_DATA_REQ_REPLY , BT_CTRL_BD_ADDR, 0   , PK_GEN_GEN, 70, "6B16B16B16B16B", "B6B" ),
    #if CSB_SUPPORT
    CMD(TRUNC_PAGE                 , BT_MNG         , 0   , PK_GEN_GEN, 9 , "6BBH"    , NULL  ),
    CMD(TRUNC_PAGE_CAN             , BT_MNG         , 0   , PK_GEN_GEN, 6 , "6B"      , "B6B" ),
    CMD(SET_CON_SLV_BCST           , BT_BCST        , 0   , PK_GEN_GEN, 11, "BBBHHHH" , "BBH" ),
    CMD(SET_CON_SLV_BCST_REC       , BT_BCST        , 0   , PK_GEN_GEN, 34, "B6BBHLLHBBH10B", "B6BB"),
    CMD(START_SYNC_TRAIN           , BT_BCST        , 0   , PK_GEN_GEN, 0 , NULL      , NULL  ),
    CMD(REC_SYNC_TRAIN             , BT_BCST        , 0   , PK_GEN_GEN, 12, "6BHHH"   , NULL  ),
    #endif //CSB_SUPPORT
    #endif// BT_EMB_PRESENT
    CMD(RD_REM_SUPP_FEATS          , CTRL        , HL_CTRL, PK_GEN_GEN, 2 , "H"       , NULL  ),
    CMD(DISCONNECT                 , CTRL        , HL_CTRL, PK_GEN_GEN, 3 , "HB"      , NULL  ),
    CMD(RD_REM_VER_INFO            , CTRL        , HL_CTRL, PK_GEN_GEN, 2 , "H"       , NULL  ),
};

///// HCI command descriptors (OGF link policy)
#if BT_EMB_PRESENT
const struct hci_cmd_desc_tag hci_cmd_desc_tab_lk_pol[] =
{
    CMD(SNIFF_MODE          , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 10, "HHHHH"   , NULL ),
    CMD(EXIT_SNIFF_MODE     , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2 , "H"       , NULL ),
    CMD(QOS_SETUP           , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 20, "HBBLLLL" , NULL ),
    CMD(ROLE_DISCOVERY      , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2 , "H"       , "BHB"),
    CMD(SWITCH_ROLE         , BT_CTRL_BD_ADDR, 0, PK_GEN_GEN, 7 , "6BB"     , NULL ),
    CMD(RD_LINK_POL_STG     , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2 , "H"       , "BHH"),
    CMD(WR_LINK_POL_STG     , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 4 , "HH"      , "BH" ),
    CMD(RD_DFT_LINK_POL_STG , BT_MNG         , 0, PK_GEN_GEN, 0 , NULL      , "BH" ),
    CMD(WR_DFT_LINK_POL_STG , BT_MNG         , 0, PK_GEN_GEN, 2 , "H"       , "B"  ),
    CMD(FLOW_SPEC           , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 21, "HBBBLLLL", NULL ),
    CMD(SNIFF_SUB           , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 8 , "HHHH"    , "BH" ),
};
#endif// BT_EMB_PRESENT

/// HCI command descriptors (OGF controller and baseband)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_ctrl_bb[] =
{
    CMD(SET_EVT_MASK                 , MNG      , HL_MNG , PK_GEN_GEN, 8  , "8B"   , "B"      ),
    CMD(RESET                        , MNG      , HL_MNG , PK_GEN_GEN, 0  , NULL   , "B"      ),
    CMD(FLUSH                        , CTRL     , HL_CTRL, PK_GEN_GEN, 2  , "H"    , "BH"     ),
    CMD(RD_TX_PWR_LVL                , CTRL     , HL_CTRL, PK_GEN_GEN, 3  , "HB"   , "BHB"    ),
    CMD(SET_CTRL_TO_HOST_FLOW_CTRL   , MNG      , HL_MNG , PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(HOST_BUF_SIZE                , MNG      , HL_MNG , PK_GEN_GEN, 7  , "HBHH" , "B"      ),
    CMD(HOST_NB_CMP_PKTS             , MNG      , HL_MNG , PK_SPE_GEN, 30 , &hci_host_nb_cmp_pkts_cmd_pkupk, "B"  ),
    #if !(BLE_QUALIF)
    CMD(RD_AUTH_PAYL_TO              , CTRL     , HL_CTRL, PK_GEN_GEN, 2  , "H"    , "BHH"    ),
    CMD(WR_AUTH_PAYL_TO              , CTRL     , HL_CTRL, PK_GEN_GEN, 4  , "HH"   , "BH"     ),
    CMD(SET_EVT_MASK_PAGE_2          , MNG      , HL_MNG , PK_GEN_GEN, 8  , "8B"   , "B"      ),
    #endif
    #if BT_EMB_PRESENT
    CMD(SET_EVT_FILTER               , BT_MNG         , 0, PK_SPE_GEN, 9  , &hci_set_evt_filter_cmd_upk, "B" ),
    CMD(RD_PIN_TYPE                  , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_PIN_TYPE                  , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_STORED_LK                 , BT_MNG         , 0, PK_GEN_GEN, 7  , "6BB"  , "BHH"    ),
    CMD(WR_STORED_LK                 , BT_MNG         , 0, PK_SPE_GEN, 243, &hci_wr_stored_lk_cmd_upk, "BB" ),
    CMD(DEL_STORED_LK                , BT_MNG         , 0, PK_GEN_GEN, 7  , "6BB"  , "BH"     ),
    CMD(WR_LOCAL_NAME                , BT_MNG         , 0, PK_GEN_GEN, 248, "248B" , "B"      ),
    CMD(RD_LOCAL_NAME                , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "B248B"  ),
    CMD(RD_CON_ACCEPT_TO             , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BH"     ),
    CMD(WR_CON_ACCEPT_TO             , BT_MNG         , 0, PK_GEN_GEN, 2  , "H"    , "B"      ),
    CMD(RD_PAGE_TO                   , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BH"     ),
    CMD(WR_PAGE_TO                   , BT_MNG         , 0, PK_GEN_GEN, 2  , "H"    , "B"      ),
    CMD(RD_SCAN_EN                   , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_SCAN_EN                   , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_PAGE_SCAN_ACT             , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BHH"    ),
    CMD(WR_PAGE_SCAN_ACT             , BT_MNG         , 0, PK_GEN_GEN, 4  , "HH"   , "B"      ),
    CMD(RD_INQ_SCAN_ACT              , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BHH"    ),
    CMD(WR_INQ_SCAN_ACT              , BT_MNG         , 0, PK_GEN_GEN, 4  , "HH"   , "B"      ),
    CMD(RD_AUTH_EN                   , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_AUTH_EN                   , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_CLASS_OF_DEV              , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "B3B"    ),
    CMD(WR_CLASS_OF_DEV              , BT_MNG         , 0, PK_GEN_GEN, 3  , "3B"   , "B"      ),
    #if (MAX_NB_SYNC > 0)
    CMD(RD_VOICE_STG                 , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BH"     ),
    CMD(WR_VOICE_STG                 , BT_MNG         , 0, PK_GEN_GEN, 2  , "H"    , "B"      ),
    #endif // (MAX_NB_SYNC > 0)
    CMD(RD_AUTO_FLUSH_TO             , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2  , "H"    , "BHH"    ),
    CMD(WR_AUTO_FLUSH_TO             , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 4  , "HH"   , "BH"     ),
    CMD(RD_NB_BDCST_RETX             , BT_BCST        , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_NB_BDCST_RETX             , BT_BCST        , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_SYNC_FLOW_CTRL_EN         , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_SYNC_FLOW_CTRL_EN         , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_LINK_SUPV_TO              , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2  , "H"    , "BHH"    ),
    CMD(WR_LINK_SUPV_TO              , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 4  , "HH"   , "BH"     ),
    CMD(RD_NB_SUPP_IAC               , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(RD_CURR_IAC_LAP              , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB3B"   ),
    CMD(WR_CURR_IAC_LAP              , BT_MNG         , 0, PK_SPE_GEN, 253, &hci_wr_curr_iac_lap_cmd_upk, "B" ),
    CMD(SET_AFH_HOST_CH_CLASS        , BT_MNG         , 0, PK_GEN_GEN, 10 , "10B"  , "B"      ),
    CMD(RD_INQ_SCAN_TYPE             , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_INQ_SCAN_TYPE             , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_INQ_MODE                  , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_INQ_MODE                  , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_PAGE_SCAN_TYPE            , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_PAGE_SCAN_TYPE            , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_AFH_CH_ASSESS_MODE        , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_AFH_CH_ASSESS_MODE        , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_EXT_INQ_RSP               , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB240B" ),
    CMD(WR_EXT_INQ_RSP               , BT_MNG         , 0, PK_GEN_GEN, 241, "B240B", "B"      ),
    CMD(REFRESH_ENC_KEY              , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 2  , "H"    , NULL     ),
    CMD(RD_SP_MODE                   , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_SP_MODE                   , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_LOC_OOB_DATA              , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "B16B16B"),
    CMD(RD_INQ_RSP_TX_PWR_LVL        , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_INQ_TX_PWR_LVL            , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_DFT_ERR_DATA_REP          , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_DFT_ERR_DATA_REP          , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(ENH_FLUSH                    , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 3  , "HB"   , NULL     ),
    CMD(SEND_KEYPRESS_NOTIF          , BT_CTRL_BD_ADDR, 0, PK_GEN_GEN, 7  , "6BB"  , "B6B"    ),
    CMD(RD_ENH_TX_PWR_LVL            , BT_CTRL_CONHDL , 0, PK_GEN_GEN, 3  , "HB"   , "BHBBB"  ),
    CMD(RD_LE_HOST_SUPP              , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BBB"    ),
    CMD(WR_LE_HOST_SUPP              , BT_MNG         , 0, PK_GEN_GEN, 2  , "BB"   , "B"      ),
    #if RW_MWS_COEX
    CMD(SET_MWS_CHANNEL_PARAMS       , BT_MNG         , 0, PK_GEN_GEN, 10 , "BHHHHB", "B"     ),
    CMD(SET_EXTERNAL_FRAME_CONFIG    , BT_MNG         , 0, PK_SPE_GEN, 255, &hci_set_external_frame_config_cmd_upk, "B"   ),
    CMD(SET_MWS_SIGNALING            , BT_MNG         , 0, PK_GEN_GEN, 30 , "HHHHHHHHHHHHHHH", "BHHHHHHHHHHHHHHHH" ),
    CMD(SET_MWS_TRANSPORT_LAYER      , BT_MNG         , 0, PK_GEN_GEN, 9  , "BLL"   , "B"     ),
    CMD(SET_MWS_SCAN_FREQ_TABLE      , BT_MNG         , 0, PK_SPE_GEN, 255, &hci_set_mws_scan_freq_table_upk, "B" ),
    CMD(SET_MWS_PATTERN_CONFIG       , BT_MNG         , 0, PK_SPE_GEN, 255, &hci_set_mws_pattern_config_upk, "B" ),
    #endif //RW_MWS_COEX
    #if CSB_SUPPORT
    CMD(SET_RES_LT_ADDR              , BT_BCST        , 0, PK_GEN_GEN, 1  , "B"    , "BB"     ),
    CMD(DEL_RES_LT_ADDR              , BT_BCST        , 0, PK_GEN_GEN, 1  , "B"    , "BB"     ),
    CMD(SET_CON_SLV_BCST_DATA        , BT_BCST        , 0, PK_SPE_GEN, 255, &hci_set_con_slv_bcst_data_cmd_upk, "BB"      ),
    CMD(RD_SYNC_TRAIN_PARAM          , BT_BCST        , 0, PK_GEN_GEN, 0  , NULL   , "BHLB"   ),
    CMD(WR_SYNC_TRAIN_PARAM          , BT_BCST        , 0, PK_GEN_GEN, 9  , "HHLB" , "BH"     ),
    #endif //CSB_SUPPORT
    CMD(RD_SEC_CON_HOST_SUPP         , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BB"     ),
    CMD(WR_SEC_CON_HOST_SUPP         , BT_MNG         , 0, PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(RD_LOC_OOB_EXT_DATA          , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "B16B16B16B16B"),
    CMD(RD_EXT_PAGE_TO               , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BH"     ),
    CMD(WR_EXT_PAGE_TO               , BT_MNG         , 0, PK_GEN_GEN, 2  , "H"    , "B"      ),
    CMD(RD_EXT_INQ_LEN               , BT_MNG         , 0, PK_GEN_GEN, 0  , NULL   , "BH"     ),
    CMD(WR_EXT_INQ_LEN               , BT_MNG         , 0, PK_GEN_GEN, 2  , "H"    , "B"      ),
    #endif// BT_EMB_PRESENT
};

/// HCI command descriptors (OGF informational parameters)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_info_par[] =
{
    CMD(RD_LOCAL_VER_INFO   , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "BBHBHH"),
    CMD(RD_LOCAL_SUPP_CMDS  , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B64B"  ),
    CMD(RD_LOCAL_SUPP_FEATS , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B8B"   ),
    CMD(RD_BUFF_SIZE        , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "BHBHH" ),
    CMD(RD_BD_ADDR          , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B6B"   ),
    #if BT_EMB_PRESENT
    CMD(RD_LOCAL_EXT_FEATS  , BT_MNG, 0      , PK_GEN_GEN, 1, "B" , "BBB8B" ),
    CMD(RD_LOCAL_SUPP_CODECS, BT_MNG, 0      , PK_GEN_GEN, 0, NULL, "BBB" ),
    #endif //BT_EMB_PRESENT
};

/// HCI command descriptors (OGF status parameters)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_stat_par[] =
{
    CMD(RD_RSSI             , CTRL          , HL_CTRL, PK_GEN_GEN, 2, "H" , "BHB"   ),
    #if BT_EMB_PRESENT
    CMD(RD_FAIL_CONTACT_CNT , BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 2, "H" , "BHH"   ),
    CMD(RST_FAIL_CONTACT_CNT, BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 2, "H" , "BH"    ),
    CMD(RD_LINK_QUAL        , BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 2, "H" , "BHB"   ),
    CMD(RD_AFH_CH_MAP       , BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 2, "H" , "BHB10B"),
    CMD(RD_CLK              , BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 3, "HB", "BHLH"  ),
    CMD(RD_ENC_KEY_SIZE     , BT_CTRL_CONHDL, 0      , PK_GEN_GEN, 2, "H" , "BHB"   ),
    #if RW_MWS_COEX
    CMD(GET_MWS_TRANSPORT_LAYER_CONFIG, BT_MNG, 0    , PK_GEN_SPE, 0, NULL, &hci_get_mws_transport_layer_config_cmd_cmp_evt_pk),
    #endif //RW_MWS_COEX
    #endif //BT_EMB_PRESENT
};

///// HCI command descriptors (OGF testing)
#if BT_EMB_PRESENT
const struct hci_cmd_desc_tag hci_cmd_desc_tab_testing[] =
{
    CMD(RD_LOOPBACK_MODE , BT_MNG, 0, PK_GEN_GEN, 0, NULL, "BB"),
    CMD(WR_LOOPBACK_MODE , BT_MNG, 0, PK_GEN_GEN, 1, "B" , "B" ),
    CMD(EN_DUT_MODE      , BT_MNG, 0, PK_GEN_GEN, 0, NULL, "B" ),
    CMD(WR_SP_DBG_MODE   , BT_MNG, 0, PK_GEN_GEN, 1, "B" , "B" ),
    CMD(WR_SEC_CON_TEST_MODE, BT_CTRL_CONHDL, 0, PK_GEN_GEN, 4, "HBB" , "BH" ),
};
#endif// BT_EMB_PRESENT

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/// HCI command descriptors (OGF LE controller)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_le[] =
{
    CMD(LE_SET_EVT_MASK               , BLE_MNG , HL_MNG , PK_GEN_GEN, 8 , "8B"           , "B"     ),
    CMD(LE_RD_BUFF_SIZE               , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BHB"   ),
    CMD(LE_RD_LOCAL_SUPP_FEATS        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_SET_RAND_ADDR              , BLE_MNG , HL_MNG , PK_GEN_GEN, 6 , "6B"           , "B"     ),
    CMD(LE_SET_ADV_PARAM              , BLE_MNG , HL_MNG , PK_GEN_GEN, 15, "HHBBB6BBB"    , "B"     ),
    CMD(LE_RD_ADV_CHNL_TX_PW          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_SET_ADV_DATA               , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "B31B"         , "B"     ),
    CMD(LE_SET_SCAN_RSP_DATA          , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "B31B"         , "B"     ),
    CMD(LE_SET_ADV_EN                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_SET_SCAN_PARAM             , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "BHHBB"        , "B"     ),
    CMD(LE_SET_SCAN_EN                , BLE_MNG , HL_MNG , PK_GEN_GEN, 2 , "BB"           , "B"     ),
    CMD(LE_CREATE_CON                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 25, "HHBB6BBHHHHHH", NULL    ),
    CMD(LE_CREATE_CON_CANCEL          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_RD_WLST_SIZE               , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_CLEAR_WLST                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_ADD_DEV_TO_WLST            , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_RMV_DEV_FROM_WLST          , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_CON_UPDATE                 , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 14, "HHHHHHH"      , NULL    ),
    CMD(LE_SET_HOST_CH_CLASS          , BLE_MNG , HL_MNG , PK_GEN_GEN, 5 , "5B"           , "B"     ),
    CMD(LE_RD_CHNL_MAP                , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , "BH5B"  ),
    CMD(LE_RD_REM_USED_FEATS          , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , NULL    ),
    CMD(LE_ENC                        , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "16B16B"       , "B16B"  ),
    CMD(LE_RAND                       , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_START_ENC                  , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 28, "H8BH16B"      , NULL    ),
    CMD(LE_LTK_REQ_REPLY              , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 18, "H16B"         , "BH"    ),
    CMD(LE_LTK_REQ_NEG_REPLY          , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , "BH"    ),
    CMD(LE_RD_SUPP_STATES             , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_RX_TEST                    , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_TX_TEST                    , BLE_MNG , HL_MNG , PK_GEN_GEN, 3 , "BBB"          , "B"     ),
    CMD(LE_TEST_END                   , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BH"    ),
    #if !(BLE_QUALIF)
    CMD(LE_REM_CON_PARAM_REQ_REPLY    , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 14   , "HHHHHHH"                             , "BH"    ),
    CMD(LE_REM_CON_PARAM_REQ_NEG_REPLY, BLE_CTRL, HL_CTRL, PK_GEN_GEN, 3    , "HB"                                  , "BH"    ),
    CMD(LE_SET_DATA_LEN               , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 6    , "HHH"                                 , "BH"    ),
    CMD(LE_RD_SUGGTED_DFT_DATA_LEN    , BLE_MNG , HL_MNG , PK_GEN_GEN, 0    , NULL                                  , "BHH"   ),
    CMD(LE_WR_SUGGTED_DFT_DATA_LEN    , BLE_MNG , HL_MNG , PK_GEN_GEN, 4    , "HH"                                  , "B"     ),
    CMD(LE_RD_MAX_DATA_LEN            , BLE_MNG , HL_MNG , PK_GEN_GEN, 0    , NULL                                  , "BHHHH" ),
    CMD(LE_ADD_DEV_TO_RSLV_LIST       , BLE_MNG , HL_MNG , PK_GEN_GEN, 39   , "B6B16B16B"                           , "B"     ),
    CMD(LE_RMV_DEV_FROM_RSLV_LIST     , BLE_MNG , HL_MNG , PK_GEN_GEN, 7    , "B6B"                                 , "B"     ),
    CMD(LE_CLEAR_RSLV_LIST            , BLE_MNG , HL_MNG , PK_GEN_GEN, 0    , NULL                                  , "B"     ),
    CMD(LE_RD_RSLV_LIST_SIZE          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0    , NULL                                  , "BB"    ),
    CMD(LE_RD_PEER_RSLV_ADDR          , BLE_MNG , HL_MNG , PK_GEN_GEN, 7    , "B6B"                                 , "B6B"   ),
    CMD(LE_RD_LOC_RSLV_ADDR           , BLE_MNG , HL_MNG , PK_GEN_GEN, 7    , "B6B"                                 , "B6B"   ),
    CMD(LE_SET_ADDR_RESOL_EN          , BLE_MNG , HL_MNG , PK_GEN_GEN, 1    , "B"                                   , "B"     ),
    CMD(LE_SET_RSLV_PRIV_ADDR_TO      , BLE_MNG , HL_MNG , PK_GEN_GEN, 2    , "H"                                   , "B"     ),
    CMD(LE_RD_LOC_P256_PUB_KEY        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0    , NULL                                  , NULL    ),
    CMD(LE_GEN_DHKEY                  , BLE_MNG , HL_MNG , PK_GEN_GEN, 64   , "64B"                                 , NULL    ),
    #if (BLE_2MBPS)
    CMD(LE_RD_PHY                     , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2    , "H"                                   , "BHBB"  ),
    CMD(LE_SET_DFT_PHY                , BLE_MNG , HL_MNG , PK_GEN_GEN, 3    , "BBB"                                 , "B"     ),
    CMD(LE_SET_PHY                    , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 7    , "HBBBH"                               , NULL    ),
    CMD(LE_ENH_RX_TEST                , BLE_MNG , HL_MNG , PK_GEN_GEN, 3    , "BBB"                                 , "B"     ),
    CMD(LE_ENH_TX_TEST                , BLE_MNG , HL_MNG , PK_GEN_GEN, 4    , "BBBB"                                , "B"     ),
    #endif // (BLE_2MBPS)
    #endif // !(BLE_QUALIF)
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT || BLE_HOST_PRESENT)
///// HCI command descriptors (OGF Vendor Specific)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_vs[] =
{
    #if (RW_DEBUG && (!BLE_HOST_PRESENT) && HCI_TL_SUPPORT)
    CMD(DBG_RD_MEM           , DBG, 0, PK_GEN_SPE, 6  , "LBB"                      , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_WR_MEM           , DBG, 0, PK_SPE_GEN, 136, &hci_dbg_wr_mem_cmd_upk    , "B"                            ),
    CMD(DBG_DEL_PAR          , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_ID_FLASH         , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BB"                           ),
    CMD(DBG_ER_FLASH         , DBG, 0, PK_GEN_GEN, 9  , "BLL"                      , "B"                            ),
    CMD(DBG_WR_FLASH         , DBG, 0, PK_SPE_GEN, 140, &hci_dbg_wr_flash_cmd_upk  , "B"                            ),
    CMD(DBG_RD_FLASH         , DBG, 0, PK_GEN_SPE, 6  , "BLB"                      , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_RD_PAR           , DBG, 0, PK_GEN_SPE, 2  , "H"                        , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_WR_PAR           , DBG, 0, PK_SPE_GEN, 132, &hci_dbg_wr_par_cmd_upk    , "B"                            ),
    #endif //(RW_DEBUG && (!BLE_HOST_PRESENT || HCI_TL_SUPPORT))
    #if (RW_DEBUG)
    CMD(DBG_RD_KERNEL_STATS      , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BBBBHH"                       ),
    #if (RW_WLAN_COEX)
    CMD(DBG_WLAN_COEX        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    #if (RW_WLAN_COEX_TEST)
    CMD(DBG_WLAN_COEXTST_SCEN, DBG, 0, PK_GEN_GEN, 4  , "L"                        , "B"                            ),
    #endif //RW_BT_WLAN_COEX_TEST
    #endif //RW_WLAN_COEX
    #if (RW_MWS_COEX)
    CMD(DBG_MWS_COEX        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    #if (RW_MWS_COEX_TEST)
    CMD(DBG_MWS_COEXTST_SCEN, DBG, 0, PK_GEN_GEN, 4  , "L"                        , "B"                            ),
    #endif //RW_BT_MWS_COEX_TEST
    #endif //RW_MWS_COEX
    #endif //(RW_DEBUG)
    CMD(DBG_PLF_RESET        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    CMD(DBG_RD_MEM_INFO      , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BHHHL"                        ),
    #if BLE_EMB_PRESENT
    #if (RW_DEBUG)
    CMD(DBG_HW_REG_RD        , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "BHL"                          ),
    CMD(DBG_HW_REG_WR        , DBG, 0, PK_GEN_GEN, 6  , "HL"                       , "BH"                           ),
    CMD(DBG_SET_BD_ADDR      , DBG, 0, PK_GEN_GEN, 6  , "6B"                       , "B"                            ),
    CMD(DBG_SET_CRC          , DBG, 0, PK_GEN_GEN, 5  , "H3B"                      , "B"                            ),
    CMD(DBG_LLCP_DISCARD     , DBG, 0, PK_GEN_GEN, 3  , "HB"                       , "B"                            ),
    CMD(DBG_RESET_RX_CNT     , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_RESET_TX_CNT     , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_RF_REG_RD        , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "BHL"                          ),
    CMD(DBG_RF_REG_WR        , DBG, 0, PK_GEN_GEN, 6  , "HL"                       , "BHE"                          ),
    CMD(DBG_WR_DLE_DFT_VALUE , DBG, 0, PK_GEN_GEN, 8  , "HHHH"                     , "B"                            ),
    #endif //(RW_DEBUG)
    #if (BLE_TESTER)
    CMD(TESTER_SET_LE_PARAMS,   BLE_CTRL, 0,PK_GEN_GEN, 16, "HBBHHHHHH"               , NULL  ),
    CMD(DBG_BLE_TST_LLCP_PT_EN, BLE_CTRL, 0,PK_GEN_GEN, 3,  "HB"                      , NULL  ),
    CMD(DBG_BLE_TST_SEND_LLCP,  BLE_CTRL, 0,PK_GEN_GEN, 29, "HB26B"                   , NULL  ),
    #endif// (BLE_TESTER)
    CMD(DBG_SET_TX_PW        , DBG, 0, PK_GEN_GEN, 3  , "HB"                        , "B"                            ),
    #endif //BLE_EMB_PRESENT
    #if (BLE_AUDIO)
    CMD(VS_AUDIO_ALLOCATE           , DBG, HL_AM0,PK_GEN_GEN,  2,  "H"                         , "BHB"                           ),
    CMD(VS_AUDIO_CONFIGURE          , DBG, HL_AM0,PK_GEN_GEN, 16,  "HBBBBBBBBBB"                , "BH"                            ),
    CMD(VS_AUDIO_SET_MODE           , DBG, HL_AM0,PK_GEN_GEN,  4,  "HBB"                        , "BH"                            ),
    CMD(VS_AUDIO_RESET              , DBG, HL_AM0,PK_GEN_GEN,  3,  "HB"                         , "BH"                            ),
    CMD(VS_AUDIO_SET_POINTER        , DBG, HL_AM0,PK_GEN_GEN,  7,  "HBBBH"                      , "BH"                            ),
    CMD(VS_AUDIO_GET_BUFFER_RANGE   , DBG, HL_AM0,PK_GEN_GEN,  1,  "B"                         ,  "BLL"                          ),
    #endif //BLE_AUDIO

    #if (BT_EMB_PRESENT && RW_DEBUG)
    CMD(DBG_BT_SEND_LMP      ,  BT_CTRL_CONHDL, 0, PK_SPE_GEN, 131,  &hci_dbg_bt_send_lmp_cmd_upk, "BH"  ),
    CMD(DBG_BT_DISCARD_LMP_EN, 	BT_CTRL_CONHDL, 0, PK_GEN_GEN, 3  ,  "HB"                        , "BH"  ),
    #endif //BT_EMB_PRESENT && RW_DEBUG
    #if CRYPTO_UT
    CMD(DBG_TEST_CRYPTO_FUNC , BT_MNG, 0, PK_SPE_GEN, 136, &hci_dbg_test_crypto_func_cmd_upk, "B"                   ),
    #endif //CRYPTO_UT
};
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/// HCI command descriptors root table (classified by OGF)
const struct hci_cmd_desc_tab_ref hci_cmd_desc_root_tab[] =
{
    {LK_CNTL_OGF,  ARRAY_LEN(hci_cmd_desc_tab_lk_ctrl) , hci_cmd_desc_tab_lk_ctrl },
    {CNTLR_BB_OGF, ARRAY_LEN(hci_cmd_desc_tab_ctrl_bb) , hci_cmd_desc_tab_ctrl_bb },
    {INFO_PAR_OGF, ARRAY_LEN(hci_cmd_desc_tab_info_par), hci_cmd_desc_tab_info_par},
    {STAT_PAR_OGF, ARRAY_LEN(hci_cmd_desc_tab_stat_par), hci_cmd_desc_tab_stat_par},
    #if BT_EMB_PRESENT
    {LK_POL_OGF,   ARRAY_LEN(hci_cmd_desc_tab_lk_pol)  , hci_cmd_desc_tab_lk_pol  },
    {TEST_OGF  ,   ARRAY_LEN(hci_cmd_desc_tab_testing) , hci_cmd_desc_tab_testing },
    #endif // BT_EMB_PRESENT
    #if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    {LE_CNTLR_OGF, ARRAY_LEN(hci_cmd_desc_tab_le)      , hci_cmd_desc_tab_le      },
    #endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT || BLE_HOST_PRESENT)
    {VS_OGF,       ARRAY_LEN(hci_cmd_desc_tab_vs)      , hci_cmd_desc_tab_vs      },
    #endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
};

/// HCI event descriptors
const struct hci_evt_desc_tag hci_evt_desc_tab[] =
{
    EVT(DISC_CMP                 , HL_CTRL, PK_GEN, "BHB"          ),
    EVT(ENC_CHG                  , HL_CTRL, PK_GEN, "BHB"          ),
    EVT(RD_REM_VER_INFO_CMP      , HL_CTRL, PK_GEN, "BHBHH"        ),
    EVT(HW_ERR                   , HL_MNG , PK_GEN, "B"            ),
    EVT(FLUSH_OCCURRED           , HL_CTRL, PK_GEN, "H"            ),
    EVT(NB_CMP_PKTS              , HL_DATA, PK_GEN, "BHH"          ),
    EVT(DATA_BUF_OVFLW           , HL_MNG , PK_GEN, "B"            ),
    EVT(ENC_KEY_REFRESH_CMP      , HL_CTRL, PK_GEN, "BH"           ),
    EVT(AUTH_PAYL_TO_EXP         , HL_CTRL, PK_GEN, "H"            ),
    #if BT_EMB_PRESENT
    EVT(INQ_CMP                  , 0      , PK_GEN, "B"            ),
    EVT(INQ_RES                  , 0      , PK_GEN, "B6BBBB3BH"    ),
    EVT(CON_CMP                  , 0      , PK_GEN, "BH6BBB"       ),
    EVT(CON_REQ                  , 0      , PK_GEN, "6B3BB"        ),
    EVT(AUTH_CMP                 , 0      , PK_GEN, "BH"           ),
    EVT(REM_NAME_REQ_CMP         , 0      , PK_GEN, "B6B248B"      ),
    EVT(CHG_CON_LK_CMP           , 0      , PK_GEN, "BH"           ),
    EVT(MASTER_LK_CMP            , 0      , PK_GEN, "BHB"          ),
    EVT(RD_REM_SUPP_FEATS_CMP    , 0      , PK_GEN, "BH8B"         ),
    EVT(QOS_SETUP_CMP            , 0      , PK_GEN, "BHBBLLLL"     ),
    EVT(ROLE_CHG                 , 0      , PK_GEN, "B6BB"         ),
    EVT(MODE_CHG                 , 0      , PK_GEN, "BHBH"         ),
    EVT(RETURN_LINK_KEYS         , 0      , PK_GEN, "B6B16B"       ),
    EVT(PIN_CODE_REQ             , 0      , PK_GEN, "6B"           ),
    EVT(LK_REQ                   , 0      , PK_GEN, "6B"           ),
    EVT(LK_NOTIF                 , 0      , PK_GEN, "6B16BB"       ),
    EVT(MAX_SLOT_CHG             , 0      , PK_GEN, "HB"           ),
    EVT(RD_CLK_OFF_CMP           , 0      , PK_GEN, "BHH"          ),
    EVT(CON_PKT_TYPE_CHG         , 0      , PK_GEN, "BHH"          ),
    EVT(QOS_VIOL                 , 0      , PK_GEN, "H"            ),
    EVT(PAGE_SCAN_REPET_MODE_CHG , 0      , PK_GEN, "6BB"          ),
    EVT(FLOW_SPEC_CMP            , 0      , PK_GEN, "BHBBBLLLL"    ),
    EVT(INQ_RES_WITH_RSSI        , 0      , PK_GEN, "B6BBB3BHB"    ),
    EVT(RD_REM_EXT_FEATS_CMP     , 0      , PK_GEN, "BHBB8B"       ),
    EVT(SYNC_CON_CMP             , 0      , PK_GEN, "BH6BBBBHHB"   ),
    EVT(SYNC_CON_CHG             , 0      , PK_GEN, "BHBBHH"       ),
    EVT(SNIFF_SUB                , 0      , PK_GEN, "BHHHHH"       ),
    EVT(EXT_INQ_RES              , 0      , PK_GEN, "B6BBB3BHB240B"),
    EVT(IO_CAP_REQ               , 0      , PK_GEN, "6B"           ),
    EVT(IO_CAP_RSP               , 0      , PK_GEN, "6BBBB"        ),
    EVT(USER_CFM_REQ             , 0      , PK_GEN, "6BL"          ),
    EVT(USER_PASSKEY_REQ         , 0      , PK_GEN, "6B"           ),
    EVT(REM_OOB_DATA_REQ         , 0      , PK_GEN, "6B"           ),
    EVT(SP_CMP                   , 0      , PK_GEN, "B6B"          ),
    EVT(LINK_SUPV_TO_CHG         , 0      , PK_GEN, "HH"           ),
    EVT(ENH_FLUSH_CMP            , 0      , PK_GEN, "H"            ),
    EVT(USER_PASSKEY_NOTIF       , 0      , PK_GEN, "6BL"          ),
    EVT(KEYPRESS_NOTIF           , 0      , PK_GEN, "6BB"          ),
    EVT(REM_HOST_SUPP_FEATS_NOTIF, 0      , PK_GEN, "6B8B"         ),
    #if CSB_SUPPORT
    EVT(SYNC_TRAIN_CMP           , 0      , PK_GEN, "B"            ),
    EVT(SYNC_TRAIN_REC           , 0      , PK_GEN, "B6BL10BBLHB"  ),
    EVT(CON_SLV_BCST_REC         , 0      , PK_SPE, &hci_con_slv_bcst_rec_evt_pkupk),
    EVT(CON_SLV_BCST_TO          , 0      , PK_GEN, "6BB"          ),
    EVT(TRUNC_PAGE_CMP           , 0      , PK_GEN, "B6B"          ),
    EVT(SLV_PAGE_RSP_TO          , 0      , PK_GEN, NULL           ),
    EVT(CON_SLV_BCST_CH_MAP_CHG  , 0      , PK_GEN, "10B"          ),
    #endif //CSB_SUPPORT
    #endif// BT_EMB_PRESENT
};


// Note: remove specific BLE Flag as soon as new debug event available on BT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
/// HCI DBG event descriptors
const struct hci_evt_desc_tag hci_evt_dbg_desc_tab[] =
{
    #if (BLE_EMB_PRESENT)
    #if (BLE_TESTER)
    DBG_EVT(DBG_BLE_TST_LLCP_RECV, HL_CTRL, PK_GEN, "BHB26B"),
    #endif // (BLE_TESTER)
    #endif // (BLE_EMB_PRESENT)

    #if (RW_DEBUG)
    DBG_EVT(DBG_ASSERT_ERR       , 0      , PK_SPE, &hci_dbg_assert_err_evt_pkupk ),
    #endif //(RW_DEBUG)
};
#endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// HCI LE event descriptors
const struct hci_evt_desc_tag hci_evt_le_desc_tab[] =
{
    LE_EVT(LE_CON_CMP,                 HL_MNG , PK_GEN, "BBHBB6BHHHB"             ),
    LE_EVT(LE_ADV_REPORT,              HL_MNG , PK_SPE, &hci_le_adv_report_evt_pkupk ),
    LE_EVT(LE_CON_UPDATE_CMP,          HL_CTRL, PK_GEN, "BBHHHH"                  ),
    LE_EVT(LE_RD_REM_USED_FEATS_CMP,   HL_CTRL, PK_GEN, "BBH8B"                   ),
    LE_EVT(LE_LTK_REQUEST,             HL_CTRL, PK_GEN, "BH8BH"                   ),
    LE_EVT(LE_REM_CON_PARAM_REQ,       HL_CTRL, PK_GEN, "BHHHHH"                  ),
    LE_EVT(LE_DATA_LEN_CHG,            HL_CTRL, PK_GEN, "BHHHHH"                  ),
    LE_EVT(LE_ENH_CON_CMP,             HL_MNG , PK_GEN, "BBHBB6B6B6BHHHB"             ),
#if (SECURE_CONNECTIONS==1)
    LE_EVT(LE_RD_LOC_P256_PUB_KEY_CMP, HL_MNG , PK_GEN, "BB64B"                   ),
    LE_EVT(LE_GEN_DHKEY_CMP,           HL_MNG , PK_GEN, "BB32B"                   ),
#endif //(SECURE_CONNECTIONS==1)
    LE_EVT(LE_DIR_ADV_REP,             HL_MNG , PK_SPE, &hci_le_dir_adv_report_evt_pkupk),
#if (BLE_2MBPS)
    LE_EVT(LE_PHY_UPD_CMP,             HL_CTRL, PK_GEN, "BBHBB"             ),
#endif // (BLE_2MBPS)
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)


/*
 * SPECIAL PACKER-UNPACKER DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
****************************************************************************************
* @brief Apply a basic pack operation
*
* @param[inout] pp_in      Current input buffer position
* @param[inout] pp_out     Current output buffer position
* @param[in]    p_in_end   Input buffer end
* @param[in]    p_out_end  Output buffer end
* @param[in]    len        Number of bytes to copy
*
* @return status
*****************************************************************************************
*/
static uint8_t hci_pack_bytes(uint8_t** pp_in, uint8_t** pp_out, uint8_t* p_in_end, uint8_t* p_out_end, uint8_t len)
{
    uint8_t status = HCI_PACK_OK;

    // Check if enough space in input buffer to read
    if((*pp_in + len) > p_in_end)
    {
        status = HCI_PACK_IN_BUF_OVFLW;
    }
    else
    {
        if(p_out_end != NULL)
        {
            // Check if enough space in out buffer to write
            if((*pp_out + len) > p_out_end)
            {
                status = HCI_PACK_OUT_BUF_OVFLW;
            }

            // Copy BD Address
            memcpy(*pp_out, *pp_in, len);
        }
        *pp_in = *pp_in + len;
        *pp_out = *pp_out + len;
    }

    return (status);
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/// Special packing/unpacking function for HCI Host Number Of Completed Packets Command
static uint8_t hci_host_nb_cmp_pkts_cmd_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    /*
     * PACKING FUNCTION
     */
    struct hci_host_nb_cmp_pkts_cmd* cmd = (struct hci_host_nb_cmp_pkts_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t i = 0;
            // Number of handles
            p_out = &cmd->nb_of_hdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            while((i < *in) && ((p_out - out) <= sizeof(struct hci_host_nb_cmp_pkts_cmd)))
            {
                // Connection handle
                p_out = (uint8_t*) &cmd->con_hdl[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Host_Num_Of_Completed_Packets
                p_out = (uint8_t*) &cmd->nb_comp_pkt[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                i++;
            }
        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_host_nb_cmp_pkts_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    return sizeof(struct hci_host_nb_cmp_pkts_cmd);

    #endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
}

#if (BT_EMB_PRESENT)
/// Special packing/unpacking function for HCI Set Event Filter command
static uint8_t hci_set_evt_filter_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_set_evt_filter_cmd temp_out;
    struct hci_set_evt_filter_cmd* cmd = (struct hci_set_evt_filter_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_evt_filter_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            cmd = (struct hci_set_evt_filter_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        do
        {
            // Filter Type
            p_out = &cmd->filter_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Check filter type
            switch(cmd->filter_type)
            {
                case INQUIRY_FILTER_TYPE:
                {
                    // Filter Condition Type
                    p_out = &cmd->filter.inq_res.cond_type;
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                    if(status != HCI_PACK_OK)
                        break;

                    // Check Filter Condition Type
                    switch(cmd->filter.inq_res.cond_type)
                    {
                        case CLASS_FILTER_CONDITION_TYPE:
                        {
                            // Class_of_Device
                            p_out = &cmd->filter.inq_res.cond.cond_1.class_of_dev.A[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                            if(status != HCI_PACK_OK)
                                break;

                            // Class_of_Device_Mask
                            p_out = &cmd->filter.inq_res.cond.cond_1.class_of_dev_msk.A[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                            if(status != HCI_PACK_OK)
                                break;
                        }
                        break;
                        case BD_ADDR_FILTER_CONDITION_TYPE:
                        {
                            // BD Address
                            p_out = &cmd->filter.inq_res.cond.cond_2.bd_addr.addr[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                            if(status != HCI_PACK_OK)
                                break;
                        }
                        break;
                        default:
                        {
                            // Nothing
                        }
                        break;
                    }
                }
                break;
                case CONNECTION_FILTER_TYPE:
                {
                    // Filter Condition Type
                    p_out = &cmd->filter.con_set.cond_type;
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                    if(status != HCI_PACK_OK)
                        break;

                    // Check Filter Condition Type
                    switch(cmd->filter.inq_res.cond_type)
                    {
                        case ALL_FILTER_CONDITION_TYPE:
                        {
                            // Auto_Accept_Flag
                            p_out = &cmd->filter.con_set.cond.cond_0.auto_accept;
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                            if(status != HCI_PACK_OK)
                                break;
                        }
                        break;
                        case CLASS_FILTER_CONDITION_TYPE:
                        {
                            // Class_of_Device
                            p_out = &cmd->filter.con_set.cond.cond_1.class_of_dev.A[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                            if(status != HCI_PACK_OK)
                                break;

                            // Class_of_Device_Mask
                            p_out = &cmd->filter.con_set.cond.cond_1.class_of_dev_msk.A[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                            if(status != HCI_PACK_OK)
                                break;

                            // Auto_Accept_Flag
                            p_out = &cmd->filter.con_set.cond.cond_1.auto_accept;
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                            if(status != HCI_PACK_OK)
                                break;
                        }
                        break;
                        case BD_ADDR_FILTER_CONDITION_TYPE:
                        {
                            // BD Address
                            p_out = &cmd->filter.con_set.cond.cond_2.bd_addr.addr[0];
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                            if(status != HCI_PACK_OK)
                                break;

                            // Auto_Accept_Flag
                            p_out = &cmd->filter.con_set.cond.cond_2.auto_accept;
                            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                            if(status != HCI_PACK_OK)
                                break;
                        }
                        break;
                        default:
                        {
                            // Nothing
                        }
                        break;
                    }
                }
                break;
                default:
                {
                    // Nothing
                }
                break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_set_evt_filter_cmd);
    }

    return (status);
}
/// Special packing/unpacking function for HCI Write Stored Link Key command
static uint8_t hci_wr_stored_lk_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_wr_stored_lk_cmd* cmd = (struct hci_wr_stored_lk_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            // Number of keys
            p_out = &cmd->num_key_wr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Parse BD Address + Key table
            for(uint8_t i = 0 ; i < *in ; i++)
            {
                // BD Address
                p_out = &cmd->link_keys[i].bd_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Link Key
                p_out = &cmd->link_keys[i].link_key.ltk[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, KEY_LEN);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_wr_stored_lk_cmd);
    }

    return (status);
}

/// Special packing/unpacking function for HCI Write Stored Link Key command
static uint8_t hci_wr_curr_iac_lap_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_wr_curr_iac_lap_cmd* cmd = (struct hci_wr_curr_iac_lap_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            // Number of IACs
            p_out = &cmd->nb_curr_iac;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Parse IAC LAP table
            for(uint8_t i = 0 ; i < *in ; i++)
            {
                // LAP
                p_out = &cmd->iac_lap[i].A[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LAP_LEN);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_wr_curr_iac_lap_cmd);
    }

    return (status);
}

#if CSB_SUPPORT
/// Special packing/unpacking function for HCI Set Connectionless Slave Broadcast Data command
static uint8_t hci_set_con_slv_bcst_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_set_con_slv_bcst_data_cmd temp_out;
    struct hci_set_con_slv_bcst_data_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_con_slv_bcst_data_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_set_con_slv_bcst_data_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // LT_ADDR
            p_out = &cmd->lt_addr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Fragment
            p_out = &cmd->fragment;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data_Length
            p_out = &cmd->data_length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, cmd->data_length);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

          *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = HCI_MAX_CMD_PARAM_SIZE;
    }

    return (status);
}

/// Special packing/unpacking function for HCI Connectionless Slave Broadcast Receive Event
static uint8_t hci_con_slv_bcst_rec_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_con_slv_bcst_rec_evt temp_out;
    struct hci_con_slv_bcst_rec_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_con_slv_bcst_rec_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            s = (struct hci_con_slv_bcst_rec_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        p_out = p_out_start;

        do
        {
            // BD_ADDR
            p_in = &s->bd_addr.addr[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
            if(status != HCI_PACK_OK)
                break;

            // LT_ADDR
            p_in = &s->lt_addr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // CLK
            p_in = (uint8_t*) &s->clk;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Offset
            p_in = (uint8_t*) &s->offset;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Receive_Status
            p_in = &s->receive_status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Fragment
            p_in = &s->fragment;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data_Length
            p_in = &s->data_length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_in = &s->data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, s->data_length);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);
}
#endif //CSB_SUPPORT

#if RW_MWS_COEX
/// Special packing/unpacking function for HCI Set External Frame Configuration command
static uint8_t hci_set_external_frame_config_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_set_external_frame_config_cmd temp_out;
    struct hci_set_external_frame_config_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_external_frame_config_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_set_external_frame_config_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Ext_Frame_Duration
            p_out = (uint8_t*)&cmd->ext_fr_duration;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Ext_Frame_Sync_Assert_Offset
            p_out = (uint8_t*)&cmd->ext_fr_sync_assert_offset;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Ext_Frame_Sync_Assert_Jitter
            p_out = (uint8_t*)&cmd->ext_fr_sync_assert_jitter;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Ext_Frame_Num_Periods
            p_out = (uint8_t*)&cmd->ext_fr_num_periods;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Period_Duration[i]
            p_out = (uint8_t*)&cmd->period_duration[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, (cmd->ext_fr_num_periods)*sizeof(uint16_t));
            if(status != HCI_PACK_OK)
                break;

            // Period_Type[i]
            p_out = (uint8_t*)&cmd->period_duration[cmd->ext_fr_num_periods];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, cmd->ext_fr_num_periods);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = HCI_MAX_CMD_PARAM_SIZE;
    }

    return (status);
}

/// Special packing/unpacking function for HCI Set MWS Scan Frequency Table command
static uint8_t hci_set_mws_scan_freq_table_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_set_mws_scan_freq_table_cmd temp_out;
    struct hci_set_mws_scan_freq_table_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_mws_scan_freq_table_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_set_mws_scan_freq_table_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Num_Scan_Frequencies
            p_out = (uint8_t*)&cmd->num_scan_frequencies;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Scan_Frequency_Low[i]
            p_out = (uint8_t*)&cmd->scan_frequency_low[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, (cmd->num_scan_frequencies)*sizeof(uint16_t));
            if(status != HCI_PACK_OK)
                break;

            // Scan_Frequency_High[i]
            p_out = (uint8_t*)&cmd->scan_frequency_low[cmd->num_scan_frequencies];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, (cmd->num_scan_frequencies)*sizeof(uint16_t));
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = HCI_MAX_CMD_PARAM_SIZE;
    }

    return (status);
}

/// Special packing/unpacking function for HCI Set MWS Pattern Configuration command
static uint8_t hci_set_mws_pattern_config_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_set_mws_pattern_config_cmd temp_out;
    struct hci_set_mws_pattern_config_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_mws_pattern_config_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_set_mws_pattern_config_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            //MWS_PATTERN_Index
            p_out = (uint8_t*)&cmd->mws_pattern_index;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            //MWS_PATTERN_NumIntervals
            p_out = (uint8_t*)&cmd->mws_pattern_num_intervals;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // MWS_PATTERN_IntervalDuration[i]
            p_out = (uint8_t*)&cmd->mws_pattern_interval_duration[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, (cmd->mws_pattern_num_intervals)*sizeof(uint16_t));
            if(status != HCI_PACK_OK)
                break;

            // MWS_PATTERN_IntervalType[i]
            p_out = (uint8_t*)&cmd->mws_pattern_interval_duration[cmd->mws_pattern_num_intervals];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, copy_data?p_out_end:NULL, cmd->mws_pattern_num_intervals);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = HCI_MAX_CMD_PARAM_SIZE;
    }

    return (status);
}

/// Special packing/unpacking function for HCI Set MWS Pattern Configuration command complete event
static uint8_t hci_get_mws_transport_layer_config_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_get_mws_transport_layer_config_cmd_cmp_evt* evt = (struct hci_get_mws_transport_layer_config_cmd_cmp_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t num_transports, sum_num_baud_rates;
            int i, offset;

            // Status
            p_in = &evt->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data Length
            p_in = &evt->num_transports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            num_transports = evt->num_transports;

            // Data
            p_in = &evt->transport_layer[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, num_transports);
            if(status != HCI_PACK_OK)
                break;

            // Num Baud Rates
            p_in = &evt->transport_layer[num_transports]; 
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, num_transports);
            if(status != HCI_PACK_OK)
                break;

            sum_num_baud_rates = 0;
            p_in = &evt->transport_layer[num_transports]; 
            for (i=0; i<num_transports; i++)
            {
                sum_num_baud_rates += p_in[i];
            }
            
            offset = (num_transports*2)/sizeof(uint32_t);

            // To MWS Baud Rate
            p_in = (uint8_t*)&evt->to_mws_baud_rate[offset];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, sum_num_baud_rates*sizeof(uint32_t));
            if(status != HCI_PACK_OK)
                break;

            // From MWS Baud Rate
            p_in = (uint8_t*)&evt->to_mws_baud_rate[sum_num_baud_rates + offset];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, sum_num_baud_rates*sizeof(uint32_t));
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

    return (status);
}
#endif //RW_MWS_COEX
#if RW_DEBUG
/// Special packing/unpacking function for HCI Debug Bt Send LMP  Command
static uint8_t hci_dbg_bt_send_lmp_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_bt_send_lmp_cmd* cmd = (struct hci_dbg_bt_send_lmp_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Connection handle
            p_out = (uint8_t*) &cmd->conhdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_bt_send_lmp_cmd);
    }

    return (status);
}
#endif //RW_DEBUG

#endif //(BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// Special packing/unpacking function for HCI LE Advertising Report Event
static uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_adv_report_evt temp_out;
    struct hci_le_adv_report_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_adv_report_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_adv_report_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;
            int i = 0 ;
            for ( i = 0; i< s->nb_reports; i++)
            {
                uint8_t data_len;

                // Event type
                p_in = &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Address type
                p_in = &s->adv_rep[i].adv_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // BD Address
                p_in = &s->adv_rep[i].adv_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Data Length
                p_in = &s->adv_rep[i].data_len;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                data_len = s->adv_rep[i].data_len;

                // ADV data
                p_in = &s->adv_rep[i].data[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
                if(status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */

    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Direct Advertising Report Event
static uint8_t hci_le_dir_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_dir_adv_rep_evt temp_out;
    struct hci_le_dir_adv_rep_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_dir_adv_rep_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_dir_adv_rep_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;
            int i=0 ;
            for (i=0; i< s->nb_reports; i++)
            {
                // Event type
                p_in = &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;


                // Address type
                p_in = &s->adv_rep[i].addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // BD Address
                p_in = &s->adv_rep[i].addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Direct Address type
                p_in = &s->adv_rep[i].dir_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Direct BD Address
                p_in = &s->adv_rep[i].dir_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */

    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}
#if (0)
static uint8_t hci_le_enh_receiver_test_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_le_enh_rx_test_cmd* cmd = (struct hci_le_enh_rx_test_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            //Number of octets in the command parameters
            p_out = (uint8_t*) &cmd->param_size;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;


            p_out = &cmd->channel;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->phys;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->modulation_idx;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->unknown[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, cmd->param_size-3);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_enh_rx_test_cmd);
    }

    return (status);

}
static uint8_t hci_le_enh_transmitter_test_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_le_enh_tx_test_cmd* cmd = (struct hci_le_enh_tx_test_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            //Number of octets in the command parameters
            p_out = (uint8_t*) &cmd->param_size;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->channel;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->payload_length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->payload_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->phys;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            p_out = &cmd->unknown[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, cmd->param_size-4);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_enh_rx_test_cmd);
    }

    return (status);
}
#endif // (BLE_2MBPS)
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
/// Special packing/unpacking function for HCI Debug Write Memory Command
uint8_t hci_dbg_wr_mem_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_mem_cmd* cmd = (struct hci_dbg_wr_mem_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Start address
            p_out = (uint8_t*) &cmd->start_addr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Memory access type
            p_out = &cmd->type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_mem_cmd);
    }

    return (status);
}

/// Special packing/unpacking function for HCI Debug Write Flash Command
uint8_t hci_dbg_wr_flash_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_flash_cmd* cmd = (struct hci_dbg_wr_flash_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Flash type
            p_out = &cmd->flashtype;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Start offset
            p_out = (uint8_t*) &cmd->startoffset;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_flash_cmd);
    }

    return (status);
}

#if CRYPTO_UT
static uint8_t hci_dbg_test_crypto_func_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
	struct hci_dbg_test_crypto_func_cmd* cmd = (struct hci_dbg_test_crypto_func_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Function type
              p_out = &cmd->function;
              status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
              if(status != HCI_PACK_OK)
                  break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_test_crypto_func_cmd);
    }

    return (status);
}
#endif //CRYPTO_UT

/// Special packing/unpacking function for HCI Debug Write Param Command
uint8_t hci_dbg_wr_par_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_par_cmd* cmd = (struct hci_dbg_wr_par_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Tag
            p_out = (uint8_t*) &cmd->param_tag;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_par_cmd);
    }

    return (status);
}

/// Special packing/unpacking function for Command Complete Event of HCI Debug Read Memory/Flash/Param Command
uint8_t hci_dbg_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_basic_rd_data_cmd_cmp_evt* evt = (struct hci_dbg_basic_rd_data_cmd_cmp_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Status
            p_in = &evt->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data Length
            p_in = &evt->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            data_len = evt->buf.length;

            // Data
            p_in = &evt->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

    return (status);
}

/// Special packing/unpacking function for HCI DBG assert error Event
uint8_t hci_dbg_assert_err_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_assert_err_evt* evt = (struct hci_dbg_assert_err_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            // Subcode
            p_in = &evt->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Line
            p_in = (uint8_t*) &evt->line;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Param0
            p_in = (uint8_t*) &evt->param0;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Param1
            p_in = (uint8_t*) &evt->param1;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // File
            p_in = (uint8_t*) &evt->file;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, strlen((char*) evt->file));
            if(status != HCI_PACK_OK)
                break;
        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }


    return (status);
}
#endif //(RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
#endif //(HCI_TL_SUPPORT)


/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

const struct hci_cmd_desc_tag* hci_look_for_cmd_desc(uint16_t opcode)
{
    const struct hci_cmd_desc_tag* tab = NULL;
    const struct hci_cmd_desc_tag* desc = NULL;
    uint16_t nb_cmds = 0;
    uint16_t index = 0;
    uint16_t ocf = HCI_OP2OCF(opcode);
    uint16_t ogf = HCI_OP2OGF(opcode);

    // Find table corresponding to this OGF
    for(index = 0; index < ARRAY_LEN(hci_cmd_desc_root_tab); index++)
    {
        // Compare the command opcodes
        if(hci_cmd_desc_root_tab[index].ogf == ogf)
        {
            // Get the command descriptors table information (size and pointer)
            tab = hci_cmd_desc_root_tab[index].cmd_desc_tab;
            nb_cmds = hci_cmd_desc_root_tab[index].nb_cmds;
            break;
        }
    }

    // Check if a table has been found for this OGF
    if(tab != NULL)
    {
        // Find the command descriptor associated to this OCF
        for(index = 0; index < nb_cmds; index++)
        {
            // Compare the command opcodes
            if(HCI_OP2OCF(tab->opcode) == ocf)
            {
                // Get the command descriptor pointer
                desc = tab;
                break;
            }

            // Jump to next command descriptor
            tab++;
        }
    }

    return (desc);
}

/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified event code
*
* @param[in]  code   event code
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
const struct hci_evt_desc_tag* hci_look_for_evt_desc(uint8_t code)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_desc_tab)/sizeof(hci_evt_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_desc_tab[index].code == code)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
const struct hci_evt_desc_tag* hci_look_for_dbg_evt_desc(uint8_t subcode)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_dbg_desc_tab)/sizeof(hci_evt_dbg_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_dbg_desc_tab[index].code == subcode)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_dbg_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}
#endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
const struct hci_evt_desc_tag* hci_look_for_le_evt_desc(uint8_t subcode)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_le_desc_tab)/sizeof(hci_evt_le_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_le_desc_tab[index].code == subcode)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_le_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#endif //(HCI_PRESENT)


/// @} HCI
