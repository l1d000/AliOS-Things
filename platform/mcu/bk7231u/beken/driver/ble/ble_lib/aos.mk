NAME := ble_lib

$(NAME)_MBINS_TYPE := kernel
$(NAME)_VERSION    := 1.0.0
$(NAME)_SUMMARY    := 

$(NAME)_INCLUDES := . \
					ip/ahi/api \
					ip/ble/hl/api \
					ip/ble/hl/inc \
					ip/ble/hl/src/gap/gapc \
					ip/ble/hl/src/gap/gapm \
					ip/ble/hl/src/gap/smpc \
					ip/ble/hl/src/gap/smpm \
					ip/ble/hl/src/gap \
					ip/ble/hl/src/gatt/attc \
					ip/ble/hl/src/gatt/attm \
					ip/ble/hl/src/gatt/atts \
					ip/ble/hl/src/gatt \
					ip/ble/hl/src/gatt/gattc \
					ip/ble/hl/src/gatt/gattm \
					ip/ble/hl/src/l2c/l2cc \
					ip/ble/hl/src/l2c/l2cm \
					ip/ble/ll/src/em \
					ip/ble/ll/src/llc \
					ip/ble/ll/src/lld \
					ip/ble/ll/src/llm \
					ip/ble/ll/src/rwble \
					ip/ea/api \
					ip/em/api \
					ip/hci/api \
					ip/hci/src \
					modules/h4tl/api \
					modules/ke/api \
					modules/ke/src

$(NAME)_INCLUDES += ../../../ip/ke \
					../../../ip/lmac/src/rwnx \
					../../../ip/mac \
					../../../ip/umac/src/apm \
					../../../ip/lmac/src/scan \
					../../../ip/lmac/src/hal \
					../../../ip/lmac/src/mm \
					../../../ip/umac/src/me \
					../../../ip/umac/src/rc \
					../../../ip/lmac/src/sta \
					../../../ip/umac/src/rxu \
					../../../ip/lmac/src/rx \
					../../../ip/umac/src/bam \
					../../../ip/lmac/src/vif \
					../../../ip/lmac/src/chan \
					../../../ip/lmac/src/tx \
					../../../ip/lmac/src/rx/rxl \
					../../../ip/lmac/src/tx/txl \
					../../../ip/umac/src/scanu

$(NAME)_INCLUDES +=	../../../func/rwnx_intf \
					../../../driver/dma \
					../../../driver/common/reg \
					../../../driver/phy \
					../../../driver/uart \
					../../../driver/ble/ble_pub/modules/rwip/api \
					../../../driver/ble/ble_pub/modules/common/api \
					../../../driver/ble/ble_pub/plf/refip/src/arch/compiler \
					../../../driver/ble/ble_pub/plf/refip/src/arch \
					../../../driver/ble/ble_pub/plf/refip/src/arch/ll \
					../../../driver/ble/ble_pub/plf/refip/src/build/ble_full/reg/fw \
					../../../driver/ble/ble_pub/plf/refip/src/driver/reg \
					../../../driver/ble/ble_pub/ip/ble/profiles/sdp/api \
					../../../driver/ble/ble_pub/modules/dbg/api \
					../../../driver/ble/ble_pub/modules/app/api \
					../../../driver/ble/ble_pub/modules/app/src \
					../../../driver/ble/ble_pub/ip/ble/hl/inc
					
$(NAME)_SOURCES	:=  ip/ahi/src/ahi.c \
					ip/ahi/src/ahi_task.c \
					ip/ble/hl/src/gap/gapc/gapc.c \
					ip/ble/hl/src/gap/gapc/gapc_hci.c \
					ip/ble/hl/src/gap/gapc/gapc_sig.c \
					ip/ble/hl/src/gap/gapc/gapc_task.c \
					ip/ble/hl/src/gap/gapm/gapm.c \
					ip/ble/hl/src/gap/gapm/gapm_hci.c \
					ip/ble/hl/src/gap/gapm/gapm_task.c \
					ip/ble/hl/src/gap/gapm/gapm_util.c \
					ip/ble/hl/src/gap/smpc/smpc.c \
					ip/ble/hl/src/gap/smpc/smpc_api.c \
					ip/ble/hl/src/gap/smpc/smpc_crypto.c \
					ip/ble/hl/src/gap/smpc/smpc_util.c \
					ip/ble/hl/src/gap/smpm/smpm_api.c \
					ip/ble/hl/src/gatt/attc/attc.c \
					ip/ble/hl/src/gatt/attm/attm.c \
					ip/ble/hl/src/gatt/attm/attm_db.c \
					ip/ble/hl/src/gatt/atts/atts.c \
					ip/ble/hl/src/gatt/gattc/gattc.c \
					ip/ble/hl/src/gatt/gattc/gattc_task.c \
					ip/ble/hl/src/gatt/gattm/gattm.c \
					ip/ble/hl/src/gatt/gattm/gattm_task.c \
					ip/ble/hl/src/l2c/l2cc/l2cc.c \
					ip/ble/hl/src/l2c/l2cc/l2cc_lecb.c \
					ip/ble/hl/src/l2c/l2cc/l2cc_pdu.c \
					ip/ble/hl/src/l2c/l2cc/l2cc_sig.c \
					ip/ble/hl/src/l2c/l2cc/l2cc_task.c \
					ip/ble/hl/src/l2c/l2cm/l2cm.c \
					ip/ble/hl/src/prf/prf_utils.c \
					ip/ble/hl/src/rwble_hl/rwble_hl.c \
					ip/ble/ll/src/em/em_buf.c \
					ip/ble/ll/src/llc/llc.c \
					ip/ble/ll/src/llc/llc_ch_asses.c \
					ip/ble/ll/src/llc/llc_hci.c \
					ip/ble/ll/src/llc/llc_llcp.c \
					ip/ble/ll/src/llc/llc_task.c \
					ip/ble/ll/src/llc/llc_util.c \
					ip/ble/ll/src/lld/lld.c \
					ip/ble/ll/src/lld/lld_evt.c \
					ip/ble/ll/src/lld/lld_pdu.c \
					ip/ble/ll/src/lld/lld_sleep.c \
					ip/ble/ll/src/lld/lld_util.c \
					ip/ble/ll/src/lld/lld_wlcoex.c \
					ip/ble/ll/src/llm/llm.c \
					ip/ble/ll/src/llm/llm_hci.c \
					ip/ble/ll/src/llm/llm_task.c \
					ip/ble/ll/src/llm/llm_util.c \
					ip/ble/ll/src/rwble/rwble.c \
					ip/ea/src/ea.c \
					ip/hci/src/hci.c \
					ip/hci/src/hci_fc.c \
					ip/hci/src/hci_msg.c \
					ip/hci/src/hci_tl.c \
					ip/hci/src/hci_util.c \
					modules/h4tl/src/h4tl.c \
					modules/ke/src/kernel.c \
					modules/ke/src/kernel_event.c \
					modules/ke/src/kernel_mem.c \
					modules/ke/src/kernel_msg.c \
					modules/ke/src/kernel_queue.c \
					modules/ke/src/kernel_task.c \
					modules/ke/src/kernel_timer.c \
					plf/refip/src/arch/main/arch_main.c
