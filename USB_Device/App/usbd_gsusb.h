#ifndef __USBD_GSUSB_H
#define __USBD_GSUSB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"
#include <stdint.h>

/* USB bulk endpoints */
#define GSUSB_IN_EP                    0x81U
#define GSUSB_OUT_EP                   0x01U

#define GSUSB_FS_MAX_PACKET_SIZE       64U
#define GSUSB_CONFIG_DESC_SIZE         32U

/* GS_USB vendor requests */
#define GS_USB_BREQ_HOST_FORMAT        0U
#define GS_USB_BREQ_BITTIMING          1U
#define GS_USB_BREQ_MODE               2U
#define GS_USB_BREQ_BERR               3U
#define GS_USB_BREQ_BT_CONST           4U
#define GS_USB_BREQ_DEVICE_CONFIG      5U
#define GS_USB_BREQ_TIMESTAMP          6U
#define GS_USB_BREQ_IDENTIFY           7U
#define GS_USB_BREQ_GET_USER_ID        8U
#define GS_USB_BREQ_SET_USER_ID        9U
#define GS_USB_BREQ_DATA_BITTIMING     10U
#define GS_USB_BREQ_BT_CONST_EXT       11U

/* CAN operating modes */
#define GS_CAN_MODE_RESET              0U
#define GS_CAN_MODE_START              1U

/* Mode flags */
#define GS_CAN_MODE_NORMAL             0U
#define GS_CAN_MODE_LISTEN_ONLY        (1UL << 0)
#define GS_CAN_MODE_LOOP_BACK          (1UL << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE      (1UL << 2)
#define GS_CAN_MODE_ONE_SHOT           (1UL << 3)
#define GS_CAN_MODE_HW_TIMESTAMP       (1UL << 4)
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE (1UL << 7)
#define GS_CAN_MODE_FD                 (1UL << 8)
#define GS_CAN_MODE_BERR_REPORTING     (1UL << 12)

/* Device feature flags */
#define GS_CAN_FEATURE_LISTEN_ONLY     (1UL << 0)
#define GS_CAN_FEATURE_LOOP_BACK       (1UL << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE   (1UL << 2)
#define GS_CAN_FEATURE_ONE_SHOT        (1UL << 3)
#define GS_CAN_FEATURE_HW_TIMESTAMP    (1UL << 4)
#define GS_CAN_FEATURE_IDENTIFY        (1UL << 5)
#define GS_CAN_FEATURE_USER_ID         (1UL << 6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1UL << 7)
#define GS_CAN_FEATURE_FD              (1UL << 8)
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX (1UL << 9)
#define GS_CAN_FEATURE_BT_CONST_EXT    (1UL << 10)
#define GS_CAN_FEATURE_TERMINATION     (1UL << 11)
#define GS_CAN_FEATURE_BERR_REPORTING  (1UL << 12)
#define GS_CAN_FEATURE_GET_STATE       (1UL << 13)

/* Linux CAN identifier flags */
#define GS_CAN_EFF_FLAG                0x80000000UL
#define GS_CAN_RTR_FLAG                0x40000000UL
#define GS_CAN_ERR_FLAG                0x20000000UL

#define GS_CAN_EFF_MASK                0x1FFFFFFFUL
#define GS_CAN_SFF_MASK                0x000007FFUL

/*
 * Structures sent over USB must have exact layouts.
 * __attribute__((packed)) prevents compiler padding.
 */

typedef struct __attribute__((packed))
{
    uint32_t byte_order;
} gs_host_config_t;

typedef struct __attribute__((packed))
{
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t interface_count;
    uint32_t software_version;
    uint32_t hardware_version;
} gs_device_config_t;

typedef struct __attribute__((packed))
{
    uint32_t feature;
    uint32_t fclk_can;
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} gs_device_bt_const_t;

typedef struct __attribute__((packed))
{
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} gs_device_bittiming_t;

typedef struct __attribute__((packed))
{
    uint32_t mode;
    uint32_t flags;
} gs_device_mode_t;

typedef struct __attribute__((packed))
{
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[8];
} gs_host_frame_t;

/* USB class object registered by usb_device.c */
extern USBD_ClassTypeDef USBD_GSUSB;

/* Called when an FDCAN frame is received */
uint8_t GSUSB_SendCANFrame(uint32_t identifier,
                           uint32_t id_type,
                           uint32_t frame_type,
                           uint8_t dlc,
                           const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_GSUSB_H */
