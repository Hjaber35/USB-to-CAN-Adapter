#include "usbd_gsusb.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "main.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint32_t GSUSB_DLCToFDCAN(uint8_t dlc)
{
    switch (dlc)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_0;
    }
}

/*
 * First stage:
 * - Defines the vendor-specific USB interface
 * - Opens the bulk IN and OUT endpoints
 * - Allows the project to compile
 *
 * We will add Linux control requests and CAN-frame handling next.
 */
static uint8_t GSUSB_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t GSUSB_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t GSUSB_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t GSUSB_Setup(USBD_HandleTypeDef *pdev,
                           USBD_SetupReqTypedef *req);
static uint8_t GSUSB_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t GSUSB_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t GSUSB_QueueIN(const gs_host_frame_t *frame);

static uint8_t *GSUSB_GetFSConfigDescriptor(uint16_t *length);
static uint8_t *GSUSB_GetHSConfigDescriptor(uint16_t *length);
static uint8_t *GSUSB_GetOtherSpeedConfigDescriptor(uint16_t *length);
static uint8_t *GSUSB_GetDeviceQualifierDescriptor(uint16_t *length);

static uint8_t gsusb_rx_buffer[GSUSB_FS_MAX_PACKET_SIZE];
static uint8_t gsusb_tx_buffer[GSUSB_FS_MAX_PACKET_SIZE];
static gs_host_config_t gsusb_host_config;
static gs_device_bittiming_t gsusb_bittiming;
static gs_device_mode_t gsusb_mode;
static uint8_t gsusb_pending_request = 0xFFU;
static volatile uint8_t gsusb_in_busy = 0;

#define GSUSB_IN_QUEUE_SIZE 8

static gs_host_frame_t gsusb_in_queue[GSUSB_IN_QUEUE_SIZE];
static volatile uint8_t gsusb_in_head = 0;
static volatile uint8_t gsusb_in_tail = 0;

static gs_device_config_t gsusb_device_config =
{
    .reserved1 = 0,
    .reserved2 = 0,
    .reserved3 = 0,
    .interface_count = 0,   /* One CAN channel: channel 0 */
    .software_version = 1,
    .hardware_version = 1
};

static gs_device_bt_const_t gsusb_bt_const =
{
    .feature = 0,
    .fclk_can = 16000000UL,
    .tseg1_min = 2,
    .tseg1_max = 256,
    .tseg2_min = 2,
    .tseg2_max = 128,
    .sjw_max = 128,
    .brp_min = 1,
    .brp_max = 512,
    .brp_inc = 1
};

/*
 * USB class callback table.
 */
USBD_ClassTypeDef USBD_GSUSB =
{
    GSUSB_Init,
    GSUSB_DeInit,
    GSUSB_Setup,
    NULL,
	GSUSB_EP0_RxReady,
    GSUSB_DataIn,
    GSUSB_DataOut,
    NULL,
    NULL,
    NULL,
    GSUSB_GetHSConfigDescriptor,
    GSUSB_GetFSConfigDescriptor,
    GSUSB_GetOtherSpeedConfigDescriptor,
    GSUSB_GetDeviceQualifierDescriptor
};

/*
 * Vendor-specific USB configuration:
 *
 * Configuration descriptor: 9 bytes
 * Interface descriptor:     9 bytes
 * Bulk OUT endpoint:        7 bytes
 * Bulk IN endpoint:         7 bytes
 *
 * Total: 32 bytes
 */
__ALIGN_BEGIN static uint8_t gsusb_config_descriptor[
    GSUSB_CONFIG_DESC_SIZE] __ALIGN_END =
{
    /* Configuration descriptor */
    0x09,
    USB_DESC_TYPE_CONFIGURATION,
    LOBYTE(GSUSB_CONFIG_DESC_SIZE),
    HIBYTE(GSUSB_CONFIG_DESC_SIZE),
    0x01,                       /* One interface */
    0x01,                       /* Configuration value */
    0x00,                       /* No configuration string */
    0x80,                       /* Bus powered */
    0x32,                       /* 100 mA */

    /* Vendor-specific interface descriptor */
    0x09,
    USB_DESC_TYPE_INTERFACE,
    0x00,                       /* Interface number */
    0x00,                       /* Alternate setting */
    0x02,                       /* Two endpoints */
    0xFF,                       /* Vendor-specific class */
    0xFF,                       /* Vendor-specific subclass */
    0xFF,                       /* Vendor-specific protocol */
    0x00,                       /* No interface string */

    /* Bulk OUT endpoint */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    GSUSB_OUT_EP,
    0x02,                       /* Bulk endpoint */
    LOBYTE(GSUSB_FS_MAX_PACKET_SIZE),
    HIBYTE(GSUSB_FS_MAX_PACKET_SIZE),
    0x00,

    /* Bulk IN endpoint */
    0x07,
    USB_DESC_TYPE_ENDPOINT,
    GSUSB_IN_EP,
    0x02,                       /* Bulk endpoint */
    LOBYTE(GSUSB_FS_MAX_PACKET_SIZE),
    HIBYTE(GSUSB_FS_MAX_PACKET_SIZE),
    0x00
};

__ALIGN_BEGIN static uint8_t gsusb_device_qualifier_descriptor[
    USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00
};

static uint8_t GSUSB_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_LL_OpenEP(
        pdev,
        GSUSB_IN_EP,
        USBD_EP_TYPE_BULK,
        GSUSB_FS_MAX_PACKET_SIZE);

    pdev->ep_in[GSUSB_IN_EP & 0x0FU].is_used = 1U;

    USBD_LL_OpenEP(
        pdev,
        GSUSB_OUT_EP,
        USBD_EP_TYPE_BULK,
        GSUSB_FS_MAX_PACKET_SIZE);

    pdev->ep_out[GSUSB_OUT_EP & 0x0FU].is_used = 1U;

    memset(gsusb_rx_buffer, 0, sizeof(gsusb_rx_buffer));
    memset(gsusb_tx_buffer, 0, sizeof(gsusb_tx_buffer));

    gsusb_in_busy = 0;
    gsusb_in_head = 0;
    gsusb_in_tail = 0;

    memset(gsusb_in_queue, 0, sizeof(gsusb_in_queue));

    USBD_LL_PrepareReceive(
        pdev,
        GSUSB_OUT_EP,
        gsusb_rx_buffer,
        GSUSB_FS_MAX_PACKET_SIZE);

    return USBD_OK;
}

static uint8_t GSUSB_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    USBD_LL_CloseEP(pdev, GSUSB_IN_EP);
    pdev->ep_in[GSUSB_IN_EP & 0x0FU].is_used = 0U;

    USBD_LL_CloseEP(pdev, GSUSB_OUT_EP);
    pdev->ep_out[GSUSB_OUT_EP & 0x0FU].is_used = 0U;

    return USBD_OK;
}

static uint8_t GSUSB_Setup(USBD_HandleTypeDef *pdev,
                           USBD_SetupReqTypedef *req)
{
    uint16_t length;

    if ((req->bmRequest & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_VENDOR)
    {
        USBD_CtlError(pdev, req);
        return USBD_FAIL;
    }

    switch (req->bRequest)
    {
        case GS_USB_BREQ_HOST_FORMAT:
        {
            if (req->wLength != sizeof(gs_host_config_t))
            {
                USBD_CtlError(pdev, req);
                return USBD_FAIL;
            }

            gsusb_pending_request = GS_USB_BREQ_HOST_FORMAT;

            USBD_CtlPrepareRx(
                pdev,
                (uint8_t *)&gsusb_host_config,
                sizeof(gsusb_host_config));

            return USBD_OK;
        }

        case GS_USB_BREQ_DEVICE_CONFIG:
        {
            length = sizeof(gsusb_device_config);

            if (req->wLength < length)
            {
                length = req->wLength;
            }

            USBD_CtlSendData(
                pdev,
                (uint8_t *)&gsusb_device_config,
                length);

            return USBD_OK;
        }

        case GS_USB_BREQ_BT_CONST:
        {
            length = sizeof(gsusb_bt_const);

            if (req->wLength < length)
            {
                length = req->wLength;
            }

            USBD_CtlSendData(
                pdev,
                (uint8_t *)&gsusb_bt_const,
                length);

            return USBD_OK;
        }
        case GS_USB_BREQ_BITTIMING:
        {
            if (req->wLength != sizeof(gs_device_bittiming_t))
            {
                USBD_CtlError(pdev, req);
                return USBD_FAIL;
            }

            gsusb_pending_request = GS_USB_BREQ_BITTIMING;

            USBD_CtlPrepareRx(
                pdev,
                (uint8_t *)&gsusb_bittiming,
                sizeof(gsusb_bittiming));

            return USBD_OK;
        }

        case GS_USB_BREQ_MODE:
        {
            if (req->wLength != sizeof(gs_device_mode_t))
            {
                USBD_CtlError(pdev, req);
                return USBD_FAIL;
            }

            gsusb_pending_request = GS_USB_BREQ_MODE;

            USBD_CtlPrepareRx(
                pdev,
                (uint8_t *)&gsusb_mode,
                sizeof(gsusb_mode));

            return USBD_OK;
        }

        default:
        {
            USBD_CtlError(pdev, req);
            return USBD_FAIL;
        }
    }
}

static uint8_t GSUSB_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    (void)pdev;

    switch (gsusb_pending_request)
    {
        case GS_USB_BREQ_HOST_FORMAT:
        {
            if (gsusb_host_config.byte_order != 0x0000BEEFUL)
            {
                gsusb_pending_request = 0xFFU;
                return USBD_FAIL;
            }

            break;
        }

        case GS_USB_BREQ_BITTIMING:
        {
            /* Stop CAN before changing timing */
            HAL_FDCAN_Stop(&hfdcan1);

            if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK)
            {
                gsusb_pending_request = 0xFFU;
                return USBD_FAIL;
            }

            hfdcan1.Init.NominalPrescaler =
                gsusb_bittiming.brp;

            hfdcan1.Init.NominalSyncJumpWidth =
                gsusb_bittiming.sjw;

            hfdcan1.Init.NominalTimeSeg1 =
                gsusb_bittiming.prop_seg +
                gsusb_bittiming.phase_seg1;

            hfdcan1.Init.NominalTimeSeg2 =
                gsusb_bittiming.phase_seg2;

            if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
            {
                gsusb_pending_request = 0xFFU;
                return USBD_FAIL;
            }

            break;
        }

        case GS_USB_BREQ_MODE:
        {
            if (gsusb_mode.mode == GS_CAN_MODE_RESET)
            {
                if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
                {
                    gsusb_pending_request = 0xFFU;
                    return USBD_FAIL;
                }
            }
            else if (gsusb_mode.mode == GS_CAN_MODE_START)
            {
                if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
                {
                    gsusb_pending_request = 0xFFU;
                    return USBD_FAIL;
                }
            }
            else
            {
                gsusb_pending_request = 0xFFU;
                return USBD_FAIL;
            }

            break;
        }

        default:
            break;
    }

    gsusb_pending_request = 0xFFU;
    return USBD_OK;
}

static uint8_t GSUSB_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (epnum == (GSUSB_IN_EP & 0x0FU))
    {
        /* The frame at tail finished transmitting */
        gsusb_in_tail =
            (uint8_t)((gsusb_in_tail + 1U) % GSUSB_IN_QUEUE_SIZE);

        /* More frames waiting? Send the next one */
        if (gsusb_in_tail != gsusb_in_head)
        {
            memcpy(
                gsusb_tx_buffer,
                &gsusb_in_queue[gsusb_in_tail],
                sizeof(gs_host_frame_t));

            gsusb_in_busy = 1;

            if (USBD_LL_Transmit(
                    pdev,
                    GSUSB_IN_EP,
                    gsusb_tx_buffer,
                    sizeof(gs_host_frame_t)) != USBD_OK)
            {
                gsusb_in_busy = 0;
            }
        }
        else
        {
            gsusb_in_busy = 0;
        }
    }

    return USBD_OK;
}

static uint8_t GSUSB_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    uint32_t rxLength =
        USBD_LL_GetRxDataSize(pdev, epnum);

    if (rxLength >= sizeof(gs_host_frame_t))
    {
        gs_host_frame_t *frame =
            (gs_host_frame_t *)gsusb_rx_buffer;

        uint32_t canId = frame->can_id;

        if (frame->can_dlc <= 8 &&
            frame->channel == 0)
        {
            if (canId & GS_CAN_EFF_FLAG)
            {
                txHeader.Identifier =
                    canId & GS_CAN_EFF_MASK;

                txHeader.IdType =
                    FDCAN_EXTENDED_ID;
            }
            else
            {
                txHeader.Identifier =
                    canId & GS_CAN_SFF_MASK;

                txHeader.IdType =
                    FDCAN_STANDARD_ID;
            }

            if (canId & GS_CAN_RTR_FLAG)
            {
                txHeader.TxFrameType =
                    FDCAN_REMOTE_FRAME;
            }
            else
            {
                txHeader.TxFrameType =
                    FDCAN_DATA_FRAME;
            }

            txHeader.DataLength =
                GSUSB_DLCToFDCAN(frame->can_dlc);

            txHeader.ErrorStateIndicator =
                FDCAN_ESI_ACTIVE;

            txHeader.BitRateSwitch =
                FDCAN_BRS_OFF;

            txHeader.FDFormat =
                FDCAN_CLASSIC_CAN;

            txHeader.TxEventFifoControl =
                FDCAN_NO_TX_EVENTS;

            txHeader.MessageMarker = 0;

            if (HAL_FDCAN_AddMessageToTxFifoQ(
                    &hfdcan1,
                    &txHeader,
                    frame->data) == HAL_OK)
            {
                /*
                 * Return the frame to Linux as TX echo.
                 *
                 * Linux uses echo_id to release the matching
                 * transmitted CAN packet.
                 */
            	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
            	GSUSB_QueueIN(frame);
        }
    }
}
    /* Arm OUT endpoint again for the next USB frame */
    USBD_LL_PrepareReceive(
        pdev,
        GSUSB_OUT_EP,
        gsusb_rx_buffer,
        GSUSB_FS_MAX_PACKET_SIZE);

    return USBD_OK;
}
static uint8_t GSUSB_QueueIN(const gs_host_frame_t *frame)
{
    uint8_t next =
        (uint8_t)((gsusb_in_head + 1U) % GSUSB_IN_QUEUE_SIZE);

    if (next == gsusb_in_tail)
    {
        return USBD_FAIL;   /* queue full */
    }

    memcpy(
        &gsusb_in_queue[gsusb_in_head],
        frame,
        sizeof(gs_host_frame_t));

    gsusb_in_head = next;

    if (!gsusb_in_busy)
    {
        gsusb_in_busy = 1;

        memcpy(
            gsusb_tx_buffer,
            &gsusb_in_queue[gsusb_in_tail],
            sizeof(gs_host_frame_t));

        if (USBD_LL_Transmit(
                &hUsbDeviceFS,
                GSUSB_IN_EP,
                gsusb_tx_buffer,
                sizeof(gs_host_frame_t)) != USBD_OK)
        {
            gsusb_in_busy = 0;
            return USBD_FAIL;
        }
    }

    return USBD_OK;
}
static uint8_t *GSUSB_GetFSConfigDescriptor(uint16_t *length)
{
    *length = sizeof(gsusb_config_descriptor);
    return gsusb_config_descriptor;
}

static uint8_t *GSUSB_GetHSConfigDescriptor(uint16_t *length)
{
    *length = sizeof(gsusb_config_descriptor);
    return gsusb_config_descriptor;
}

static uint8_t *GSUSB_GetOtherSpeedConfigDescriptor(uint16_t *length)
{
    *length = sizeof(gsusb_config_descriptor);
    return gsusb_config_descriptor;
}

static uint8_t *GSUSB_GetDeviceQualifierDescriptor(uint16_t *length)
{
    *length = sizeof(gsusb_device_qualifier_descriptor);
    return gsusb_device_qualifier_descriptor;
}

uint8_t GSUSB_SendCANFrame(uint32_t identifier,
                           uint32_t id_type,
                           uint32_t frame_type,
                           uint8_t dlc,
                           const uint8_t *data)
{
    gs_host_frame_t frame = {0};

    if (dlc > 8)
    {
        return USBD_FAIL;
    }

    frame.echo_id = 0xFFFFFFFFUL;
    frame.channel = 0;
    frame.flags = 0;
    frame.reserved = 0;
    frame.can_dlc = dlc;

    if (id_type == FDCAN_EXTENDED_ID)
    {
        frame.can_id =
            (identifier & GS_CAN_EFF_MASK) |
            GS_CAN_EFF_FLAG;
    }
    else
    {
        frame.can_id =
            identifier & GS_CAN_SFF_MASK;
    }

    if (frame_type == FDCAN_REMOTE_FRAME)
    {
        frame.can_id |= GS_CAN_RTR_FLAG;
    }

    if (data != NULL && frame_type != FDCAN_REMOTE_FRAME)
    {
        memcpy(frame.data, data, dlc);
    }

    return GSUSB_QueueIN(&frame);
}
