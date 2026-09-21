#include "slcan.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static char usbRxBuffer[128];
static uint16_t usbIndex = 0;
static bool channelOpen = false;

/* Send text through USB CDC */
static void SLCAN_Send(const char *text)
{
    uint16_t length = (uint16_t)strlen(text);

    for (uint8_t attempt = 0; attempt < 10; attempt++)
    {
        if (CDC_Transmit_FS((uint8_t *)text, length) == USBD_OK)
        {
            return;
        }

        HAL_Delay(1);
    }
}

/* Convert one hexadecimal character to a number */
static int8_t HexToValue(char c)
{
    if ((c >= '0') && (c <= '9'))
    {
        return (int8_t)(c - '0');
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        return (int8_t)(c - 'A' + 10);
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        return (int8_t)(c - 'a' + 10);
    }

    return -1;
}

/* Convert SLCAN DLC to STM32 FDCAN DLC */
static uint32_t LengthToDLC(uint8_t length)
{
    switch (length)
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

/* Convert STM32 FDCAN DLC to a normal byte length */
static uint8_t DLCToLength(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}

/* Process one complete SLCAN command */
static void SLCAN_ProcessCommand(char *command)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8] = {0};

    switch (command[0])
    {
        /* Set bitrate: S6 means 500 kbit/s */
        case 'S':
        {
            if (command[1] == '6' && command[2] == '\0')
            {
                SLCAN_Send("\r");
            }
            else
            {
                SLCAN_Send("\a");
            }

            break;
        }

        /* Open CAN channel */
        case 'O':
        {
            channelOpen = true;
            SLCAN_Send("\r");
            break;
        }

        /* Close CAN channel */
        case 'C':
        {
            channelOpen = false;
            SLCAN_Send("\r");
            break;
        }

        /* Firmware version */
        case 'V':
        {
            SLCAN_Send("V0101\r");
            break;
        }

        /* Serial number */
        case 'N':
        {
            SLCAN_Send("N0001\r");
            break;
        }

        /* Transmit standard 11-bit CAN data frame */
        case 't':
        {
            if (!channelOpen)
            {
                SLCAN_Send("\a");
                break;
            }

            /* Format: tIIILDD...
             * t = standard frame
             * III = 3-digit CAN ID
             * L = data length
             */

            if (strlen(command) < 5)
            {
                SLCAN_Send("\a");
                break;
            }

            int8_t id0 = HexToValue(command[1]);
            int8_t id1 = HexToValue(command[2]);
            int8_t id2 = HexToValue(command[3]);
            int8_t dlcValue = HexToValue(command[4]);

            if ((id0 < 0) || (id1 < 0) || (id2 < 0) ||
                (dlcValue < 0) || (dlcValue > 8))
            {
                SLCAN_Send("\a");
                break;
            }

            uint32_t identifier =
                ((uint32_t)id0 << 8) |
                ((uint32_t)id1 << 4) |
                (uint32_t)id2;

            uint8_t dataLength = (uint8_t)dlcValue;

            if (strlen(command) != (uint32_t)(5 + (dataLength * 2)))
            {
                SLCAN_Send("\a");
                break;
            }

            for (uint8_t i = 0; i < dataLength; i++)
            {
                int8_t high = HexToValue(command[5 + (i * 2)]);
                int8_t low = HexToValue(command[6 + (i * 2)]);

                if ((high < 0) || (low < 0))
                {
                    SLCAN_Send("\a");
                    return;
                }

                txData[i] = (uint8_t)((high << 4) | low);
            }

            txHeader.Identifier = identifier;
            txHeader.IdType = FDCAN_STANDARD_ID;
            txHeader.TxFrameType = FDCAN_DATA_FRAME;
            txHeader.DataLength = LengthToDLC(dataLength);
            txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            txHeader.BitRateSwitch = FDCAN_BRS_OFF;
            txHeader.FDFormat = FDCAN_CLASSIC_CAN;
            txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            txHeader.MessageMarker = 0;

            if (HAL_FDCAN_AddMessageToTxFifoQ(
                    &hfdcan1,
                    &txHeader,
                    txData) == HAL_OK)
            {
                SLCAN_Send("\r");
            }
            else
            {
                SLCAN_Send("\a");
            }

            break;
        }

        default:
        {
            SLCAN_Send("\a");
            break;
        }
    }
}

void SLCAN_Init(void)
{
    usbIndex = 0;
    channelOpen = false;
    memset(usbRxBuffer, 0, sizeof(usbRxBuffer));
}

void SLCAN_Process(void)
{
    /* Reserved for future background processing */
}

void SLCAN_ProcessChar(uint8_t c)
{
    /* SLCAN commands finish with carriage return */
    if (c == '\r')
    {
        usbRxBuffer[usbIndex] = '\0';

        if (usbIndex > 0)
        {
            SLCAN_ProcessCommand(usbRxBuffer);
        }

        usbIndex = 0;
        memset(usbRxBuffer, 0, sizeof(usbRxBuffer));
        return;
    }

    /* Ignore newline characters */
    if (c == '\n')
    {
        return;
    }

    if (usbIndex < (sizeof(usbRxBuffer) - 1))
    {
        usbRxBuffer[usbIndex] = (char)c;
        usbIndex++;
    }
    else
    {
        usbIndex = 0;
        memset(usbRxBuffer, 0, sizeof(usbRxBuffer));
        SLCAN_Send("\a");
    }
}

void SLCAN_USB_Receive(uint8_t *Buf, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++)
    {
        SLCAN_ProcessChar(Buf[i]);
    }
}

void SLCAN_CANRx(FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    if (!channelOpen)
    {
        return;
    }

    if (header->IdType != FDCAN_STANDARD_ID)
    {
        return;
    }

    uint8_t length = DLCToLength(header->DataLength);
    char message[32];
    int position = 0;

    position += snprintf(
        &message[position],
        sizeof(message) - position,
        "t%03lX%1X",
        header->Identifier,
        length);

    for (uint8_t i = 0; i < length; i++)
    {
        position += snprintf(
            &message[position],
            sizeof(message) - position,
            "%02X",
            data[i]);
    }

    snprintf(
        &message[position],
        sizeof(message) - position,
        "\r");

    SLCAN_Send(message);
}
