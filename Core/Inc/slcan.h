#ifndef SLCAN_H
#define SLCAN_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void SLCAN_Init(void);
void SLCAN_Process(void);

void SLCAN_ProcessChar(uint8_t c);
void SLCAN_USB_Receive(uint8_t *Buf, uint32_t Len);
void SLCAN_CANRx(FDCAN_RxHeaderTypeDef *hdr, uint8_t *data);

#endif
