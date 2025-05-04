#ifndef __USART3_WabCam_H
#define __USART3_WabCam_H

#include <stdio.h>

extern uint8_t Serial_TxPacket[4];				//定义发送数据包数组，数据包格式：FF 01 02 03 04 FE
extern uint8_t Serial_RxPacket[4];				//定义接收数据包数组
extern uint8_t USART3_Serial_RxFlag;	

void USART3_Serial_Init(void);
void USART3_Serial_SendByte(uint8_t Byte);
void USART3_Serial_SendArray(uint8_t *Array, uint16_t Length);
void USART3_Serial_SendString(char *String);
void USART3_Serial_SendNumber(uint32_t Number, uint8_t Length);
void USART3_Serial_Printf(char *format, ...);

uint8_t USART3_Serial_GetRxFlag(void);

#endif
