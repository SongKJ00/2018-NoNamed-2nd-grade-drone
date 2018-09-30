#ifndef __SBUS_H
#define __SBUS_H

#include <string.h>
#include "usart.h"
#include "stm32f4xx_hal.h"

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3 
#define CH5 4 
#define CH6 5 
#define CH7 6 
#define CH8 7 
#define CH9 8
#define CH10 9 
#define CH11 10 
#define CH12 11

#define CH_SIZE 12

#define SBUS_DMA_LEN 1024
#define SBUS_PACKET_LEN 25

#define START_BYTE 0x0F
#define END_BYTE   0x04

void read_sbus_packet();
void get_sbus_packet();
void sbus_decode();

extern uint8_t sbus_dma_buff[SBUS_DMA_LEN];
extern int sbus_ch_data[CH_SIZE];

#endif
