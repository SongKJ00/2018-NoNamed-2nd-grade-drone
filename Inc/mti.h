#ifndef __MTI_H
#define __MTI_H

#include <string.h>
#include "stm32f4xx_hal.h"
#include "usart.h"

#define MTI_DMA_LEN 1024 
#define MTI_PACKET_LEN 50

#define ROLL 0
#define PITCH 1
#define YAW 2

#define PQR_X 0
#define PQR_Y 1
#define PQR_Z 2

typedef union
{
  float data;
  uint8_t byte[4];
}MTI_UNION;


typedef struct
{
  float euler[3];
  float pqr[3];
}MTI3;

void read_mti3_packet();
void mti3_decode();
void get_packet();
int checksum();

extern uint8_t mti_dma_buff[MTI_DMA_LEN];
extern MTI3 mti3;

#endif
