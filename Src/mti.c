#include "mti.h"

uint8_t mti_packet[MTI_PACKET_LEN];
uint8_t mti_dma_buff[MTI_DMA_LEN];

int mti_cnt = 0;
int mti_flag = 0;
/* New Code Begin */
uint8_t mti_decode_flag = 0;
/* New Code End */

MTI_UNION mti_union;
MTI3 mti3;

void read_mti3_packet()
{
  /* New Code Begin */
  mti_decode_flag = 0;
  /* New Code End */
  get_packet();
  if(mti_flag)
  {
    mti3_decode();
    /* New Code Begin */
    mti_decode_flag = 1;
    /* New Code End */
    mti_flag = 0;
  }
}

void get_packet()
{
  static int now_ndt = 0, last_ndt = 0;
  static int rx_idx = 0, end, start = 0;
  
  last_ndt = now_ndt;
  now_ndt = MTI_DMA_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  
  end = (rx_idx + MTI_PACKET_LEN - 1) % MTI_DMA_LEN;
  
  if(last_ndt <= end && end < now_ndt)
  {
    mti_flag = 1;
    rx_idx = (end + 1) % MTI_DMA_LEN;
  }
  else if (last_ndt > now_ndt && (last_ndt <= end || end < now_ndt))
  {
    mti_flag = 1;
    rx_idx = (end + 1) % MTI_DMA_LEN;
  }
  else 
    return;
  
  
  if(mti_flag)
  {
    if(start < end)
    {
      memcpy(mti_packet, &mti_dma_buff[start], MTI_PACKET_LEN);
    }
    else if(end < start)
    {
      memcpy(mti_packet, &mti_dma_buff[start], MTI_DMA_LEN - start);
      memcpy(&mti_packet[MTI_DMA_LEN - start], mti_dma_buff, end + 1);
    }
    start = rx_idx;
  }
  
  return;
}

void mti3_decode()
{ 
  if(checksum() == 1)
  {
    /* euler angle */
    mti_union.byte[0] = mti_packet[10];
    mti_union.byte[1] = mti_packet[9];
    mti_union.byte[2] = mti_packet[8];
    mti_union.byte[3] = mti_packet[7];
    mti3.euler[ROLL] = mti_union.data;
    
    mti_union.byte[0] = mti_packet[14];
    mti_union.byte[1] = mti_packet[13];
    mti_union.byte[2] = mti_packet[12];
    mti_union.byte[3] = mti_packet[11]; 
    mti3.euler[PITCH] = mti_union.data;
       
    mti_union.byte[0] = mti_packet[18];
    mti_union.byte[1] = mti_packet[17];
    mti_union.byte[2] = mti_packet[16];
    mti_union.byte[3] = mti_packet[15];
    mti3.euler[YAW] = mti_union.data;
    
    
    /* gyro*/
    mti_union.byte[0] = mti_packet[25];
    mti_union.byte[1] = mti_packet[24];
    mti_union.byte[2] = mti_packet[23];
    mti_union.byte[3] = mti_packet[22];
    /* New Code Begin */
    //mti3.pqr[PQR_X] = mti_union.data;
    /* multiple "180/PI" for converting Radian to Degree(mti에서는 라디안 단위로 자이로 값 제공하므로 도 단위로 변환) */
    mti3.pqr[PQR_X] = mti_union.data * 180/PI;
    /* New Code Begin */
    
    mti_union.byte[0] = mti_packet[29];
    mti_union.byte[1] = mti_packet[28];
    mti_union.byte[2] = mti_packet[27];
    mti_union.byte[3] = mti_packet[26];
    /* New Code Begin */
    //mti3.pqr[PQR_Y] = mti_union.data;
    mti3.pqr[PQR_Y] = mti_union.data * 180/PI;
    /* New Code Begin */
    
    mti_union.byte[0] = mti_packet[33];
    mti_union.byte[1] = mti_packet[32];
    mti_union.byte[2] = mti_packet[31];
    mti_union.byte[3] = mti_packet[30]; 
    /* New Code Begin */
    //mti3.pqr[PQR_Z] = mti_union.data;
    mti3.pqr[PQR_Z] = mti_union.data * 180/PI;
    /* New Code End */
  }
  else
  {
    return;
  }
  
//  printf("mti3_euler_ROLL : %f\n\r", mti3.euler[ROLL]);  
//  printf("mti3_euler_PIRCH : %f\n\r", mti3.euler[PITCH]);
//  printf("mti3_euler_YAW : %f\n\r", mti3.euler[YAW]);
//  printf("\n\r");
    //printf("%.4f %.4f %.4f\n\r", mti3.euler[ROLL], mti3.euler[PITCH], mti3.euler[YAW]);
  
//  printf("mti3_pqr_PQR_X : %f\n\r", mti3.pqr[PQR_X]);  
//  printf("mti3_pqr_PQR_Y : %f\n\r", mti3.pqr[PQR_Y]);
//  printf("mti3_pqr_PQR_Z : %f\n\r", mti3.pqr[PQR_Z]);
//  printf("\n\r");
  printf("%.4f %.4f %.4f\n\r", mti3.pqr[0], mti3.pqr[1], mti3.pqr[2]);
}

int checksum()
{
  int i;
  uint16_t temp = 0;
  uint16_t checksum = 0;
  
  for(i = 1; i < MTI_PACKET_LEN; i++)
  {
      temp += mti_packet[i];
  }
  
  checksum = temp & 0x00ff;
  
  if(checksum == 0x0000)
    return 1;
  
  else
    return 0;
}
