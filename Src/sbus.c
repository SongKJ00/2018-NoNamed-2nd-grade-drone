#include "sbus.h"

int sbus_flag = 0;
uint8_t sbus_dma_buff[SBUS_DMA_LEN];
uint8_t sbus_packet_buffer[SBUS_PACKET_LEN];

int sbus_ch_data[CH_SIZE];

void read_sbus_packet()
{
    get_sbus_packet();
    if(sbus_flag)
    {
        sbus_decode();
        sbus_flag = 0;
    }
}

void get_sbus_packet()
{
    static int now_ndt = 0, last_ndt = 0;
    static int head = 0, end = SBUS_PACKET_LEN - 1;
    
    last_ndt = now_ndt;
    now_ndt = SBUS_DMA_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    
    if(last_ndt <= end && end < now_ndt)
    {
        sbus_flag = 1;
    }
    else if(last_ndt > now_ndt && (last_ndt <= end || end < now_ndt))
    {
        sbus_flag = 1;
    }
    else
      return;
    
    if(sbus_flag && (sbus_dma_buff[head] == START_BYTE && (sbus_dma_buff[end] & 0x04) == END_BYTE))
    {
        if(head < end)
        {
            memcpy(sbus_packet_buffer, &sbus_dma_buff[head], SBUS_PACKET_LEN);	
        }
        else if(head > end)
        {
            memcpy(sbus_packet_buffer, &sbus_dma_buff[head], SBUS_DMA_LEN - head);
            memcpy(&sbus_packet_buffer[SBUS_DMA_LEN-head], sbus_dma_buff, end + 1);
        }
        head = end;
        end = (end + SBUS_PACKET_LEN) % SBUS_DMA_LEN;
    }
    else 
      return;
}

void sbus_decode()
{
	sbus_ch_data[CH1]  = (uint32_t)(sbus_packet_buffer[1]+((sbus_packet_buffer[2]&0x07)<<8));  //yaw-회전 
	sbus_ch_data[CH2]  = (uint32_t)(((sbus_packet_buffer[2]&0xf8)>>3)+((sbus_packet_buffer[3]&0x3f)<<5)); //pitch-앞뒤 
	sbus_ch_data[CH3]  = (uint32_t)(((sbus_packet_buffer[3]&0xc0)>>6)+(sbus_packet_buffer[4]<<2)+((sbus_packet_buffer[5]&0x01)<<10));  //motor
	sbus_ch_data[CH4]  = (uint32_t)(((sbus_packet_buffer[5]&0xfe)>>1)+((sbus_packet_buffer[6]&0x0f)<<7));  //roll-좌우
	sbus_ch_data[CH5]  = (uint32_t)(((sbus_packet_buffer[6]&0xf0)>>4)+((sbus_packet_buffer[7]&0x7f)<<4));
	sbus_ch_data[CH6]  = (uint32_t)(((sbus_packet_buffer[7]&0x80)>>7)+(sbus_packet_buffer[8]<<1)+((sbus_packet_buffer[9]&0x03)<<9));
	sbus_ch_data[CH7]  = (uint32_t)(((sbus_packet_buffer[9]&0xfc)>>2)+((sbus_packet_buffer[10]&0x1f)<<6));
	sbus_ch_data[CH8]  = (uint32_t)(((sbus_packet_buffer[10]&0xe0)>>5)+(sbus_packet_buffer[11]<<3));
	sbus_ch_data[CH9]  = (uint32_t)(sbus_packet_buffer[12]+((sbus_packet_buffer[13]&0x07)<<8));                                     
	sbus_ch_data[CH10] = (uint32_t)(((sbus_packet_buffer[13]&0xf8)>>3)+((sbus_packet_buffer[14]&0x3f)<<5));
	sbus_ch_data[CH11] = (uint32_t)(((sbus_packet_buffer[14]&0xc0)>>6)+(sbus_packet_buffer[15]<<2)+((sbus_packet_buffer[16]&0x01)<<10));
	sbus_ch_data[CH12] = (uint32_t)(((sbus_packet_buffer[16]&0xfe)>>1)+((sbus_packet_buffer[17]&0x0f)<<7));
        
       // printf("ROLL : %d\n\r PITCH : %d\n\rYAW : %d\n\rTROTTLE : %d\n\r\n\r", sbus_ch_data[CH4],sbus_ch_data[CH2],sbus_ch_data[CH1],sbus_ch_data[CH3]);
}
