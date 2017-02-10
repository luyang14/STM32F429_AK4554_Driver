#ifndef __I2S_H
#define __I2S_H
   									
#include "stm32f4xx.h"

extern void (*i2s_tx_callback)(void);		//IIS TX回调函数指针  
extern u16 sample_mic;
extern u16 sample_audio;
void I2S3_Init(u16 I2S_Standard,u16 I2S_Mode,u16 I2S_Clock_Polarity,u16 I2S_DataFormat,u32 _AudioFreq); 
void I2S3_RX_DMA_Init(u8* buf0,u8 *buf1,u16 num); 
void I2S3_TX_DMA_Init(u8* buf0,u8 *buf1,u16 num);
u8 I2S3_SampleRate_Set(u32 samplerate);

void I2S_Play_Start(void); 
void I2S_Play_Stop(void);  
#endif





















