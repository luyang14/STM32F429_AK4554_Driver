#ifndef STM32F4_AK4554_H
#define STM32F4_AK4554_H

#include "stdio.h"	
#include "stm32f4xx.h"

#define ma_AudioFreq  I2S_AudioFreq_16k

void AK4554_init(void);
uint32_t EVAL_AUDIO_Init(uint32_t AudioFreq);

extern u8 *i2sbuf[]; 					//音频缓冲帧,占用内存数=AUDIO_BUF_NUM*AUDIO_OUT_PACKET 字节
extern u8 *i2sRbuf[]; 					//音频缓冲帧,占用内存数=AUDIO_BUF_NUM*AUDIO_OUT_PACKET 字节
#endif
