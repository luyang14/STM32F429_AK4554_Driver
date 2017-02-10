#ifndef STM32F4_AK4554_H
#define STM32F4_AK4554_H

#include "stdio.h"	
#include "stm32f4xx.h"

#define ma_AudioFreq  I2S_AudioFreq_16k

void AK4554_init(void);
uint32_t EVAL_AUDIO_Init(uint32_t AudioFreq);

extern u8 *i2sbuf[]; 					//��Ƶ����֡,ռ���ڴ���=AUDIO_BUF_NUM*AUDIO_OUT_PACKET �ֽ�
extern u8 *i2sRbuf[]; 					//��Ƶ����֡,ռ���ڴ���=AUDIO_BUF_NUM*AUDIO_OUT_PACKET �ֽ�
#endif
