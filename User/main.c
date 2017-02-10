/**
  ******************************************************************************
  * @file    main.c
  * @author  luyang
  * @version V1.0.0
  * @date    7-2-2017
  * @brief            
  * @verbatim
  * @endverbatim
  ******************************************************************************
  * @attention
	*
	*
  ******************************************************************************  
	*/ 

/* Includes ------------------------------------------------------------------*/

#include "stm32f4_ak4554.h"
#include "stm32f4_usart.h"
#include "stm32f4_led.h"
#include "stm32f4_Tim2.h"
#include "stm32f4_i2s.h"

void Delay(__IO uint32_t nTime)
{ 
  while(--nTime != 0);
}

int main()
{ 
  UB_Led_Init();
	uart_init(115200);
	AK4554_init();
	TIM2_Config();
	EVAL_AUDIO_Init(ma_AudioFreq);

	while(1)
	{
//		printf("\r\n%d",GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13));
//    GPIO_SetBits(GPIOE,GPIO_Pin_15);
//		Delay(0x2FFFFF);
//		GPIO_ResetBits(GPIOE,GPIO_Pin_15);
//		Delay(0x2FFFFF);
	}
}
