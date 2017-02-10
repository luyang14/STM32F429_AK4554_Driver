#include "stm32f4_ak4554.h"	
#include "stm32f4_i2s.h" 

void AK4554_init(void)
	{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
			
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	
		//端口配置MCLK:PC7 SCLK:PC10 SDTO:PC11 SDTI:PC12
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC7,PC10,PC12,PC11
		//端口配置LRCK:PA15	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA15
		
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_SPI3);  //PA15,AF6  I2S_LRCK
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3);	//PC10,AF6  I2S_SCLK 
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);	//PC12,AF6  I2S_DACDATA 
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_SPI3);	  //PC7 ,AF6  I2S_MCK
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);	//PC11,AF6  I2S_ADCDATA 
	}
//音频数据I2S DMA传输回调函数
//void audio_i2s_dma_callback(void) 
//{      
//	if((i2splaybuf==i2ssavebuf)&&audiostatus==0)
//	{ 
//		I2S_Play_Stop();
//	}else
//	{
//		i2splaybuf++;
//		if(i2splaybuf>(AUDIO_BUF_NUM-1))i2splaybuf=0;
//		if(DMA1_Stream4->CR&(1<<19))
//		{	 
//			DMA_MemoryTargetConfig(DMA1_Stream4,(u32)i2sbuf[i2splaybuf],DMA_Memory_0);
//			//DMA1_Stream4->M0AR=(u32)i2sbuf[i2splaybuf];//指向下一个buf 
//		}
//		else 
//		{   
//			DMA_MemoryTargetConfig(DMA1_Stream4,(u32)i2sbuf[i2splaybuf],DMA_Memory_1);
//			//DMA1_Stream4->M1AR=(u32)i2sbuf[i2splaybuf];//指向下一个buf 
//		} 
//	}
//}  
//配置音频接口
//OutputDevice:输出设备选择,未用到.
//Volume:音量大小,0~100
//AudioFreq:音频采样率
#define AUDIO_BUF_NUM		100				//由于采用的是USB异步时钟播放
											//STM32 IIS的速度和USB传送过来数据的速度存在差异,比如在48Khz下,实
											//际IIS是低于48Khz(47.991Khz)的,所以电脑送过来的数据流,会比STM32播放
											//速度快,缓冲区写位置追上播放位置(i2ssavebuf==i2splaybuf)时,就会出现
											//混叠.设置尽量大的AUDIO_BUF_NUM值,可以尽量减少混叠次数. 
								
u8 *i2sbuf[AUDIO_BUF_NUM]; 					//音频缓冲帧,占用内存数=AUDIO_BUF_NUM*AUDIO_OUT_PACKET 字节
u8 *i2sRbuf[AUDIO_BUF_NUM]; 			    //音频缓冲帧,占用内存数=AUDIO_BUF_NUM*AUDIO_OUT_PACKET 字节
uint32_t EVAL_AUDIO_Init(uint32_t AudioFreq)
{   
	I2S3_Init(I2S_Standard_Phillips,I2S_Mode_MasterTx,I2S_CPOL_Low,I2S_DataFormat_16b,ma_AudioFreq);//飞利浦标准,主机发送,时钟低电平有效,16位扩展帧长度
	I2S3_SampleRate_Set(AudioFreq);		//设置采样率
	//if(i==1)printf("\r\n频率设置失败");
	//if(i==0)printf("\r\n频率设置完成");
//	I2S3_TX_DMA_Init(i2sbuf[0],i2sbuf[1],(uint32_t)(((8000 * 2 * 2) /1000))/2); 
//	I2S3_RX_DMA_Init(i2sRbuf[0],i2sRbuf[1],(uint32_t)(((8000 * 2 * 2) /1000))/2);
// 	i2s_tx_callback=audio_i2s_dma_callback;		//回调函数指wav_i2s_dma_callback
//	I2S_Play_Start();							//开启DMA  
	//printf("\r\nEVAL_AUDIO_Init:%d\r\n",AudioFreq);
	return 0; 
}
