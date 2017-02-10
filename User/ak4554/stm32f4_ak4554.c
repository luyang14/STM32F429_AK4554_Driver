#include "stm32f4_ak4554.h"	
#include "stm32f4_i2s.h" 

void AK4554_init(void)
	{
		//GPIO�˿�����
		GPIO_InitTypeDef GPIO_InitStructure;
			
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
	
		//�˿�����MCLK:PC7 SCLK:PC10 SDTO:PC11 SDTI:PC12
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC7,PC10,PC12,PC11
		//�˿�����LRCK:PA15	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //����
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
		GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA15
		
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_SPI3);  //PA15,AF6  I2S_LRCK
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3);	//PC10,AF6  I2S_SCLK 
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);	//PC12,AF6  I2S_DACDATA 
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_SPI3);	  //PC7 ,AF6  I2S_MCK
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);	//PC11,AF6  I2S_ADCDATA 
	}
//��Ƶ����I2S DMA����ص�����
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
//			//DMA1_Stream4->M0AR=(u32)i2sbuf[i2splaybuf];//ָ����һ��buf 
//		}
//		else 
//		{   
//			DMA_MemoryTargetConfig(DMA1_Stream4,(u32)i2sbuf[i2splaybuf],DMA_Memory_1);
//			//DMA1_Stream4->M1AR=(u32)i2sbuf[i2splaybuf];//ָ����һ��buf 
//		} 
//	}
//}  
//������Ƶ�ӿ�
//OutputDevice:����豸ѡ��,δ�õ�.
//Volume:������С,0~100
//AudioFreq:��Ƶ������
#define AUDIO_BUF_NUM		100				//���ڲ��õ���USB�첽ʱ�Ӳ���
											//STM32 IIS���ٶȺ�USB���͹������ݵ��ٶȴ��ڲ���,������48Khz��,ʵ
											//��IIS�ǵ���48Khz(47.991Khz)��,���Ե����͹�����������,���STM32����
											//�ٶȿ�,������дλ��׷�ϲ���λ��(i2ssavebuf==i2splaybuf)ʱ,�ͻ����
											//���.���þ������AUDIO_BUF_NUMֵ,���Ծ������ٻ������. 
								
u8 *i2sbuf[AUDIO_BUF_NUM]; 					//��Ƶ����֡,ռ���ڴ���=AUDIO_BUF_NUM*AUDIO_OUT_PACKET �ֽ�
u8 *i2sRbuf[AUDIO_BUF_NUM]; 			    //��Ƶ����֡,ռ���ڴ���=AUDIO_BUF_NUM*AUDIO_OUT_PACKET �ֽ�
uint32_t EVAL_AUDIO_Init(uint32_t AudioFreq)
{   
	I2S3_Init(I2S_Standard_Phillips,I2S_Mode_MasterTx,I2S_CPOL_Low,I2S_DataFormat_16b,ma_AudioFreq);//�����ֱ�׼,��������,ʱ�ӵ͵�ƽ��Ч,16λ��չ֡����
	I2S3_SampleRate_Set(AudioFreq);		//���ò�����
	//if(i==1)printf("\r\nƵ������ʧ��");
	//if(i==0)printf("\r\nƵ���������");
//	I2S3_TX_DMA_Init(i2sbuf[0],i2sbuf[1],(uint32_t)(((8000 * 2 * 2) /1000))/2); 
//	I2S3_RX_DMA_Init(i2sRbuf[0],i2sRbuf[1],(uint32_t)(((8000 * 2 * 2) /1000))/2);
// 	i2s_tx_callback=audio_i2s_dma_callback;		//�ص�����ָwav_i2s_dma_callback
//	I2S_Play_Start();							//����DMA  
	//printf("\r\nEVAL_AUDIO_Init:%d\r\n",AudioFreq);
	return 0; 
}
