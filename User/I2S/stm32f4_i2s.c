#include "stm32f4_i2s.h"  
#include "stm32f4_usart.h"


/*
*********************************************************************************************************
*	函 数 名: I2S_NVIC_Config
*	功能说明: 配置I2S NVIC通道(中断模式)。中断服务函数void SPI3_IRQHandler(void) 在stm32f10x_it.c
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void I2S_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* SPI3 IRQ 通道配置 */
	NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/********************************************************************************************************
 * 函数名：I2S3初始化
 * 描述  ：
 * 输入  : I2S_Standard: @SPI_I2S_Standard  I2S标准,
 *                            I2S_Standard_Phillips,飞利浦标准;
 *                            I2S_Standard_MSB,MSB对齐标准(右对齐);
 *                            I2S_Standard_LSB,LSB对齐标准(左对齐);
 *                            I2S_Standard_PCMShort,
 *														 I2S_Standard_PCMLong:PCM标准
 *         I2S_Mode: @SPI_I2S_Mode  I2S_Mode_SlaveTx:从机发送;
 *                                  I2S_Mode_SlaveRx:从机接收;
 *																	 I2S_Mode_MasterTx:主机发送;
 *																	 I2S_Mode_MasterRx:主机接收;
 *         I2S_Clock_Polarity  &SPI_I2S_Clock_Polarity:  I2S_CPOL_Low,时钟低电平有效;
 *                                                       I2S_CPOL_High,时钟高电平有效
 *         I2S_DataFormat：@SPI_I2S_Data_Format :数据长度,I2S_DataFormat_16b,16位标准;
 *                                                        I2S_DataFormat_16bextended,16位扩展(frame=32bit);
 *																												I2S_DataFormat_24b,24位;
 *																												I2S_DataFormat_32b,32位.
 * 输出 ：无
 *********************************************************************************************************/
void I2S3_Init(u16 I2S_Standard,u16 I2S_Mode,u16 I2S_Clock_Polarity,u16 I2S_DataFormat,u32 _AudioFreq)
{ 
  I2S_InitTypeDef I2S_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI3时钟

  I2S_InitStructure.I2S_Mode=I2S_Mode;                   //IIS模式
  I2S_InitStructure.I2S_Standard=I2S_Standard;           //IIS标准
  I2S_InitStructure.I2S_DataFormat=I2S_DataFormat;       //IIS数据长度
  I2S_InitStructure.I2S_MCLKOutput=I2S_MCLKOutput_Enable;//主时钟输出禁止I2S_MCLKOutput_Disable
  I2S_InitStructure.I2S_AudioFreq=_AudioFreq;            //IIS频率设置
  I2S_InitStructure.I2S_CPOL=I2S_Clock_Polarity;         //空闲状态时钟电平
  I2S_Init(SPI3,&I2S_InitStructure);                     //初始化IIS
  
	/* Configure the I2Sx_ext (the second instance) in Slave Receiver Mode */
	I2S_FullDuplexConfig(I2S3ext, &I2S_InitStructure);
	
	
//	SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Rx,ENABLE);         //SPI3 TX DMA请求使能.
//  SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Tx,ENABLE);         //SPI3 TX DMA请求使能.
	
	/* 禁止I2S3 TXE中断(发送缓冲区空)，需要时再打开 */
	SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_TXE, ENABLE);

	/* 禁止I2S3 RXNE中断(接收不空)，需要时再打开 */
	SPI_I2S_ITConfig(I2S3ext, SPI_I2S_IT_RXNE, ENABLE);
	
  I2S_Cmd(SPI3,ENABLE);                                  //SPI3 I2S EN使能.	
  /* Enable the I2Sx_ext peripheral for Full Duplex mode */
	I2S_Cmd(I2S3ext, ENABLE);
	
	I2S_NVIC_Config();
} 
//采样率计算公式:Fs=I2SxCLK/[256*(2*I2SDIV+ODD)]
//I2SxCLK=(HSE/pllm)*PLLI2SN/PLLI2SR
//一般HSE=8Mhz 
//pllm:在Sys_Clock_Set设置的时候确定，一般是8
//PLLI2SN:一般是192~432 
//PLLI2SR:2~7
//I2SDIV:2~255
//ODD:0/1
//I2S分频系数表@pllm=8,HSE=8Mhz,即vco输入频率为1Mhz
//表格式:采样率/10,PLLI2SN,PLLI2SR,I2SDIV,ODD
const u16 I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz采样率
	{1102,429,4,19,0},		//11.025Khz采样率 
	{1600,213,2,13,0},		//16Khz采样率
	{2205,429,4, 9,1},		//22.05Khz采样率
	{3200,213,2, 6,1},		//32Khz采样率
	{4410,271,2, 6,0},		//44.1Khz采样率
	{4800,258,3, 3,1},		//48Khz采样率
	{8820,316,2, 3,1},		//88.2Khz采样率
	{9600,344,2, 3,1},  	//96Khz采样率
	{17640,361,2,2,0},  	//176.4Khz采样率 
	{19200,393,2,2,0},  	//192Khz采样率
}; 
/**********************************
 * 函数名 : I2S2_SampleRate_Set
 * 功能   ：设置IIS的采样率(@MCKEN)
 * 输入   ：samplerate:采样率,单位:Hz
 * 输出   : 0,设置成功;1,无法设置.
 **********************************/
u8 I2S3_SampleRate_Set(u32 samplerate)
{ 
	u8 i=0; 
	u32 tempreg=0;
	
	samplerate/=10;//缩小10倍   
	
	for(i=0;i<(sizeof(I2S_PSC_TBL)/10);i++)//看看改采样率是否可以支持
	{
		if(samplerate==I2S_PSC_TBL[i][0])break;
	}
 
	RCC_PLLI2SCmd(DISABLE);//先关闭PLLI2S
	if(i==(sizeof(I2S_PSC_TBL)/10))return 1;//搜遍了也找不到
	RCC_PLLI2SConfig((u32)I2S_PSC_TBL[i][1],(u32)I2S_PSC_TBL[i][2]);//设置I2SxCLK的频率(x=2)  设置PLLI2SN PLLI2SR
	RCC->CR|=1<<26;					      //开启I2S时钟
	while((RCC->CR&1<<27)==0);		//等待I2S时钟开启成功. 
	tempreg=I2S_PSC_TBL[i][3]<<0;	//设置I2SDIV
	tempreg|=I2S_PSC_TBL[i][3]<<8;//设置ODD位
	tempreg|=1<<9;					      //使能MCKOE位,输出MCK
	SPI2->I2SPR=tempreg;			    //设置I2SPR寄存器 
	return 0;
} 
/*********************************************************************
 * 函数名 : I2S3_RX_DMA_Init
 * 功能   ：I2S3 RX DMA配置   设置为双缓冲模式,并开启DMA传输完成中断
 * 输入   ：buf0:M0AR地址. buf1:M1AR地址. num:每次传输数据量
 * 输出   ：无
 *********************************************************************/
void I2S3_RX_DMA_Init(u8* buf0,u8 *buf1,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	DMA_DeInit(DMA1_Stream2);
	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}//等待DMA1_Stream1可配置 
		
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道0 SPI3_RX通道 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;//外设地址为:(u32)&SPI3->DR
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)buf0;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = num;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度：16位 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //不使用FIFO模式        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//外设突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//存储器突发单次传输
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);//初始化DMA Stream
	
	DMA_DoubleBufferModeConfig(DMA1_Stream2,(u32)buf1,DMA_Memory_0);//双缓冲模式配置
 
  DMA_DoubleBufferModeCmd(DMA1_Stream2,ENABLE);//双缓冲模式开启
 
  DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);//开启传输完成中断
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置 
}  
/*********************************************************************
 * 函数名 : I2S2_TX_DMA_Init
 * 功能   ：I2S2 TX DMA配置   设置为双缓冲模式,并开启DMA传输完成中断
 * 输入   ：buf0:M0AR地址. buf1:M1AR地址. num:每次传输数据量
 * 输出   ：无
 *********************************************************************/
void I2S3_TX_DMA_Init(u8* buf0,u8 *buf1,u16 num)
{  
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	DMA_DeInit(DMA1_Stream7);
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}//等待DMA1_Stream1可配置 
		
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道0 SPI2_TX通道 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;//外设地址为:(u32)&SPI3->DR
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)buf0;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = num;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度：16位 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //不使用FIFO模式        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//外设突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//存储器突发单次传输
  DMA_Init(DMA1_Stream7, &DMA_InitStructure);//初始化DMA Stream
	
	DMA_DoubleBufferModeConfig(DMA1_Stream7,(u32)buf1,DMA_Memory_0);//双缓冲模式配置
 
  DMA_DoubleBufferModeCmd(DMA1_Stream7,ENABLE);//双缓冲模式开启
 
  DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);//开启传输完成中断
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
  
}  

/**
  * @brief  This function handles SPI3 Handler.
  * @param  None
  * @retval None
  */
u16 sample_mic = 0;
u16 sample_audio = 0;
void SPI3_IRQHandler(void)
{	
	if (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == SET)
		{
			SPI_I2S_SendData(SPI3, sample_mic);
		}

		if (SPI_I2S_GetFlagStatus(I2S3ext, SPI_I2S_FLAG_RXNE) == SET)
		{
			sample_mic = SPI_I2S_ReceiveData(I2S3ext);
//			printf("%02X ", sample_mic);
		}
	
}


//I2S DMA回调函数指针
void (*i2s_tx_callback)(void);	//TX回调函数 
//DMA1_Stream4中断服务函数
void DMA1_Stream7_IRQHandler(void)
{      
	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF4)==SET)////DMA1_Stream4,传输完成标志
	{ 
		DMA_ClearITPendingBit(DMA1_Stream7,DMA_IT_TCIF7);
    i2s_tx_callback();	//执行回调函数,读取数据等操作在这里面处理  
	}   											 
}  
//I2S开始播放
void I2S_Play_Start(void)
{   	  
	DMA_Cmd(DMA1_Stream7,ENABLE);//开启DMA TX传输,开始播放 		
}
//关闭I2S播放
void I2S_Play_Stop(void)
{   
		DMA_Cmd(DMA1_Stream7,DISABLE);//关闭DMA,结束播放	 
} 

