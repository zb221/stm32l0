
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
//#include "stm32l072xx.h"

/* USER CODE BEGIN Includes */

#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330))

//#define EEPROM_ADDR  0x08006000 //EEPROM 地址
/* USER CODE END Includes */


//要写入到STM32 FLASH的字符串数组
//const uint8_t TEXT_Buffer[]={"STM32F103 FLASH TEST"};
const uint16_t  f[12]={0,1,2,3,4,5,6,7,8,9,10,11};
//#define SIZE sizeof(b)		//数组长度
//#define FLASH_SAVE_ADDR  0X08070000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR1  0X08010000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)


//用户根据自己的需要设置
#define STM32_FLASH_SIZE 	128 	 		//所选STM32的FLASH容量大小(单位为K)STM32F103ZE 512K  STM32L072KBUx为128K
#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 		//STM32 FLASH的起始地址
//FLASH解锁键值
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB
#define FLASH_KEY1               0XFBEAD9C8
#define FLASH_KEY2               0X24252627

#define EEPROM_BASE_ADDR	0x08080000	
#define EEPROM_BYTE_SIZE	0x01FF

 uint8_t TEXT_Buffer[]={"STM32F103 FLASH"};
#define SIZE sizeof(TEXT_Buffer)		//数组长度


//#define FLASH_OPTKEY1              ((uint32_t)0xFBEAD9C8U) /*!< Flash option key1 */
//#define FLASH_OPTKEY2              ((uint32_t)0x24252627U) /*!< Flash option key2: used with FLASH_OPTKEY1 to


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;   

UART_HandleTypeDef huart1;

//UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int16_t TempTab[16][3] = {  //温度补偿数组
	
	//                温度  1450  1660
//										{125,-80,  400},   //125----保护
//										{94, -80,  400},	  //85
//										{89, -97,  431},	  //80
//										{85, -81,  444},    //75
//										{80, -65,  458},    //70
//										{75, -44, 471},     //65
//										{70, -24, 484},			//60								  
//										{65, 8, 495},     //55
//										{60, 40,  505},			//50								  
//										{54, 58,  517},     //45
//										
//										{48, 76,  531},     //40
//										{45, 95,  530},     //35
//										{39, 115, 530},			//30							  
//										{37, 150, 540},      //25
//										{33, 170, 545},			//20							  
//										{24, 210, 550},		  //15
//										
//										{22, 220, 555},     //10
//										{19, 230, 555},      //5
//										{14, 255, 555},			//0								  
//										{9,  275, 555},      //-5
//										{5,  295, 545},		//-10									  
//										{0,  315, 530},	  //-15
//										
//										{-5, 315, 505},    //-20
//										{-9, 330, 490},    //-25
//										{-14,360, 470},		//-30									  
//										{-18,365, 450},     //-35
//										{-23,375, 430},		//-40									  
//										{-28,385, 410},	    //-45
//										{-40,385, 410}	    //-50---保护

										{125,0,0},   //125----保护
										{86,0,0},	  //
										{75,0,0},	  //
										{66,0,0},    //
										{58,0,0},    //
										{49,0,0},     //
										{42,0,0},			//								  
										{33,0,0},     //
										{22,0,0},			//								  
										{15,0,0},     //
										
										{7,0,0},     //
										{-2,0,0},     //
										{-11,0,0},			//							  
										{-20,0,0},      //
										{-28,0,0},			//						  										
										{-50,0,0}	    //-50---保护
								                        };


/* -----------------------------temperature参数----------------------------*/
int32_t ad_temp_avge=0;
int32_t temperature=0;
uint32_t temp_cnt=0;  

int16_t temp_value,adj_temp_1450,adj_temp_1660,j;
/* -----------------------------电压参数----------------------------*/
uint32_t Vbias1450,Vbias1660;

/* -----------------------------dac参数-----------------------------*/
uint32_t  adc_val_pos=0; 
int16_t  dac_val_adj=0,dac_val_adj_1=0;   
uint8_t  dac_val_sign=0;   
uint32_t dac_val_base= 4095;  
uint32_t dac_val_set,dac1_1450_val_set,dac2_1660_val_set;  


uint8_t  dac_uart_channel;
uint8_t  dac_uart_sign=0;   
uint16_t  dac_uart_adj=0; 
uint16_t  dac_uart_adj_H=0; 
uint16_t  dac_uart_adj_L=0; 

uint8_t  dac_uart_sign2=0;   
uint16_t  dac_uart_adj2=0; 
uint16_t  dac_uart_adj_H2=0; 
uint16_t  dac_uart_adj_L2=0; 
/* -----------------------------adc参数----------------------------*/
uint32_t adc_val=0;
uint8_t  adc_flag=0;

uint8_t ping_flag=0;

/* -----------------------------timer参数----------------------------*/
uint16_t systick_1scnt=0;
uint8_t systick_1sflag=0;
uint16_t systick_100mscnt=0;
uint8_t systick_100msflag=0;

/* -----------------------------uart参数----------------------------*/

//uint16_t a[12]={0,1,2,3,4,5,6,7,8,9,10,11};

#define RXBUF_NUM 1
uint8_t uart1_process_txbuf[102] = {0};
uint8_t uart1_process_rxbuf[102] = {0};
uint8_t rx_array_102=0;
uint8_t rxbuf[RXBUF_NUM] = {0};                //串口接收
uint8_t rxflag[10] = {0};
uint8_t arx_data=0;
uint8_t TX_ON,RX_ERR_FLAG;
uint8_t RX_FLAG = 1;
uint8_t RX_FLAG_PRO = 0;
int rx_index = 2;
uint8_t header_flag = 1;
uint8_t adc_or_rx = 1;

uint16_t  a[48]={0,1,2,3,4,5,6,7,8,9,10,11,11,10,9,8,7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7,8,9,10,11,11,10,9,8,7,6,5,4,3,2,1,0};
int16_t b[16]={0};
int16_t c[16]={0};
int16_t d[16]={0};
uint8_t e[96]={0};
uint8_t arr[2];



uint8_t datatemp[96];	
/* -----------------------------eeprom参数----------------------------*/
uint32_t rdeeprom1=0;
uint32_t wreeprom1=0;
uint32_t rdeeprom2=0;
uint32_t wreeprom2=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
void Uart1_process(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void variables_Init(void);
int16_t FindBySeq(int16_t temp_value);
void adc_pro(void);

/* USER CODE END PFP */

#define EN_INT      	__enable_irq();		//???????
#define DIS_INT     	__disable_irq();	//???????
#define PEKEY1	0x89ABCDEF		//FLASH_PEKEYR
#define PEKEY2	0x02030405		//FLASH_PEKEYR

void EEPROM_Write_Bytes(uint32_t Addr,uint8_t *Buffer,uint16_t Length)
{
	uint32_t wAddr = DATA_EEPROM_BASE + Addr;

	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();
	HAL_FLASHEx_DATAEEPROM_Unlock();
	
	while(Length--)
		{
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, wAddr, (uint32_t)(*Buffer));
		wAddr++;
		Buffer++;
	}
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();
}

void	EEPROM_read_Bytes(uint32_t	addr, uint8_t *data, uint16_t len)
{
	int8_t*	wAddr;
	wAddr = (int8_t*)(addr);
	while(len--)
	{
		*data++ = *wAddr++;
	}
}

/* USER CODE BEGIN 0 */

/*******************************************************************************
* : delay_us

*******************************************************************************/
void delay_us(uint32_t i)
{
    uint32_t temp;
    SysTick->LOAD=9*i;         //??????, 72MHZ?
    SysTick->CTRL=0X01;         //??,???????,???????
    SysTick->VAL=0;                //?????
    do
    {
        temp=SysTick->CTRL;           //????????
    }
    while((temp&0x01)&&(!(temp&(1<<16))));     //??????
    SysTick->CTRL=0;    //?????
    SysTick->VAL=0;        //?????
}


void Delay(__IO uint32_t nCount)  //软件延时函数
{
  for(; nCount != 0; nCount--);
} 

void HAL_SYSTICK_Callback()   //滴答定时器回调函数
{
	 if(systick_1scnt<1000) systick_1scnt++;
	 else
	 {
      systick_1scnt=0;
      systick_1sflag=1;	

			HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM);
			RX_FLAG = 1;
			rx_index = 2;
			header_flag = 1;
			for(uint8_t index1=0;index1<10;index1++)
			rxflag[index1] = 0;
	 }
	 
	 if(systick_100mscnt<100) 
	 {
		  systick_100mscnt++;	  
	 }		  
	 else
	 {
      systick_100mscnt=0;
      systick_100msflag=1;		 
	 }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)  //接收中断回调函数
{
	adc_or_rx = 0;

	if (RX_FLAG == 1)
	{
		if (header_flag){
			if (rxbuf[0] == 0xaa)
				rxflag[0] = rxbuf[0];
			if (rxbuf[0] == 0x55){
				rxflag[1] = rxbuf[0];
				header_flag = 0;
				rx_index = 2;
			}
	  }
		if ((rxflag[0]==0xaa)&&(rxflag[1]==0x55)){
			uart1_process_rxbuf[rx_index++-1] = rxbuf[0];
		}
		if (uart1_process_rxbuf[3]+6 == rx_index){
			uart1_process_rxbuf[0] = rxflag[0];
			uart1_process_rxbuf[1] = rxflag[1];
			rx_index = 2;
			RX_FLAG=0;
			RX_FLAG_PRO = 1;
			header_flag = 1;
		}
//		if (RX_FLAG == 1)
			HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //adc回调函数
{
	 adc_flag=1;	 	
}

void variables_Init() //变量初始化
{
	
	 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
	 HAL_DAC_Start(&hdac, DAC_CHANNEL_1);	
	 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	 HAL_DAC_Start(&hdac, DAC_CHANNEL_2);	 
	 HAL_ADCEx_Calibration_Start(&hadc,ADC_SINGLE_ENDED); 
	
	 HAL_Delay(1000);	
	
   HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM); //开接收中断，接收到1个数据后进入回调函数
	 
}



//解锁STM32的FLASH
void STMFLASH_Unlock(void)
{
	FLASH->KEYR=FLASH_KEY1;//写入解锁序列.
	FLASH->KEYR=FLASH_KEY2;
}

//flash上锁
void STMFLASH_Lock(void)
{
	FLASH->CR|=1<<7;//上锁
}
//得到FLASH状态
uint8_t STMFLASH_GetStatus(void)
{	
	uint32_t res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //忙
	else if(res&(1<<2))return 2;	//编程错误
	else if(res&(1<<4))return 3;	//写保护错误
	return 0;						//操作完成
}

//等待操作完成
//time:要延时的长短
//返回值:状态.
uint8_t STMFLASH_WaitDone(uint16_t time)
{
	uint8_t res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//非忙,无需等待了,直接退出.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//擦除页
//paddr:页地址
//返回值:执行情况
uint8_t STMFLASH_ErasePage(uint32_t paddr)
{
	uint8_t res=0;
	res=STMFLASH_WaitDone(0X5FFF);//等待上次操作结束,>20ms    
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//页擦除
		FLASH->AR=paddr;//设置页地址 
		FLASH->CR|=1<<6;//开始擦除		  
		res=STMFLASH_WaitDone(0X5FFF);//等待操作结束,>20ms  
		if(res!=1)//非忙
		{
			FLASH->CR&=~(1<<1);//清除页擦除标志.
		}
	}
	return res;
}

//在FLASH指定地址写入半字
//faddr:指定地址(此地址必须为2的倍数!!)
//dat:要写入的数据
//返回值:写入的情况
uint8_t STMFLASH_WriteHalfWord(uint32_t faddr, uint16_t dat)
{
	uint8_t res;	   	    
	res=STMFLASH_WaitDone(0XFF);	 
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;		//编程使能
		*(vu16*)faddr=dat;		//写入数据
		res=STMFLASH_WaitDone(0XFF);//等待操作完成
		if(res!=1)//操作成功
		{
			FLASH->CR&=~(1<<0);	//清除PG位.
		}
	} 
	return res;
} 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);

	return *(vu16*)faddr;
  	
}
#if STM32_FLASH_WREN	//如果使能了写   
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数 
void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;
	for(i=0;i<NumToWrite;i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
	  WriteAddr+=2;//地址增加2.
	}  
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead)   	
{
	uint16_t i;
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
	for(i=0;i<NumToRead;i++)
	{
	//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
	//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
		ReadAddr+=2;//偏移2个字节.
    		
	}
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
}


//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
uint16_t STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //扇区地址
	uint16_t secoff;	   //扇区内偏移地址(16位字计算)
	uint16_t secremain; //扇区内剩余地址(16位字计算)	   
 	uint16_t i;    
	uint32_t offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);	
	STMFLASH_Unlock();						//解锁
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);	
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小
 // HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);	
	/*
	STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);
	for(i=0;i<NumToWrite;i++)
	{
		 STMFLASH_WriteHalfWord(WriteAddr+i*2,pBuffer[i]);
	}
	*/
	
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
//	HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
	while(1) 
	{	
//		HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
//		HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);	
		for(i=0;i<secremain;i++)	//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)				//需要擦除
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区  
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumToWrite==secremain) break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain*2;	//写地址偏移(16位数据地址,需要*2)	   
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};
	
	STMFLASH_Lock();//上锁
}
#endif


uint16_t hebing(uint16_t res,uint8_t array[2])
{
    res=array[1];//gaowei
    res=(res<<8)|array[0];
  //  Info_UART_TX_PutChar(res);
    return res;
    
}


void adc_pro()  //adc处理函数
{	  
		HAL_UART_AbortReceive_IT(&huart1);

	   uint8_t adc_pro_txbuf[16] = {0};
		 int checksum = 0;

		 if(ping_flag==1)
      {
         for(uint8_t m=0;m<16;m++)
         {
      
          TempTab[m][0]=b[m];
          TempTab[m][1]=c[m];
					TempTab[m][2]=d[m];
          }
       
          ping_flag=0;
       }
	if(adc_flag==1)
		{
			 adc_flag=0; 
			 HAL_ADC_Stop(&hadc);
			 adc_val=HAL_ADC_GetValue(&hadc);
			
			 temperature = ((adc_val * VDD_APPLI / VDD_CALIB)- (int32_t) *TEMP30_CAL_ADDR );
			 temperature = temperature * (int32_t)(130 - 30);
       temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
       temperature = temperature + 30;			      
			 temp_cnt++;		
		}
		
    if(systick_100msflag==1)
		{
			  systick_100msflag=0;
		    HAL_ADC_Start_IT(&hadc);
		}
		
		if(systick_1sflag==1)      //1秒钟进行一次补偿
		{
//			HAL_NVIC_DisableIRQ(USART1_IRQn);
			  systick_1sflag=0;
			  HAL_ADC_Stop(&hadc);
			  
			  if(temp_cnt!=0)
				{
					
					  ad_temp_avge=temperature;
            if(ad_temp_avge>=0)		
						  {
								temp_value=ad_temp_avge;
							}
						else
						  {
							  temp_value=-ad_temp_avge;
						    temp_value=-temp_value;							  						
							}						
							
            dac_val_base=3300;					 					
						dac1_1450_val_set=dac_val_base;
						dac2_1660_val_set=dac_val_base;
							
							
             j=FindBySeq(temp_value);
					
         	adj_temp_1450=TempTab[j][1]+(TempTab[j+1][1]-TempTab[j][1])/(TempTab[j][0]-TempTab[j+1][0])*(TempTab[j][0]-temp_value) ;						
          adj_temp_1660=TempTab[j][2]+(TempTab[j+1][2]-TempTab[j][2])/(TempTab[j][0]-TempTab[j+1][0])*(TempTab[j][0]-temp_value) ;				
							
					dac1_1450_val_set=dac1_1450_val_set+adj_temp_1450;
					dac2_1660_val_set=dac2_1660_val_set+adj_temp_1660;
				

            
							
							
						if(dac_uart_sign)                       //串口dac1调节 1450
						{							
						  dac1_1450_val_set=dac1_1450_val_set-dac_uart_adj;																
						}							
						else
						{
						dac1_1450_val_set=dac1_1450_val_set+dac_uart_adj;														
						}

						if(dac_uart_sign2)                       //串口dac2调节 1660
						{							
						dac2_1660_val_set=dac2_1660_val_set-dac_uart_adj2;																
						}							
						else
						{
						dac2_1660_val_set=dac2_1660_val_set+dac_uart_adj2;														
						}			


						
		       HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac1_1450_val_set);   //设置dac1 1450	
	         HAL_DAC_Start(&hdac, DAC_CHANNEL_1);	
			     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac2_1660_val_set);   //设置dac2 1660
	         HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

						
					adc_pro_txbuf[0]=0x55;
					adc_pro_txbuf[1]=0xAA;
					adc_pro_txbuf[2]=0xD1;
					adc_pro_txbuf[3]=0x0A;

					 if(ad_temp_avge>=0) 
					 {
						  adc_pro_txbuf[4]=0;
						  adc_pro_txbuf[5]=ad_temp_avge;					
					 }
					 else
					 {
						  adc_pro_txbuf[4]=0x01;
						  adc_pro_txbuf[5]=-ad_temp_avge;					 
					 }
					 
					 Vbias1450=(71-dac1_1450_val_set*14.9994/1000)*100;
					 Vbias1660=(71-dac2_1660_val_set*14.9994/1000)*100;
					 					 
					 adc_pro_txbuf[6]=Vbias1450>>8;
					 adc_pro_txbuf[7]=Vbias1450;
					 adc_pro_txbuf[8]=Vbias1660>>8;
					 adc_pro_txbuf[9]=Vbias1660;
					 				 
					 adc_pro_txbuf[10]=dac1_1450_val_set>>8;
					 adc_pro_txbuf[11]=dac1_1450_val_set;
//					 printf("1->1450: %d = ",adc_pro_txbuf[10]);
//					 printf("2->1450: %d = ",adc_pro_txbuf[11]);
					 adc_pro_txbuf[12]=dac2_1660_val_set>>8;
					 adc_pro_txbuf[13]=dac2_1660_val_set;
					 
					 for (int i = 1; i <= adc_pro_txbuf[3];i++)
					 {
							checksum += adc_pro_txbuf[3+i];
					 }

					 adc_pro_txbuf[14] = checksum & 0xFF;
					 adc_pro_txbuf[15] = (checksum>>8) & 0xFF;
					 
	         HAL_UART_Transmit(&huart1,(uint8_t *)adc_pro_txbuf,16,5000);	

			 //    HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);	
				//	 HAL_UART_Transmit(&huart1,(uint8_t *)TEXT_Buffer,SIZE,5000);	 
				//	 STMFLASH_Write(FLASH_SAVE_ADDR,(uint16_t *)TEXT_Buffer,SIZE);
		     //  HAL_UART_Transmit(&huart1,(uint8_t *)a,24,5000);
	       //  STMFLASH_Read(FLASH_SAVE_ADDR,(uint16_t *)datatemp,SIZE);
		    //   HAL_UART_Transmit(&huart1,(uint8_t *)datatemp,SIZE,5000);
					 
			     temp_cnt=0;
					 
					 
				}
//				HAL_NVIC_EnableIRQ(USART1_IRQn);
		}
		HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM);

}  

int16_t FindBySeq(int16_t temp_value)
{

	  int16_t i;
    for ( i = 0; i < 16; i++)
    {
        if ((TempTab[i][0] >= temp_value) && (TempTab[i+1][0] < temp_value))
        return i;
    }
		return 0;    
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
 // 	uint16_t datatemp[SIZE];	   
	/* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  variables_Init();
  /* USER CODE END 2 */
 	 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	EEPROM_read_Bytes(0x08081000,datatemp,96);
	//   STMFLASH_Read(FLASH_SAVE_ADDR1,(uint16_t *)datatemp,96);
	HAL_UART_Transmit(&huart1,(uint8_t *)datatemp,96,5000);	
	   for(uint8_t i=0;i<16;i++)
     {
          arr[0]=datatemp[i*6+0];
          arr[1]=datatemp[i*6+1];
          b[i]=hebing(b[i],arr);
											
          arr[0]=datatemp[i*6+2];
          arr[1]=datatemp[i*6+3];
          c[i]=hebing(c[i],arr);
											
				  arr[0]=datatemp[i*6+4];
          arr[1]=datatemp[i*6+5];
          d[i]=hebing(d[i],arr);
			 ping_flag=1;
		 }
	
  while (1)
  {
  /* USER CODE BEGIN 3 */
		Uart1_process();
		if (adc_or_rx)
			adc_pro();
	//	delay_us(1000000);
  }
  /* USER CODE END 3 */

}


void Info_DataProcess(uint8_t array[2],uint16_t res)
{
    array[0] = res&0x00FF;
    array[1] = (res&0xFF00) >>8;
}




void shuzuchafen(void)
{
   uint8_t i;
   for(i=0;i<16;i++)
   {
     b[i]=TempTab[i][0];
   
     c[i]=TempTab[i][1];
  
		 d[i]=TempTab[i][2];
		 
		 
	 }
}

int checksum_C3 = 0;
void Uart1_process(void)
{
	int checksum = 0;

	int vaild_data_num = 0;

//  char tmp[102] = {1,2,3,4,5,6,7,8,9,}; //only for test

	if(RX_FLAG_PRO)
	{
		RX_FLAG_PRO=0;		 
		if(uart1_process_rxbuf[0]==0xAA && uart1_process_rxbuf[1]==0x55)
		{
			switch(uart1_process_rxbuf[2])
			{
				case 0xC1:
					vaild_data_num = uart1_process_rxbuf[3];
//					HAL_UART_Transmit(&huart1,(uint8_t *)tmp,2,5000);
					if ((uart1_process_rxbuf[3+vaild_data_num+1]|(uart1_process_rxbuf[3+vaild_data_num+2]<<8)) == (uart1_process_rxbuf[4]+uart1_process_rxbuf[5]+uart1_process_rxbuf[6])){
						dac_uart_sign = uart1_process_rxbuf[4];
						dac_uart_adj_H = uart1_process_rxbuf[5];
						dac_uart_adj_L = uart1_process_rxbuf[6];
						dac_uart_adj = (dac_uart_adj_H<<8)+dac_uart_adj_L;
					}
					break;

				case 0xC2:
					vaild_data_num = uart1_process_rxbuf[3];
//				HAL_UART_Transmit(&huart1,(uint8_t *)tmp+1,2,5000);
					if ((uart1_process_rxbuf[3+vaild_data_num+1]|(uart1_process_rxbuf[3+vaild_data_num+2]<<8)) == (uart1_process_rxbuf[4]+uart1_process_rxbuf[5]+uart1_process_rxbuf[6])){
						dac_uart_sign2=uart1_process_rxbuf[4];
						dac_uart_adj_H2=uart1_process_rxbuf[5];
						dac_uart_adj_L2=uart1_process_rxbuf[6];
						dac_uart_adj2=(dac_uart_adj_H2<<8)+dac_uart_adj_L2;
					}
					break;

				case 0xC3:
					vaild_data_num = uart1_process_rxbuf[3];
//				  HAL_UART_Transmit(&huart1,(uint8_t *)tmp+2,2,5000);
					for (uint16_t index=0;index<vaild_data_num;index++)
							checksum += uart1_process_rxbuf[index+4];

				  checksum_C3 = (uart1_process_rxbuf[3+vaild_data_num+1])|(uart1_process_rxbuf[3+vaild_data_num+2]<<8);
					if ((checksum & 0xFF) == checksum_C3)
					{
						for(uint8_t i=0;i<16;i++)
						{
							arr[0]=uart1_process_rxbuf[i*6+4];
							e[i*6+0]=arr[0];
							arr[1]=uart1_process_rxbuf[i*6+5];
							e[i*6+1]=arr[1];
							b[i]=hebing(b[i],arr);

							arr[0]=uart1_process_rxbuf[i*6+6];
							e[i*6+2]=arr[0];
							arr[1]=uart1_process_rxbuf[i*6+7];
							e[i*6+3]=arr[1];
							c[i]=hebing(c[i],arr);

							arr[0]=uart1_process_rxbuf[i*6+8];
							e[i*6+4]=arr[0];
							arr[1]=uart1_process_rxbuf[i*6+9];
							e[i*6+5]=arr[1];
							d[i]=hebing(d[i],arr);
						}
						ping_flag=1;
					}
					break;
				case 0xC4:
//						HAL_UART_Transmit(&huart1,(uint8_t *)tmp+3,2,5000);
						EEPROM_Write_Bytes(0x1000,(uint8_t *)e,96);
						break;
				case 0xC5:
//					HAL_UART_Transmit(&huart1,(uint8_t *)tmp+4,2,5000);
						uart1_process_txbuf[0]=0x55;
						uart1_process_txbuf[1]=0xAA;
						uart1_process_txbuf[2]=0xD2;
						uart1_process_txbuf[3]=0x60;
						shuzuchafen();
						for(uint8_t j=0;j<16;j++)
						{
							Info_DataProcess(arr,b[j]);
							uart1_process_txbuf[j*6+4]=arr[0];
							uart1_process_txbuf[j*6+5]=arr[1];

							Info_DataProcess(arr,c[j]);
							uart1_process_txbuf[j*6+6]=arr[0];
							uart1_process_txbuf[j*6+7]=arr[1];

							Info_DataProcess(arr,d[j]);
							uart1_process_txbuf[j*6+8]=arr[0];
							uart1_process_txbuf[j*6+9]=arr[1];           
						}
						for (uint8_t num=4;num<=99;num++)
							checksum +=  uart1_process_txbuf[num];

						uart1_process_txbuf[100] = checksum & 0xFF;
						uart1_process_txbuf[101] = (checksum>>8) & 0xFF;
						HAL_UART_Transmit(&huart1,(uint8_t *)uart1_process_txbuf,102,5000);	
						break;
				case 0xC6:
					TX_ON=1;
					break;
				case 0xC7:
					TX_ON=0;
					break;
				default:
					break;
			}
		}
/*
		if(RX_ERR_FLAG)
		{
			RX_ERR_FLAG=0;
			HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM);
		}
*/
		for(uint8_t index1=0;index1<10;index1++)
			rxflag[index1] = 0;
		for(uint8_t index2=0;index2<102;index2++)
			uart1_process_rxbuf[index2] = 0;

//		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//		__HAL_UART_CLEAR_FLAG(&huart1,UART_CLEAR_TCF);
		HAL_UART_Receive_IT(&huart1,(uint8_t *) rxbuf,RXBUF_NUM);
		RX_FLAG = 1;
		adc_or_rx = 1;
	}

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
  {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 2, 2);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
//	HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//	__HAL_UART_CLEAR_FLAG(&huart1,UART_CLEAR_TCF);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
