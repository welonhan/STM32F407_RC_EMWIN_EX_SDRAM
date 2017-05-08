/**
  ******************************************************************************
  * @file    main.c
  * @author  Hanwl
  * @version V1.0
  * @date    2016-12-19
  * @brief   Main program body
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4_discovery_lcd.h"
#include "stm32f4_discovery_uart.h"
#include "stm32f4_discovery_touch.h"
#include "stm32f4_discovery_sd.h"
#include "drv_RF24l01.h"
#include "drv_adc.h"
#include "stm324xg_eval_sram.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <stdio.h>


/*GUI includes component */
#include "GUI.h"
#include "GUIApp.h"
#include "DIALOG.h"

/* TOUCH ---------------------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
TOUCH_XY_Typedef TOUCH_Dat;

/* UART ----------------------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

/* SD card -------------------------------------------------------------------*/
SD_HandleTypeDef uSdHandle;
SD_CardInfo uSdCardInfo;

/* LCD backlight PWM ---------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef sConfig;

/* ADC -----------------------------------------------------------------------*/
ADC_HandleTypeDef  AdcHandle;
uint32_t ADC_Bat_Num=0, ADC_Tmp_Num=0;
uint8_t ADC_Count=0;

/* SRAM ------------------------------------------------------------*/
#define BUFFER_SIZE         ((uint32_t)0x0100)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)
uint16_t aTxBuffer[BUFFER_SIZE];
uint16_t aRxBuffer[BUFFER_SIZE];

/* FATFS ---------------------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
uint8_t duty=0;

uint8_t GUI_Initialized   = 0;

GUI_PID_STATE Touch_State;

/* NRF24L01 ------------------------------------------------------------------*/
#ifdef DEBUG_MODE

char		NRF24L01_TX_DATA[32]="aaaaaaaaaaaaaaaaaaaazzzzzzzzzzb";
uint8_t NRF24L01_ACK_DATA[32]={0};
uint8_t NRF24L01_ACK[8]={88,00,99,99,00,00,66,55};
uint8_t NRF24L01_TX_NUM=sizeof(NRF24L01_TX_DATA);
uint8_t NRF24L01_ACK_NUM=sizeof(NRF24L01_ACK_DATA);
uint16_t ADC_DATA_Debug[4]={0,0,0,0};
uint32_t *DMA_ADC_MEM_ADDR =(uint32_t *)&ADC_DATA_Debug[0];
RC_DATA_TypeDef RC_DATA;

#else
RC_DATA_TypeDef RC_DATA;
uint8_t 	NRF24L01_TX_NUM=sizeof(RC_DATA.RC_TX_Data);
uint8_t 	NRF24L01_ACK_NUM=sizeof(RC_DATA.RC_ACK_Data);
uint32_t 	*DMA_ADC_MEM_ADDR=(uint32_t *)&RC_DATA.ADC_DATA;

#endif

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void 				FATFS_Test(void);
static void Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uwOffset);
uint8_t 		Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);	
void 				SRAM_Test(void);
void 				TXT_2_NUM(uint8_t* txt);
uint32_t 		NUM_2_TXT(uint8_t* txt);
void 				READ_CONFIG_File(void);
void 				WRITE_CONFIG_File(void);

Typdef_ModelData _ModelData[4][4]=
{
	//Model1
	{
		{11,	0, 	100, 100,0},
		{12,	0, 	100, 100,0},
		{13,	0, 	100, 100,0},
		{14,	0, 	100, 100,0},
	},

	//Model2
	{
		{21,	0, 	100, 100,0},
		{22,	0, 	100, 100,0},
		{23,	0, 	100, 100,0},
		{24,	0, 	100, 100,0},
	},

	//Model3
	{
		{31,	0, 	100, 100,0},
		{32,	0, 	100, 100,0},
		{33,	0, 	100, 100,0},
		{34,	0, 	100, 100,0},
	},

	//Model4
	{
		{41,	0, 	100, 100,0},
		{42,	0, 	100, 100,0},
		{43,	0, 	100, 100,0},
		{44,	0, 	100, 100,0},
	},
};
uint8_t ADC_REF=50;
uint8_t BACKLIGHT=100;
uint8_t tmp_backlight, tmp_adc_ref;
Typdef_ModelData tmp_modeldata[4];

uint8_t CH3_Switch=0;
uint8_t CH4_Switch=0;
uint8_t MODEL=0;
uint8_t BACKLIGHT_ON=1;
uint32_t DASH_BOARD[4]=
{
	180,180,180,180
};

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
	uint32_t i,temp1=0,temp2=0;
	//uint32_t x,y,pixel,pixel1;
	RC_DATA.TX_CHANL=60;
	
  HAL_Init();
	
	//0 bits for preemption priority
  //4 bits for subpriority
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  
  /* Configure LED1 and LED3 */
  BSP_LED_Init(LEDR);
  BSP_LED_Init(LEDG);
	
	BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_EXTI);
	
	BSP_UART2_Init();	
	printf("UART2 init ok!\n\r");
	
	BSP_LCD_BACKLIGHT_PWM_Init();
	
	BSP_LCD_Init();
	printf("LCD init ok!\n\r");
	
	BSP_SRAM_Init();
	
	BSP_TOUCH_Init();
	
	BSP_ADC_Init();
	
	for(i=0;i<10;i++)
	{
		temp1+=RC_DATA.ADC_DATA.ADC_STR;
		temp2+=RC_DATA.ADC_DATA.ADC_THRT;
		HAL_Delay(20);
	}
	RC_DATA.ADC_StrMid=(uint16_t)(temp1/10);
	RC_DATA.ADC_ThrtMid=(uint16_t)(temp2/10);

	//NRF24L01_check();
	
	NRF24L01_Init();	
/*	
	pixel=1;
	while(1)
	{
		BSP_LCD_FillRec(0,0,800-1,480-1,pixel);
		HAL_Delay(1);
		pixel++;
		if(pixel>0xffff)
			pixel=0;
	}

	BSP_LCD_FillRec(0,0,5,5,0x1f);
	
	while(1){
	for(pixel=0;pixel<65536;pixel+=200)
	{
		for(y=0;y<480;y+=240)
		{
			pixel1=pixel;
			for(x=0; x<=800;x+=50)
			{
				BSP_LCD_FillRec(x,y,x+50,y+240,pixel1);
				pixel1+=50;
			}
		}
		HAL_Delay(10000);
	}
	}
*/
	
	//FATFS_Test();
	//SRAM_Test();
	READ_CONFIG_File();
	BSP_LCD_BACKLIGHT_PWM_Set(BACKLIGHT);
	//STemWin need enable CRC
	__HAL_RCC_CRC_CLK_ENABLE();
	
	/* Activate the use of memory device feature */
  //WM_SetCreateFlags(WM_CF_MEMDEV);
  
  /* Init the STemWin GUI Library */
  GUI_Init();
  
  GUI_Initialized = 1;

  /* Start Demo */
  MainTask();
	
/*	
	while(1)
	{
		//��ʱ����ң�����ݣ���ʱ�ɸ�����Ҫ�޸�
		HAL_Delay(20);
		RC_TX_DataHandle(&RC_DATA);
		NRF24L01_TxPacket( (uint8_t *)&RC_DATA.RC_TX_Data	, NRF24L01_TX_NUM );
		
	}
*/	
}

/**
  * @brief  FATFS SD card test
  * @param  None
  * @retval None
  */
void FATFS_Test(void)
{
	FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  //uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t wtext[] = "1,0,100,100,0;2,0,100,100,0;3,0,100,100,0;4,0,100,100,0;5,100;";
	uint8_t rtext[100];                                   /* File read buffer */
  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      /*##-3- Create a FAT file system (format) on the logical drive #########*/
      /* WARNING: Formatting the uSD card will delete all content on the device */
      if(f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK)
      {
        /* FatFs Format Error */
        Error_Handler();
      }
      else
      {
        /*##-4- Create and Open a new text file object with write access #####*/
        if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          /* 'STM32.TXT' file Open for write Error */
          Error_Handler();
        }
        else
        {
          /*##-5- Write data to the text file ################################*/
          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
          
          if((byteswritten == 0) || (res != FR_OK))
          {
            /* 'STM32.TXT' file Write or EOF Error */
            Error_Handler();
          }
          else
          {
            /*##-6- Close the open text file #################################*/
            f_close(&MyFile);
            
            /*##-7- Open the text file object with read access ###############*/
            if(f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK)
            {
              /* 'STM32.TXT' file Open for read Error */
              Error_Handler();
            }
            else
            {
              /*##-8- Read data from the text file ###########################*/
              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
              
              if((bytesread == 0) || (res != FR_OK))
              {
                /* 'STM32.TXT' file Read or EOF Error */
                Error_Handler();
              }
              else
              {
                /*##-9- Close the open text file #############################*/
                f_close(&MyFile);
                
                /*##-10- Compare read data with the expected data ############*/
                if((bytesread != byteswritten))
                {                
                  /* Read data is different from the expected data */
                  Error_Handler();
                }
                else
                {
                  /* Success of the demo: no error occurrence */
                  BSP_LED_On(LEDG);
                }
              }
            }
          }
        }
      }
    }
  }
  
  /*##-11- Unlink the micro SD disk I/O driver ###############################*/
  FATFS_UnLinkDriver(SDPath);	
	
}

/**
  * @brief  Read config data file from SD Card
  * @param  None
  * @retval None
  */
void READ_CONFIG_File(void)
{
	FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "11,50,60,60,0;12,60,70,70,1;13,70,80,80,0;14,80,90,100,0;\
											21,50,60,60,0;22,60,70,70,1;23,70,80,80,0;24,80,90,100,0;\
												31,50,60,60,0;32,60,70,70,1;33,70,80,80,0;34,80,90,100,0;\
													41,50,60,60,0;42,60,70,70,1;43,70,80,80,0;44,80,90,100,0;\
														51,70;\
															61,2;\
																71,50;"; /* File write buffer */
  uint8_t rtext[500];                                   /* File read buffer */
	//uint8_t text[500];
  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      if(f_open(&MyFile, "RC.TXT", FA_OPEN_ALWAYS | FA_WRITE|FA_READ) != FR_OK)
      {
          /* 'STM32.TXT' file Open for write Error */
          Error_Handler();
      }
      else
      {
          /*##-5- Write data to the text file ################################*/
          res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
          if(res != FR_OK)
          {
            /* 'STM32.TXT' file read or EOF Error */
            Error_Handler();
          }
					else if(bytesread == 0)
          {
            /* 'STM32.TXT' file empty, write default data to file */
            /*##-5- Write data to the text file ################################*/
						res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
						if((byteswritten == 0) || (res != FR_OK))
						{
							/* 'STM32.TXT' file Write or EOF Error */
							Error_Handler();
						}
						else
							f_close(&MyFile);
						TXT_2_NUM(wtext);
					}
					
          else
					{
						TXT_2_NUM(rtext);
						/*##-6- Close the open text file #################################*/
						f_close(&MyFile);
					}            
      }
    }
  }
  
  /*##-11- Unlink the micro SD disk I/O driver ###############################*/
  FATFS_UnLinkDriver(SDPath);		
	
	//NUM_2_TXT(text);
	
}

/**
  * @brief  Read config data file from SD Card
  * @param  None
  * @retval None
  */
void WRITE_CONFIG_File(void)
{
	FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytes;                     /* File write/read counts */
  uint8_t wtext[500]; 																	/* File write buffer */
  //uint8_t rtext[100];                                   /* File read buffer */
  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      if(f_unlink ("RC.TXT") != FR_OK)
      {
          /* 'STM32.TXT' file Open for write Error */
          Error_Handler();
      }
      else
      {
        if(f_open(&MyFile, "RC.TXT", FA_OPEN_ALWAYS | FA_WRITE|FA_READ) != FR_OK)
				{
          /* 'STM32.TXT' file Open for write Error */
          Error_Handler();
				}
				else
				{
          //writ to the file
					bytes=NUM_2_TXT(wtext);
					res = f_write(&MyFile, wtext, bytes, (void *)&byteswritten);
					if((byteswritten == 0) || (res != FR_OK))
					{
						/* 'STM32.TXT' file Write or EOF Error */
						Error_Handler();
					}
					else
						f_close(&MyFile);						
        }
			}
		}
  }
  /*##-11- Unlink the micro SD disk I/O driver ###############################*/
  FATFS_UnLinkDriver(SDPath);		
}

/**
  * @brief  Read config data file from SD Card
  * @param  None
  * @retval None
  */
uint32_t NUM_2_TXT(uint8_t* txt)
{
	uint32_t i=0,j=0,ch=0,model;
	uint8_t tmp[3];
	uint32_t num;
	uint32_t data[5];	
	for(model=0;model<4;model++)
	{
		for(ch=0;ch<4;ch++)
		{ 
			data[0]=(uint32_t)(_ModelData[model][ch].Model);
			data[1]=(uint32_t)(_ModelData[model][ch].SubTrim+50);
			data[2]=(uint32_t)_ModelData[model][ch].Endpoint_p;
			data[3]=(uint32_t)_ModelData[model][ch].Endpoint_n;
			data[4]=(uint32_t)_ModelData[model][ch].Reverse;	
			for(j=0;j<5;j++)
			{
				tmp[0]=data[j]/100;
				num=data[j]%100;
				tmp[1]=num/10;
				tmp[2]=num%10;
				if(tmp[0]!=0)
				{
					*(txt+i)=tmp[0]+48;
					i++;
					*(txt+i)=tmp[1]+48;
					i++;
					*(txt+i)=tmp[2]+48;
					i++;
				}
				else if(tmp[1]!=0)
				{
					*(txt+i)=tmp[1]+48;
					i++;
					*(txt+i)=tmp[2]+48;
					i++;
				}		
				else
				{
					*(txt+i)=tmp[2]+48;
					i++;
				}
				if(j==4)			
					*(txt+i)=';';
				else
					*(txt+i)=',';
				i++;
			}
		}
	}	
	//backlight
	*(txt+i)='5';
	i++;
	*(txt+i)='1';
	i++;
	*(txt+i)=',';
	i++;
	data[0]=BACKLIGHT;
	tmp[0]=data[0]/100;
	num=data[0]%100;
	tmp[1]=num/10;
	tmp[2]=num%10;
	if(tmp[0]!=0)
	{
		*(txt+i)=tmp[0]+48;
		i++;
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}
	else if(tmp[1]!=0)
	{
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}		
	else
	{
		*(txt+i)=tmp[2]+48;
		i++;
	}
	*(txt+i)=';';
	i++;
	
	//model
	*(txt+i)='6';
	i++;
	*(txt+i)='1';
	i++;
	*(txt+i)=',';
	i++;
	data[0]=MODEL;
	tmp[0]=data[0]/100;
	num=data[0]%100;
	tmp[1]=num/10;
	tmp[2]=num%10;
	if(tmp[0]!=0)
	{
		*(txt+i)=tmp[0]+48;
		i++;
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}
	else if(tmp[1]!=0)
	{
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}		
	else
	{
		*(txt+i)=tmp[2]+48;
		i++;
	}
	*(txt+i)=';';
	i++;
	
	//ADC REF
	*(txt+i)='7';
	i++;
	*(txt+i)='1';
	i++;
	*(txt+i)=',';
	i++;
	data[0]=ADC_REF;
	tmp[0]=data[0]/100;
	num=data[0]%100;
	tmp[1]=num/10;
	tmp[2]=num%10;
	if(tmp[0]!=0)
	{
		*(txt+i)=tmp[0]+48;
		i++;
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}
	else if(tmp[1]!=0)
	{
		*(txt+i)=tmp[1]+48;
		i++;
		*(txt+i)=tmp[2]+48;
		i++;
	}		
	else
	{
		*(txt+i)=tmp[2]+48;
		i++;
	}
	*(txt+i)=';';
	i++;
	*(txt+i)='\0';
	
	return i;
}


/**
  * @brief  Read config data file from SD Card
  * @param  None
  * @retval None
  */
void TXT_2_NUM(uint8_t* txt)
{
	uint32_t i=0,j=0,ch=0,k=0,model=0;
	uint8_t tmp[3];
	uint32_t num;
	uint32_t data[5];	
	//Typdef_ModelData *_ModelData[4];
	for(i=0;*(txt+i)!='\0';i++)
  { 
		if((*(txt+i)!=';')&&(*(txt+i)!=','))
		{	
			tmp[j]=*(txt+i);
			j++;
		}
		else
		{
			if(j==1)
				num=tmp[0]-48;
			else if(j==2)
				num=10*(tmp[0]-48)+(tmp[1]-48);
			else if(j==3)
				num=100*(tmp[0]-48)+10*(tmp[1]-48)+(tmp[2]-48);
			
			j=0;
			data[k]=num;
			k++;
		}
		
		if(*(txt+i)==';')
		{
			k=0;
			model=data[0]/10-1;
			ch=data[0]%10-1;
			if((ch<4)&&(model<4))
			{
				_ModelData[model][ch].Model=data[0];
				_ModelData[model][ch].SubTrim=(signed int)(data[1]-50);				
				_ModelData[model][ch].Endpoint_p=data[2];
				_ModelData[model][ch].Endpoint_n=data[3];
				_ModelData[model][ch].Reverse=data[4];
				//ch++;				
			}
			//if(ch==4)
			////{
			//	ch=0;
				//model++;
			//}
			if(model==4)
			{
				//model++;
				BACKLIGHT=data[1];			
			}
			if(model==5)
				MODEL=data[1];
			if(model==6)
				ADC_REF=data[1];
		}			
	}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t NRF24L01_IRQ_status;
	//uint32_t=10000;
  if((GPIO_Pin == TOUCH_INT_PIN)&&BACKLIGHT_ON)										//PB7 TOUCH INT
  {

    if(BSP_TOUCH_Read(TOUCH_Dat)==0)
		{	/*
			BSP_LCD_DrawPixel(TOUCH_Dat.x, TOUCH_Dat.y, RED);
			BSP_LCD_DrawPixel(TOUCH_Dat.x, TOUCH_Dat.y+1, RED);
			BSP_LCD_DrawPixel(TOUCH_Dat.x+1, TOUCH_Dat.y, RED);
			BSP_LCD_DrawPixel(TOUCH_Dat.x+1, TOUCH_Dat.y, RED);
			*/
			Touch_State.Layer=0;
			Touch_State.x=TOUCH_Dat.x;
			Touch_State.y=TOUCH_Dat.y;
			Touch_State.Pressed=TOUCH_Dat.pressed;
			GUI_PID_StoreState(&Touch_State);
			
			BSP_LED_Toggle(LEDG);
		}
		else
			Touch_State.Pressed=TOUCH_Dat.pressed;
			GUI_PID_StoreState(&Touch_State);
			BSP_LED_Toggle(LEDR);
  }
	
	else if(GPIO_Pin == KEY_BUTTON_PIN)									//PA0 KEY INT
  {
    BSP_LED_Toggle(LEDR);
		//GUI_Delay(20);
		
		if(BACKLIGHT_ON)
		{
			//BSP_LCD_BACKLIGHT_PWM_Set(0);
			BACKLIGHT_ON=0;
			BSP_LCD_BACKLIGHT_Off();
		}
		else
		{
			BSP_LCD_BACKLIGHT_PWM_Set(BACKLIGHT);
			BACKLIGHT_ON=1;
			
		}
		
#ifdef DEBUG_MODE		
		NRF24L01_TxPacket( (uint8_t *)NRF24L01_TX_DATA, NRF24L01_TX_NUM );


		if (duty==0)
		{	LCD_BACKLIGHT_PWM_50duty();
			duty=1;
		}
			else 
		{
			LCD_BACKLIGHT_PWM_25duty();
			duty=0;
		}
#endif		
  }
	
	else if(GPIO_Pin == RF24L01_IRQ_GPIO_PIN)						//RF24L01 IRQ
	{
#ifdef DEBUG_MODE
		NRF24L01_IRQ_status=NRF24L01_IRQ((uint8_t *) &NRF24L01_ACK_DATA,NRF24L01_ACK_NUM);
		if(NRF24L01_IRQ_status ==(TX_OK|RX_OK))
		{	
			for(i=0;i<8;i++)
			{
				if(NRF24L01_ACK[i]!=NRF24L01_ACK_DATA[i])
					return;
			}	
			BSP_LED_Toggle(LED4);			
		}
#else
		NRF24L01_IRQ_status=NRF24L01_IRQ((uint8_t *)&RC_DATA.RC_ACK_Data.RX_BAT_L,NRF24L01_ACK_NUM);	
		if(NRF24L01_IRQ_status ==(TX_OK|RX_OK))
		{	
			RC_DATA.RX_BAT=(uint16_t)RC_DATA.RC_ACK_Data.RX_BAT_H;
			RC_DATA.RX_BAT=(RC_DATA.RX_BAT <<8) + RC_DATA.RC_ACK_Data.RX_BAT_L;
			BSP_LED_Toggle(LEDG);			
		}
#endif		
	}
	
}

/**
  * @brief  ����ת������ADC����
  * @param  RC_DATA_TypeDef
  * @retval None
  */
void RC_TX_DataHandle(RC_DATA_TypeDef* rc_data,Typdef_ModelData *_ModelData)
{
	uint32_t tmp;
	uint32_t temp,temp_mid;
	
	//��������
	temp_mid=rc_data->ADC_StrMid	+(_ModelData[0].SubTrim*12);				
	/////////////////////////////////////////////////////////////////////////
	//ת�����С�ڻ���ֵ������ת, 0-100
	if(temp_mid>rc_data->ADC_DATA.ADC_STR)
	{	
		tmp=(100*(temp_mid-rc_data->ADC_DATA.ADC_STR))/temp_mid;
		tmp=-(tmp*_ModelData[0].Endpoint_n/100);
	}
	//ת��������ڻ���ֵ������ת 100-200
	else
	{
		tmp=(100*(rc_data->ADC_DATA.ADC_STR-temp_mid))/(0xFFF-temp_mid);
		tmp=tmp*_ModelData[0].Endpoint_p/100;					
	}
	
	//����
	if(_ModelData[0].Reverse==1)
			tmp=-tmp;
	rc_data->RC_TX_Data.CHANNEL1=100+tmp;
	
	///////////////////////////////////////////////////////////////////////////
	//����channel1 ָʾ����
	if(rc_data->RC_TX_Data.CHANNEL1<=100)
	{
		tmp=(45*rc_data->RC_TX_Data.CHANNEL1)/100;
		DASH_BOARD[0]=135+tmp;
	}
	else
	{
		tmp=(45*(rc_data->RC_TX_Data.CHANNEL1-100))/100;
		DASH_BOARD[0]=180+tmp;
	}
	
	///////////////////////////////////////////////////////////////////////////
	//���Ų���С�ڻ���ֵ����ǰ��100-200
	temp_mid=rc_data->ADC_ThrtMid	+(_ModelData[1].SubTrim*12);	
	if(temp_mid>rc_data->ADC_DATA.ADC_THRT)
	{	
		tmp=(100*(temp_mid-rc_data->ADC_DATA.ADC_THRT))/temp_mid;
		tmp=-(tmp*_ModelData[1].Endpoint_n/100);
	}
	//ת��������ڻ���ֵ�����0-100
	else
	{
		tmp=(100*(rc_data->ADC_DATA.ADC_THRT-temp_mid))/(0xFFF-temp_mid);
		tmp=tmp*_ModelData[1].Endpoint_p/100;
	}
	
	if(_ModelData[1].Reverse==1)
			tmp=-tmp;	
	rc_data->RC_TX_Data.CHANNEL2=100+tmp;
	
	///////////////////////////////////////////////////////////////////////////
	//����channel2 ָʾ����
	if(rc_data->RC_TX_Data.CHANNEL2<100)
	{
		tmp=(45*rc_data->RC_TX_Data.CHANNEL2)/100;
		DASH_BOARD[1]=135+tmp;
	}
	else
	{
		tmp=(45*(rc_data->RC_TX_Data.CHANNEL2-100))/100;
		DASH_BOARD[1]=180+tmp;
	}	
	
	///////////////////////////////////////////////////////////////////////////
	//����channel3 ָʾ����
	tmp=100;
	if(_ModelData[2].Reverse==1)
	{
		if(CH3_Switch==1)
			tmp=tmp-_ModelData[2].Endpoint_n;
		else if(CH3_Switch==2) 
			tmp+=_ModelData[2].Endpoint_p;
		else if(CH3_Switch==0)
			tmp=tmp-_ModelData[2].SubTrim;
	}
	else
	{
		if(CH3_Switch==1)
			tmp+=_ModelData[2].Endpoint_p;
		else if(CH3_Switch==2) 
			tmp=tmp-_ModelData[2].Endpoint_n;
		else if(CH3_Switch==0)
			tmp+=_ModelData[2].SubTrim;
	}
	rc_data->RC_TX_Data.CHANNEL3=tmp;

	if(rc_data->RC_TX_Data.CHANNEL3<=100)
	{
		tmp=(45*rc_data->RC_TX_Data.CHANNEL3)/100;
		DASH_BOARD[2]=135+tmp;
	}
	else
	{
		tmp=(45*(rc_data->RC_TX_Data.CHANNEL3-100))/100;
		DASH_BOARD[2]=180+tmp;
	}
	
	///////////////////////////////////////////////////////////////////////////
	//����channel4 ָʾ����
	tmp=100;
	if(_ModelData[3].Reverse==1)
	{
		if(CH4_Switch==1)
			tmp=tmp-_ModelData[3].Endpoint_n;
		else if(CH4_Switch==2) 
			tmp+=_ModelData[3].Endpoint_p;
		else
			tmp=tmp-_ModelData[3].SubTrim;
	}
	else
	{
		if(CH4_Switch==1)
			tmp+=_ModelData[3].Endpoint_p;
		else if(CH4_Switch==2) 
			tmp=tmp-_ModelData[3].Endpoint_n;	
		else
			tmp+=_ModelData[3].SubTrim;
	}
	rc_data->RC_TX_Data.CHANNEL4=tmp;

	if(rc_data->RC_TX_Data.CHANNEL4<=100)
	{
		tmp=(45*rc_data->RC_TX_Data.CHANNEL4)/100;
		DASH_BOARD[3]=135+tmp;
	}
	else
	{
		tmp=(45*(rc_data->RC_TX_Data.CHANNEL4-100))/100;
		DASH_BOARD[3]=180+tmp;
	}
	
	//�¶ȴ���, ��λ���϶�
	//(Vsense-V25)/Vslope+25, Vref=2890mV, Vsense=Vref*Adc>>12, V25=760mV, Vslope=2.5mV/C     
	temp=(10*(((rc_data->ADC_DATA.ADC_TMP*(3260+ADC_REF-50))>>12)-760))/25+25;
	ADC_Tmp_Num +=temp;

	
	
	//��ص�ѹ������λmV
	//Vsense=Vref*Adc>>12, Vref=3250mV  
	temp=(((rc_data->ADC_DATA.ADC_BAT*(3260+ADC_REF-50))>>12)/2)*3;
	ADC_Bat_Num+=temp;
	
	ADC_Count++;
	if(ADC_Count==10)
	{
		rc_data->TX_BAT=(ADC_Bat_Num/ADC_Count);
		rc_data->TX_TMP=(ADC_Tmp_Num/ADC_Count);
		ADC_Count=0;
		ADC_Bat_Num=0;
		ADC_Tmp_Num=0;
	}
}

void SRAM_Test()
{
	//uint8_t uwWriteReadStatus;
	uint32_t uwIndex;
	/*##-2- SRAM memory read/write access ######################################*/  
  /* Fill the buffer to write */
	Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0xC20F);   
  /*
	for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
  {
    if(uwIndex%2==1)
			aTxBuffer[uwIndex]=0xFFFF;
		else
			aTxBuffer[uwIndex]=0;
  }
	*/
  /* Write data to the SRAM memory */
  for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
  {
    *(__IO uint16_t*) (SRAM_BANK_ADDR + WRITE_READ_ADDR + 2*uwIndex) = aTxBuffer[uwIndex];
  }    
  
  /* Read back data from the SRAM memory */
  for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
  {
    aRxBuffer[uwIndex] = *(__IO uint16_t*) (SRAM_BANK_ADDR + WRITE_READ_ADDR + 2*uwIndex);
  } 

  /*##-3- Checking data integrity ############################################*/    
  //uwWriteReadStatus = Buffercmp(aTxBuffer, aRxBuffer, BUFFER_SIZE);	
	
	
}


/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the buffer to fill
  * @param  uwBufferLenght: size of the buffer to fill
  * @param  uwOffset: first value to fill on the buffer
  * @retval None
  */
static void Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uwOffset)
{
  uint16_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
  }
} 

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer identical to pBuffer1
  *         FAILED: pBuffer differs from pBuffer1
  */
uint8_t Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 0;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 1;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LEDR);
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

int fputc(int ch, FILE *f)
{
	USART2->DR = ((uint8_t)ch & (uint16_t)0x01FF);
	while (!(USART2->SR & USART_SR_TXE));
	return (ch);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
