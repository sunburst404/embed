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
  * COPYRIGHT(c) 2023 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"  
#include "lcd.h"
#include "touch.h" 
#include "pic.h" 
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
uint8_t Buff[30]; // 用于接收数据的缓冲区

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t touchflag = 0;
#define BEEP              	1
#define RELAY              	2
#define DCMOTOR             3
#define STREE								4
#define STEP								5
#define SPIFLASH						6
#define LED     						7
#define RFID								8
#define ALARM_NUMBER				2

const uint16_t origX = 20;
const uint16_t origY = 100;

int dateYearInt, dateMonthInt, dateDayInt, dateWeekdayInt;
int timeHourInt, timeMinuteInt, timeSecondInt;
int alarmEnableInt[ALARM_NUMBER], alarmHourInt[ALARM_NUMBER], alarmMinuteInt[ALARM_NUMBER];

uint8_t dateYearStr[5];
uint8_t dateMonthStr[5];
uint8_t dateDayStr[5];

const uint16_t dateYearWidth = 50;
const uint16_t dateMonthWidth = 36;
const uint16_t dateDayWidth = 36;

uint8_t timeHourStr[5];
uint8_t timeMinuteStr[5];
uint8_t timeSecondStr[5];

const uint16_t timeHourWidth = 25;
const uint16_t timeMinuteWidth = 36;
const uint16_t timeSecondWidth = 36;

uint8_t alarmEnableStr[ALARM_NUMBER][5];
uint8_t alarmHourStr[ALARM_NUMBER][5];
uint8_t alarmMinuteStr[ALARM_NUMBER][5];

const uint16_t alarmEnableWidth = 50;
const uint16_t alarmHourWidth = 25;
const uint16_t alarmMinuteWidth = 36;

#define MODE_CLOCK			0
#define MODE_SET_DATE		1
#define MODE_SET_TIME		2
#define MODE_SET_ALARM 	3
#define MODE_MAX        3
uint8_t modeInt = MODE_CLOCK;
uint8_t *modeStr[] = {
	"Clock    ",
	"SetDate  ",
	"SetTime  ", 
	"SetAlarm "
};

int focusInt = 0;
int focusMax = 0;
uint16_t *focusPrev = NULL;
uint16_t *focusCurr = NULL;

uint16_t modePos[] = {0};

const uint16_t lineLabelWidth = 120;

uint16_t dateFocus[][3] =  {
	{origX + lineLabelWidth,                                  origY+50+20, dateYearWidth},
	{origX + lineLabelWidth + dateYearWidth, 					  		  origY+50+20, dateMonthWidth},
	{origX + lineLabelWidth + dateYearWidth + dateMonthWidth, origY+50+20, dateDayWidth}
};

uint16_t timeFocus[][3] = {
	{origX + lineLabelWidth,                                  origY+100+20, timeHourWidth},
	{origX + lineLabelWidth + timeHourWidth,                  origY+100+20, timeMinuteWidth},
	{origX + lineLabelWidth + timeHourWidth + timeMinuteWidth,origY+100+20, timeSecondWidth}
};

uint16_t alarmFocus[][3] = {
	{origX + lineLabelWidth,                                    origY+150+20, alarmEnableWidth},
	{origX + lineLabelWidth + alarmEnableWidth,                 origY+150+20, alarmHourWidth},
	{origX + lineLabelWidth + alarmEnableWidth + alarmHourWidth,origY+150+20, alarmMinuteWidth},
	{origX + lineLabelWidth,                                    origY+200+20, alarmEnableWidth},
	{origX + lineLabelWidth + alarmEnableWidth,                 origY+200+20, alarmHourWidth},
	{origX + lineLabelWidth + alarmEnableWidth + alarmHourWidth,origY+200+20, alarmMinuteWidth}
};

uint8_t setTimeFlag = 0;
uint8_t setDateFlag = 0;
uint8_t setAlarmFlag[ALARM_NUMBER] = {0};

uint8_t alarmFlag = 0;

uint8_t rtcDateStr[100] = {0};
uint8_t rtcTimeStr[100] = {0};



void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);//清屏   
 	POINT_COLOR=BLUE;//设置字体为蓝色 
	LCD_ShowString(lcddev.width-24,0,200,16,16,"RST");//显示清屏区域
	//POINT_COLOR=RED;//设置画笔蓝色 
}

////////////////////////////////////////////////////////////////////////////////
//电容触摸屏专有部分
//画水平线
//x0,y0:坐标
//len:线长度
//color:颜色
void gui_draw_hline(uint16_t x0,uint16_t y0,uint16_t len,uint16_t color)
{
	if(len==0)return;
	LCD_Fill(x0,y0,x0+len-1,y0,color);	
}
//画实心圆
//x0,y0:坐标
//r:半径
//color:颜色
void gui_fill_circle(uint16_t x0,uint16_t y0,uint16_t r,uint16_t color)
{											  
	uint32_t i;
	uint32_t imax = ((uint32_t)r*707)/1000+1;
	uint32_t sqmax = (uint32_t)r*(uint32_t)r+(uint32_t)r/2;
	uint32_t x=r;
	gui_draw_hline(x0-r,y0,2*r,color);
	for (i=1;i<=imax;i++) 
	{
		if ((i*i+x*x)>sqmax)// draw lines from outside  
		{
 			if (x>imax) 
			{
				gui_draw_hline (x0-i+1,y0+x,2*(i-1),color);
				gui_draw_hline (x0-i+1,y0-x,2*(i-1),color);
			}
			x--;
		}
		// draw lines from inside (center)  
		gui_draw_hline(x0-x,y0+i,2*x,color);
		gui_draw_hline(x0-x,y0-i,2*x,color);
	}
}  
//两个数之差的绝对值 
//x1,x2：需取差值的两个数
//返回值：|x1-x2|
uint16_t my_abs(uint16_t x1,uint16_t x2)
{			 
	if(x1>x2)return x1-x2;
	else return x2-x1;
}  
//画一条粗线
//(x1,y1),(x2,y2):线条的起始坐标
//size：线条的粗细程度
//color：线条的颜色
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint8_t size,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	if(x1<size|| x2<size||y1<size|| y2<size)return; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		gui_fill_circle(uRow,uCol,size,color);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}   
////////////////////////////////////////////////////////////////////////////////
 //5个触控点的颜色(电容触摸屏用)												 
const uint16_t POINT_COLOR_TBL[OTT_MAX_TOUCH]={RED,GREEN,BLUE,BROWN,GRED};  


//电容触摸屏测试函数
void ctp_test(void)
{
	//uint8_t key;
	uint8_t t=0;
	//uint8_t i=0;	  	    
	//uint16_t line_x=0,line_y=0;	
 	uint16_t lastpos[5][2];		//最后一次的数据 
	while(1)
	{	
		//key=KEY_Scan(0);
		tp_dev.scan(0);
		for(t=0;t<OTT_MAX_TOUCH;t++)
		{
			if((tp_dev.sta)&(1<<t))
			{
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height)
				{
					if(lastpos[t][0]==0XFFFF)
					{
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//画线
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					//line_x=tp_dev.x[t];
					//line_y=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-24)&&tp_dev.y[t]<20)
					{
						Load_Drow_Dialog();//清除
					}
				}
			}else lastpos[t][0]=0XFFFF;
		}
		delay_ms(5);
	}	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
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
  MX_USART1_UART_Init();
  MX_FSMC_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
	
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	
  /* USER CODE BEGIN 2 */
	uint8_t lcd_id[12];				//存放LCD ID字符串
	delay_init(168);  //初始化延时函数
 	LCD_Init();					//LCD初始化 
	tp_dev.init();				//触摸屏初始化
	POINT_COLOR=BLUE;	 
	sprintf((char*)lcd_id,"ID:%04X",lcddev.id);//将LCD ID打印到lcd_id数组。		
  LCD_ShowString(30,40,210,24,24,"STM32F407IGT6");
	LCD_ShowString(30,70,200,16,16,"TFTLCD TEST");
	LCD_ShowString(170,420,200,24,24,"FS_TFTLCD_V1.0");
 	LCD_ShowString(30,110,200,16,16,lcd_id);		//显示LCD ID	      					  
	//HAL_Delay(2000);
 	Load_Drow_Dialog();	 
	//LCD_DrawPicture(0,300,480,500,(uint8_t *)gImage_hqyj);

	//if(tp_dev.touchtype&0X80)ctp_test();//电容屏测试
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//初始化日期 时间
	RTC_DateTypeDef date;
	date.Year = 23;
	date.Month = 1;
	date.Date = 1;
	date.WeekDay = 3;
	HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
	
	RTC_TimeTypeDef time;
	time.Hours = 0;
	time.Minutes = 0;
	time.Seconds = 57;
	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
	
	alarmEnableInt[0] = 1;
	alarmHourInt[0] = 0;
	alarmMinuteInt[0] = 1;
	
	alarmEnableInt[1] = 0;
	alarmHourInt[1] = 0;
	alarmMinuteInt[1] = 1;
	
	for(int i = 0; i < ALARM_NUMBER; i++) {
		sprintf((char*)alarmEnableStr[i], "%s", alarmEnableInt[i] ? "On ":"Off");
		sprintf((char*)alarmHourStr[i], "%02d", alarmHourInt[i]);
		sprintf((char*)alarmMinuteStr[i], ":%02d", alarmMinuteInt[i]);
	}


	

	HAL_TIM_Base_Start_IT(&htim2);
	  	
	LCD_ShowString(origX, origY,     lineLabelWidth, 24, 24, "Mode    :");
	LCD_ShowString(origX, origY+50,  lineLabelWidth, 24, 24, "Date    :");
	LCD_ShowString(origX, origY+100, lineLabelWidth, 24, 24, "Time    :");
	LCD_ShowString(origX, origY+150, lineLabelWidth, 24, 24, "Alarm   :");
	LCD_ShowString(origX, origY+450, lineLabelWidth, 24, 24, "RTC Date:");
	LCD_ShowString(origX, origY+500, lineLabelWidth, 24, 24, "RTC Time:");
	
	/*
	char keyStr[5];
	for (int i = 0; i < 4; i++) {
		sprintf(keyStr, "Key%d", i+1);
		LCD_DrawRectangle(origX + i*120, origY+500, origX + i*120 + 80, origY+500+100);
		LCD_ShowString(origX + i*120 + 20, origY+500+40, 60, 24, 24, (uint8_t*)keyStr);
	}*/

	////////////////////////////////////////////////////////////
	while (1)
  {

  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		
		if (setTimeFlag) {
			RTC_TimeTypeDef time;
			time.Hours = timeHourInt;
			time.Minutes = timeMinuteInt;
			time.Seconds = timeSecondInt;
			HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
			setTimeFlag = 0;
		}
		if (setDateFlag) {
			RTC_DateTypeDef date;
			date.Year = dateYearInt;
			date.Month = dateMonthInt;
			date.Date = dateDayInt;
			date.WeekDay = dateWeekdayInt;
			HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
			setDateFlag = 0;
		}
		
		
		if (focusPrev) {
			LCD_Fill(focusPrev[0], focusPrev[1], focusPrev[0]+focusPrev[2], focusPrev[1]+1, WHITE);
			focusPrev = NULL;
		}
		if (MODE_CLOCK != modeInt) {
			LCD_Fill(focusCurr[0], focusCurr[1], focusCurr[0]+focusCurr[2], focusCurr[1]+1, BLACK);
		}
		
		LCD_ShowString(origX + lineLabelWidth, origY, 200, 24, 24, modeStr[modeInt]);

		LCD_ShowString(origX + lineLabelWidth,                                  origY+50,  dateYearWidth,  24, 24, dateYearStr);
		LCD_ShowString(origX + lineLabelWidth + dateYearWidth,                  origY+50,  dateMonthWidth, 24, 24, dateMonthStr);
		LCD_ShowString(origX + lineLabelWidth + dateYearWidth + dateMonthWidth, origY+50,  dateDayWidth,   24, 24, dateDayStr);

		LCD_ShowString(origX + lineLabelWidth,                                  origY+100, timeHourWidth,  24, 24, timeHourStr);
		LCD_ShowString(origX + lineLabelWidth + timeHourWidth,                  origY+100, timeMinuteWidth,24, 24, timeMinuteStr);
		LCD_ShowString(origX + lineLabelWidth + timeHourWidth + timeMinuteWidth,origY+100, timeSecondWidth,24, 24, timeSecondStr);

		for(int i = 0; i < ALARM_NUMBER; i++) {
			LCD_ShowString(origX + lineLabelWidth,                                    origY+150+50*i, alarmEnableWidth,24, 24, alarmEnableStr[i]);
			LCD_ShowString(origX + lineLabelWidth + alarmEnableWidth,                 origY+150+50*i, alarmHourWidth,  24, 24, alarmHourStr[i]);
			LCD_ShowString(origX + lineLabelWidth + alarmEnableWidth + alarmHourWidth,origY+150+50*i, alarmMinuteWidth,24, 24, alarmMinuteStr[i]);
			
		}
		
		LCD_ShowString(origX + lineLabelWidth, origY+450, 300, 24, 24, rtcDateStr);
		LCD_ShowString(origX + lineLabelWidth, origY+500, 300, 24, 24, rtcTimeStr);

		//HAL_UART_Receive_IT(&huart1, Buff, 1);//中断方式接收，中断回调函数里发送Buff
		HAL_UART_Receive_IT(&huart1, Buff, 30);
		
		delay_ms(5);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//用户中断接收回调函数如下
//////////////////////////////have problem/////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//处理发送的信息
	/*
	if (huart->Instance == USART1) // 确认是正确的UART实例
			{
        messageBuffer[messageLength] = Buff[0]; // 将接收到的数据添加到消息缓冲区
        messageLength++; // 增加消息长度

        if (messageLength == 14) 
        {
            // 执行对应的程序
						int alarmNum;
						int hour;
						int minute;
						if(sscanf((char*)Buff, "alarm %d %d:%d",&alarmNum, &hour, &minute)){
							alarmEnableInt[alarmNum-1] = 1;
							alarmHourInt[alarmNum-1] = hour;
							alarmMinuteInt[alarmNum-1] = minute;
							sprintf((char*)alarmEnableStr[alarmNum-1], "%s", alarmEnableInt[alarmNum-1] ? "On":"Off");
							sprintf((char*)alarmHourStr[alarmNum-1], "%02d", alarmHourInt[alarmNum-1]);
							sprintf((char*)alarmMinuteStr[alarmNum-1], ":%02d", alarmMinuteInt[alarmNum-1]);
							
						}
        }

        HAL_UART_Receive_IT(&huart1, Buff, 1);
    }*/
		//HAL_UART_Transmit(&huart1,messageBuffer ,30, 100);
  HAL_UART_Transmit(&huart1,Buff , sizeof(Buff), 100);
	int alarmNum;
	int hour;
	int minute;
	if(sscanf((char*)Buff, "now 20%d-%d-%d %d:%d:%d %d", &dateYearInt, &dateMonthInt, &dateDayInt, &timeHourInt, &timeMinuteInt, &timeSecondInt, &dateWeekdayInt)){
		printf("change");
		setDateFlag=1;
		setTimeFlag=1;
		sprintf((char*)dateDayStr, "-%02d", dateDayInt);
		sprintf((char*)dateMonthStr, "-%02d", dateMonthInt);
		sprintf((char*)dateYearStr, "20%02d", dateYearInt);

		sprintf((char*)timeHourStr, "%02d", timeHourInt);
		sprintf((char*)timeMinuteStr, ":%02d", timeMinuteInt);
		sprintf((char*)timeSecondStr, ":%02d", timeSecondInt);
	}

	else if(sscanf((char*)Buff, "alarm %d %d:%d",&alarmNum, &hour, &minute)==3){
		printf("add %d",alarmNum);
		alarmEnableInt[alarmNum-1] = 1;
		alarmHourInt[alarmNum-1] = hour;
		alarmMinuteInt[alarmNum-1] = minute;
		sprintf((char*)alarmEnableStr[alarmNum-1], "%s", alarmEnableInt[alarmNum-1] ? "On ":"Off");
		sprintf((char*)alarmHourStr[alarmNum-1], "%02d", alarmHourInt[alarmNum-1]);
		sprintf((char*)alarmMinuteStr[alarmNum-1], ":%02d", alarmMinuteInt[alarmNum-1]);
		
	}
	else if(sscanf((char*)Buff, "alarm %d delete",&alarmNum)){
		printf("delete %d",alarmNum);
		alarmEnableInt[alarmNum-1] = 0;
		sprintf((char*)alarmEnableStr[alarmNum-1], "%s", alarmEnableInt[alarmNum-1] ? "On ":"Off");
	}
	
}

int fputc(int ch, FILE *f)
{ 
  uint8_t tmp[1]={0};
	tmp[0] = (uint8_t)ch;
	HAL_UART_Transmit(&huart1,tmp,1,10);	
	return ch;
}

//按键操作
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == KEY1_Pin) {
		if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) {
			printf("Key1\r\n");
			if (MODE_SET_TIME == modeInt) {
				setTimeFlag = 1;
			}
			if (MODE_SET_DATE == modeInt) {
				setDateFlag = 1;
			}

			modeInt++;
			if (MODE_MAX < modeInt) {
				modeInt = 0;
			}
			focusInt = 0;
			focusPrev = focusCurr;
			if (MODE_SET_DATE == modeInt) {
				focusCurr = dateFocus[0];
				focusMax = 3;
			}
			else if (MODE_SET_TIME == modeInt) {
				focusCurr = timeFocus[0];
				focusMax = 3;
			}
			else if (MODE_SET_ALARM == modeInt) {
				focusCurr = alarmFocus[0];
				focusMax = 3 * ALARM_NUMBER;
			}
		}
	}
	else if (GPIO_Pin == KEY2_Pin)
	{
		if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
			printf("Key2\r\n");
			if (MODE_CLOCK != modeInt) {
				focusPrev = focusCurr;
				focusInt++;
				if (focusMax == focusInt) {
					focusInt = 0;
				}
				if (MODE_SET_DATE == modeInt) {
					focusCurr = dateFocus[focusInt];
				}
				else if (MODE_SET_TIME == modeInt) {
					focusCurr = timeFocus[focusInt];
				}
				else if (MODE_SET_ALARM == modeInt) {
					focusCurr = alarmFocus[focusInt];
				}
			}
		}
	}
	else if (GPIO_Pin == KEY3_Pin)
	{
		if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET) {
			printf("Key3\r\n");
			if (MODE_SET_DATE == modeInt) {
				if (0 == focusInt && 0 < dateYearInt) {
					dateYearInt--;				
				}
				if (1 == focusInt && 0 < dateMonthInt) {
					dateMonthInt--;
				}
				if (2 == focusInt && 0 < dateDayInt) {
					dateDayInt--;
				}
				sprintf((char*)dateDayStr, "-%02d", dateDayInt);
				sprintf((char*)dateMonthStr, "-%02d", dateMonthInt);
				sprintf((char*)dateYearStr, "20%02d", dateYearInt);
			}
			if (MODE_SET_TIME == modeInt) {
				if (0 == focusInt && 0 < timeHourInt) {
					timeHourInt--;
				}
				if (1 == focusInt && 0 < timeMinuteInt) {
					timeMinuteInt--;
				}
				if (2 == focusInt && 0 < timeSecondInt) {
					timeSecondInt--;
				}
				sprintf((char*)timeHourStr, "%02d", timeHourInt);
				sprintf((char*)timeMinuteStr, ":%02d", timeMinuteInt);
				sprintf((char*)timeSecondStr, ":%02d", timeSecondInt);
			}
			if (MODE_SET_ALARM == modeInt) {
				for(int i = 0; i < ALARM_NUMBER; i++) {
					if ((0 + 3*i) == focusInt){
						alarmEnableInt[i] ^= 1;
					}
					if ((1 + 3*i) == focusInt && 0 < alarmHourInt[i]){
						alarmHourInt[i]--;
					}
					if ((2 + 3*i) == focusInt && 0 < alarmMinuteInt[i]){
						alarmMinuteInt[i]--;
					}
					sprintf((char*)alarmEnableStr[i], "%s", alarmEnableInt[i] ? "On ":"Off");
					sprintf((char*)alarmHourStr[i], "%02d", alarmHourInt[i]);
					sprintf((char*)alarmMinuteStr[i], ":%02d", alarmMinuteInt[i]);
				}
				

			}	
		}
	}
	else if (GPIO_Pin == KEY4_Pin)
	{
		if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_SET) {
			printf("Key4\r\n");
			if (MODE_SET_DATE == modeInt) {
				if (0 == focusInt ) {
					dateYearInt++;
				}
				if (1 == focusInt && 13 > dateMonthInt) {
					dateMonthInt++;
				}
				if (2 == focusInt && 32 > dateDayInt) {
					dateDayInt++;
					
				}
				sprintf((char*)dateDayStr, "-%02d", dateDayInt);
				sprintf((char*)dateMonthStr, "-%02d", dateMonthInt);
				sprintf((char*)dateYearStr, "20%02d", dateYearInt);
			}
			if (MODE_SET_TIME == modeInt) {
				if (0 == focusInt && 23 > timeHourInt) {
					timeHourInt++;
				}
				if (1 == focusInt && 59 > timeMinuteInt) {
					timeMinuteInt++;
				}
				if (2 == focusInt && 59 > timeSecondInt) {
					timeSecondInt++;
				}
				sprintf((char*)timeHourStr, "%02d", timeHourInt);
				sprintf((char*)timeMinuteStr, ":%02d", timeMinuteInt);
				sprintf((char*)timeSecondStr, ":%02d", timeSecondInt);
			}
			if (MODE_SET_ALARM == modeInt) {
				for(int i = 0; i < ALARM_NUMBER; i++) {
					if ((0 + 3*i) == focusInt){
						alarmEnableInt[i] ^= 1;
					}
					if ((1 + 3*i) == focusInt && 23 > alarmHourInt[i]){
						alarmHourInt[i]++;
					}
					if ((2 + 3*i) == focusInt && 59 > alarmMinuteInt[i]){
						alarmMinuteInt[i]++;
					}
					sprintf((char*)alarmEnableStr[i], "%s", alarmEnableInt[i] ? "On ":"Off");
					sprintf((char*)alarmHourStr[i], "%02d", alarmHourInt[i]);
					sprintf((char*)alarmMinuteStr[i], ":%02d", alarmMinuteInt[i]);
				}
				

			}	
		}
	}
}

void HAL_SYSTICK_Callback(void)
{
	static int tickCnt = 0;
	static int secondCnt = 0;
	tickCnt++;
	if (tickCnt == 1000) {
		tickCnt = 0;
		secondCnt++;
		//printf("SysTick second=%d\r\n", secondCnt);
		
		RTC_TimeTypeDef time;
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		RTC_DateTypeDef date;
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
		
		for(int i = 0; i < ALARM_NUMBER; i++) {
			if (time.Hours == alarmHourInt[i] && time.Minutes == alarmMinuteInt[i] && 00 == time.Seconds) {
				alarmFlag = 5;
			}
			sprintf((char*)rtcDateStr, "20%02d-%02d-%02d Weekday(%d)", date.Year, date.Month, date.Date, date.WeekDay);
			sprintf((char*)rtcTimeStr, "%02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);

			if (MODE_CLOCK == modeInt) {
				sprintf((char*)dateYearStr, "20%02d", date.Year);
				sprintf((char*)dateMonthStr, "-%02d", date.Month);
				sprintf((char*)dateDayStr, "-%02d", date.Date);
				
				sprintf((char*)timeHourStr, "%02d", time.Hours);
				sprintf((char*)timeMinuteStr, ":%02d", time.Minutes);
				sprintf((char*)timeSecondStr, ":%02d", time.Seconds);
				
			}
		}
		
		//printf("RTC 20%02d-%02d-%02d %02d:%02d:%02d weekday=%d\r\n", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds, date.WeekDay);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int tim2Cnt = 0;
	static int secondCnt = 0;
	static int led_flash = 0;
	//static int beep = 0;
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(htim);
	if (htim->Instance == TIM2) {
		if (alarmFlag) {
			led_flash++;
			if(led_flash==500){
				led_flash=0;
				printf("----alarm----");
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			}
			//HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
			#if 0
			beep = 1 - beep;
			if (beep) {
				HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
			}
			#endif
		}
		tim2Cnt++;
		if (tim2Cnt == 1000) {
			tim2Cnt = 0;
			secondCnt++;
			if (alarmFlag) {
				alarmFlag--;
				if(!alarmFlag){
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				}
			}
			//printf("TIM second=%d\r\n", secondCnt);
		}
	}
}

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
