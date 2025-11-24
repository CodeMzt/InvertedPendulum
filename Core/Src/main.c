/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "delay.h"
#include "oled.h"
#include "Key.h"
#include "adc.h"
#include "Encoder.h"
#include "motor.h"
#include "../Inc/pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define   CENTER_ANGLE          2050
#define   CENTER_RANGE          500
#define   START_LOAD            40
#define   START_TIME            100
#define   START_SAMPLE_TIME     40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Mode,RunState;

uint16_t Angle;
int32_t Location;

PID_T AnglePID={
  .Kp = 0.2,
  .Ki = 0.01,
  .Kd = 0.45,

  .Target = CENTER_ANGLE,
  .Current = 0,
  .Last = 0,
  .Out = 0,
  .Err_Integral = 0,
  .Err_Last = 0,
  .Diff_Last = 0,
  .Flags = 0x02,
  .InputDeadZone_Threshold = 8,
  .Diff_Filter = 0.9,
  .IntegralSeparation_Threshold = 0,
  .VariableIntegral = 0,
  .OutputOffset_Threshold = 0,
  .OutputOffset = 0,
  .Out_Max = 100,
  .Out_Min = -100,
  .Err_Integral_Out_Max = 100,
  .Err_Integral_Out_Min = -100
},LocationPID={
  .Kp = -0.2 ,
  .Ki = -0.00,
  .Kd = -4.0,
  .Target = 0,
  .Current = 0,
  .Last = 0,
  .Out = 0,
  .Err_Integral = 0,
  .Err_Last = 0,
  .Diff_Last = 0,
  .Flags = 0x04,
  .InputDeadZone_Threshold = 5,
  .Diff_Filter = 0.95,
  .IntegralSeparation_Threshold = 0,
  .VariableIntegral = 0,
  .OutputOffset_Threshold = 4,
  .OutputOffset = 1,
  .Out_Max = 100,
  .Out_Min = -100,
  .Err_Integral_Out_Max = 100,
  .Err_Integral_Out_Min = -100
};

//PID_T Speed={0},Location={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
* @brief    LED控制函数：点亮、熄灭、电平翻转，
* @param    无
* @return   无
*/
void LED_ON(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_RESET);
}
void LED_OFF(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);
}
void LED_Turn(void) {
  if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET)
    LED_ON();
  else
    LED_OFF();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  Motor_Init();
  Encoder_Init();
  HAL_UART_Init(&huart1);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /** 定速电机测试 **/
  {
    // Speed.Flags = 0x00;
    // Speed.Out_Max = 100;
    // Speed.Out_Min = -100;
    // Speed.Err_Integral_Out_Max = 100;
    // Speed.Err_Integral_Out_Min = -100;
    // Location.Flags = 0x00;
    // Location.Out_Max = 100;
    // Location.Out_Min = -100;
    // Location.Err_Integral_Out_Max = 100;
    // Location.Err_Integral_Out_Min = -100;
  }
  char display_buf[128]={0};

  while (1)
  {
    /** 定速电机测试 **/
     {
    //   switch (Key_Getkey()) {
    //     case 1:Mode = 1;break;
    //     case 2:Mode = 2;break;
    //     case 3:Mode = 3;Location.Current = 0;Location.Err_Integral = 0;Speed.Err_Integral = 0;break;
    //     case 4:Mode = 4;break;
    //   }
    //   /* 速度控制 */
    //   if (Mode == 1) {
    //     Speed.Kp = RP_GetValue(1) / 4096.0 * 5.0;
    //     Speed.Ki = RP_GetValue(2) / 4096.0 * 1.0;
    //     Speed.Kd = RP_GetValue(3) / 4096.0 * 1.5;
    //     Speed.Target = RP_GetValue(4) * 90 / 4096.0  - 45;
    //     sprintf(display_buf,"kp:%4.3f",Speed.Kp);
    //     OLED_ShowString(0,0,display_buf,8);
    //     sprintf(display_buf,"ki:%4.3f",Speed.Ki);
    //     OLED_ShowString(0,16,display_buf,8);
    //     sprintf(display_buf,"kd:%4.3f",Speed.Kd);
    //     OLED_ShowString(0,32,display_buf,8);
    //     sprintf(display_buf,"t:%03.1f",Speed.Target);
    //     OLED_ShowString(64,0,display_buf,8);
    //     //实际速度-40~40
    //     sprintf(display_buf,"n:%03.1f",Speed.Current);
    //     OLED_ShowString(64,16,display_buf,8);
    //     sprintf(display_buf,"o:%03.1f ",Speed.Out);
    //     OLED_ShowString(64,32,display_buf,8);
    //     sprintf(display_buf,":%.1f,%.1f,%.1f\r\n",Speed.Target,Speed.Current,Speed.Out);
    //     //卧槽，绷不住了，这里Speed Control字符串多加两个空格会使pid控制紊乱
    //     OLED_ShowString(0,48,"Speed Control   ",8);
    //   }else if (Mode == 2) {
    //     /*位置控制*/
    //     Location.Kp = RP_GetValue(1) / 4096.0 * 5.0;
    //     Location.Ki = RP_GetValue(2) / 4096.0 * 1.0;
    //     Location.Kd = RP_GetValue(3) / 4096.0 * 1.5;
    //     Location.Target = RP_GetValue(4) * 426 / 4096.0  - 213;
    //     sprintf(display_buf,"kp:%4.3f",Location.Kp);
    //     OLED_ShowString(0,0,display_buf,8);
    //     sprintf(display_buf,"ki:%4.3f",Location.Ki);
    //     OLED_ShowString(0,16,display_buf,8);
    //     sprintf(display_buf,"kd:%4.3f",Location.Kd);
    //     OLED_ShowString(0,32,display_buf,8);
    //     sprintf(display_buf,"t:%03.1f",Location.Target);
    //     OLED_ShowString(64,0,display_buf,8);
    //     //实际速度-40~40
    //     sprintf(display_buf,"n:%03.1f",Location.Current);
    //     OLED_ShowString(64,16,display_buf,8);
    //     sprintf(display_buf,"o:%03.1f ",Location.Out);
    //     OLED_ShowString(64,32,display_buf,8);
    //     sprintf(display_buf,":%.1f,%.1f,%.1f\r\n",Location.Target,Location.Current,Location.Out);
    //     OLED_ShowString(0,48,"Pos Ctrl",8);
    //   }
    //   else if (Mode == 3) {
    //     /*双环位置控制*/
    //     Location.Target = RP_GetValue(4) * 430 / 4096.0  - 215;
    //     sprintf(display_buf,"kp:%4.3f",Speed.Kp);
    //     OLED_ShowString(0,0,display_buf,8);
    //     sprintf(display_buf,"ki:%4.3f",Speed.Ki);
    //     OLED_ShowString(0,16,display_buf,8);
    //     sprintf(display_buf,"kd:%4.3f",Speed.Kd);
    //     OLED_ShowString(0,32,display_buf,8);
    //     sprintf(display_buf,"kp:%4.3f",Location.Kp);
    //     OLED_ShowString(64,0,display_buf,8);
    //     sprintf(display_buf,"ki:%4.3f",Location.Ki);
    //     OLED_ShowString(64,16,display_buf,8);
    //     sprintf(display_buf,"kd:%4.3f",Location.Kd);
    //     OLED_ShowString(64,32,display_buf,8);
    //     sprintf(display_buf,":%.1f,%.1f,%.1f\r\n",Location.Target,Location.Current,Location.Out);
    //     OLED_ShowString(0,48,"2*Pos Ctrl",8);
    //   }
    //   OLED_UpdateScreen(0,8);
    //   HAL_UART_Transmit(&huart1,display_buf,strlen(display_buf),100);
     }
    /** 倒立摆实验 **/
     {
       switch (Key_Getkey()) {
         case 1:RunState = !RunState;AnglePID.Err_Integral = 0;LocationPID.Err_Integral=0;break;
         case 2: Mode = 1;break;
         case 3: Mode = 2;break;
         case 4: RunState = 7;break;
       }
       if (RunState&&HAL_GPIO_ReadPin(LED_GPIO_Port,LED_Pin))LED_Turn();
       else if (!RunState)LED_OFF();
       if (Mode == 1) {
          AnglePID.Kp = RP_GetValue(1) / 4096.0 * 2.0;
          AnglePID.Ki = RP_GetValue(2) / 4096.0 * 1.0;
          AnglePID.Kd = RP_GetValue(3) / 4096.0 * 1.5;
       }else if (Mode == 2) {
          LocationPID.Kp = -RP_GetValue(1) / 4096.0 * 4.0;
          LocationPID.Ki = RP_GetValue(2) / 4096.0 * 1.0;
          LocationPID.Kd = -RP_GetValue(3) / 4096.0 * 10;
       }
       LocationPID.Target = RP_GetValue(4) / 8;
       OLED_Print(0,0,6,"Angle");
       OLED_Print(0,12,6,"kp:%4.3f",AnglePID.Kp);
       OLED_Print(0,20,6,"ki:%4.3f",AnglePID.Ki);
       OLED_Print(0,28,6,"kd:%4.3f",AnglePID.Kd);
       OLED_Print(0,40,6,"tar:%04.0f",AnglePID.Target);
       OLED_Print(0,48,6,"cur:%04d",Angle);
       OLED_Print(0,56,6,"out:%03.1f ",AnglePID.Out);
       OLED_Print(64,0,6,"Location");
       OLED_Print(64,12,6,"kp:%4.3f",LocationPID.Kp);
       OLED_Print(64,20,6,"ki:%4.3f",LocationPID.Ki);
       OLED_Print(64,28,6,"kd:%4.3f",LocationPID.Kd);
       OLED_Print(64,40,6,"tar:%+04.0f",LocationPID.Target);
       OLED_Print(64,48,6,"cur:%+04d",Location);
       OLED_Print(64,56,6,"out:%03.1f ",LocationPID.Out);
       sprintf(display_buf,":%.1f,%d,%.1f\r\n",LocationPID.Target,Location,LocationPID.Out);
       OLED_UpdateScreen(0,8);
       HAL_UART_Transmit(&huart1,display_buf,sizeof(display_buf),100);
     }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 定时器更新中断回调函数
 * @param htim: 定时器句柄指针
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  /** 定速电机控制 **/
  {
    // static uint8_t count1=0,count2=0,count3=0;
    // if (htim->Instance == TIM1){
    //   count1 ++;
    //   Key_Tick();
    //   if (count1>=10){
    //     count1=0;
    //     if (Mode == 1){
    //       Speed.Current = ReadSpeed();
    //       PID_Update(&Speed);
    //       Load(Speed.Out);
    //     }
    //     else if (Mode == 2) {
    //       Location.Current += ReadSpeed();
    //       PID_Update(&Location);
    //       Load(Location.Out);
    //     }
    //   }
    //
    //   /**双环pid控制**/
    //   if (Mode == 3) {
    //     count2 ++;
    //     count3 ++;
    //     //内环——速度环
    //     if (count2 >=10) {
    //       count2=0;
    //       Speed.Current = ReadSpeed();
    //       Location.Current+=Speed.Current;
    //       PID_Update(&Speed);
    //       Load(Speed.Out);
    //     }
    //     //外环——位置环
    //     if (count3 >=15) {
    //       count3=0;
    //       PID_Update(&Location);
    //       Speed.Target = Location.Out;
    //       //外环输出限幅，可以控制转动速度大小快慢
    //       Speed.Target = Speed.Target>40?40:(Speed.Target<-40?-40:Speed.Target);
    //     }
    //
    //   }
    // }
  }
  /** 倒立摆实验 **/
  static uint8_t count_key=0,count_inner=0,count_outer=0,count_time=0,sample_time=START_SAMPLE_TIME;
  static uint16_t Angle_Sample[3]={0};
  if (htim->Instance == TIM1) {
    count_key++;
    if (count_key >= 10) {
      count_key = 0;
      Key_Tick();
    }
    Angle = Angle_GetValue();
    Location += ReadSpeed();
    if (RunState==0) {
      Location = 0;
      Load(0);
    }
    else if (RunState == 1) {
      sample_time--;
      if (!sample_time) {
        if (fabs(Angle - CENTER_ANGLE)<=CENTER_RANGE/2) {
          RunState = 2;
          Location = 0;
          return;
        }
        sample_time = START_SAMPLE_TIME;
        Angle_Sample[0] = Angle_Sample[1];
        Angle_Sample[1] = Angle_Sample[2];
        Angle_Sample[2] = Angle;
        if (Angle_Sample[1]<Angle_Sample[2]
          &&Angle_Sample[1]<Angle_Sample[0]
          &&Angle_Sample[0]>CENTER_ANGLE+CENTER_RANGE/2
          &&Angle_Sample[1]>CENTER_ANGLE+CENTER_RANGE/2
          &&Angle_Sample[2]>CENTER_ANGLE+CENTER_RANGE/2) RunState = 3;
        else if (Angle_Sample[1]>Angle_Sample[2]
          &&Angle_Sample[1]>Angle_Sample[0]
          &&Angle_Sample[0]<CENTER_ANGLE-CENTER_RANGE/2
          &&Angle_Sample[1]<CENTER_ANGLE-CENTER_RANGE/2
          &&Angle_Sample[2]<CENTER_ANGLE-CENTER_RANGE/2) RunState = 7;
        }
      }
    else if (RunState == 2) {
      if (!(fabs(Angle - CENTER_ANGLE)<=CENTER_RANGE*2)) {
        RunState = 0;
        return;
      }
      count_inner++;
      count_outer++;
      //内环调控周期5ms
      if (count_inner>=5) {
        count_inner = 0;
        AnglePID.Current = Angle;
        PID_Update(&AnglePID);
        Load(AnglePID.Out);
      }
      if (count_outer>=50) {
        count_outer = 0;
        LocationPID.Current = Location;
        PID_Update(&LocationPID);
        AnglePID.Target = CENTER_ANGLE + LocationPID.Out;
      }
    }
    else if (RunState == 3) {
      Load(START_LOAD);
      count_time=START_TIME;
      RunState=4;
    }
    else if (RunState == 4) {
      count_time--;
      if (!count_time) {RunState=5;}
    }
    else if (RunState == 5) {
      Load(-START_LOAD);
      count_time=START_TIME;
      RunState=6;
    }
    else if (RunState == 6) {
      count_time--;
      if (!count_time) {
        RunState=1;
        Load(0);
      }
    }
    else if (RunState == 7) {
      Load(-START_LOAD);
      count_time=START_TIME;
      RunState=8;
    }
    else if (RunState == 8) {
      count_time--;
      if (!count_time) {RunState=9;}
    }
    else if (RunState == 9) {
      Load(START_LOAD);
      count_time=START_TIME;
      RunState=10;
    }
    else if (RunState == 10) {
      count_time--;
      if (!count_time) {
        Load(0);
        RunState=1;
      }
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
