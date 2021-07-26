/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265f
#define kf 10.06119447f   //RPS = kf*force(in gm)^0.5

#define PID_TAU 0.000056f

#define PID_LIM_MIN -50.0f
#define PID_LIM_MAX  50.0f

#define PID_LIM_MIN_INT -2.0f
#define PID_LIM_MAX_INT  2.0f

#define SAMPLE_TIME_S 0.004f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for PID_RTOS */
osThreadId_t PID_RTOSHandle;
const osThreadAttr_t PID_RTOS_attributes = {
  .name = "PID_RTOS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for Heartbeat_RTOS */
osThreadId_t Heartbeat_RTOSHandle;
const osThreadAttr_t Heartbeat_RTOS_attributes = {
  .name = "Heartbeat_RTOS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU_filter */
osThreadId_t IMU_filterHandle;
const osThreadAttr_t IMU_filter_attributes = {
  .name = "IMU_filter",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myTimer02 */
osTimerId_t myTimer02Handle;
const osTimerAttr_t myTimer02_attributes = {
  .name = "myTimer02"
};
/* USER CODE BEGIN PV */
#define NO_OF_RECIVED_BYTES 68
struct ser{

   float head;

   float pitch;
   float roll;
   float yaw;

    float gy;
    float gx;
    float gz;

    float ay;
    float ax;
    float az;

    float throttle_y;
    float angle_y;
    float throttle_x;
    float angle_x;

    float gain;
    float select;
    float heartbeat;
  }ser;
  union{
    uint8_t bytes[NO_OF_RECIVED_BYTES];
    struct ser rx;

  }stm;



  union{
    uint8_t bytes[4];
    float h;

  }head;




  struct params{
	  float error[2];
	  float errord;
	  float errori;
	  float kp;
	  float kd;
	  float ki;
  };
  struct params phi,theta,psi;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void PID_R(void *argument);
void Heartbeat_R(void *argument);
void IMU_filter_RT(void *argument);
void Timer1Callback(void *argument);
void Timer2Callback(void *argument);

/* USER CODE BEGIN PFP */
//void heartbeat(float a);
void control_law(float throttle,float angle_desz,float angle_desx,float angle_desy,float phi_pid,float theta_pid,float psi_pid);
void gain_select(void);
uint32_t counter;

float thrust,phi_des,theta_des,psi_des;
float u1,u2,u3,u4;
float ws1,ws2,ws3,ws4;
float w1,w2,w3,w4;
float p1=1000,p2=1000,p3=1000,p4=1000;

float gw_phi,gw_theta,gw_psi;
float theta_acc,phi_acc,theta_comp,phi_comp,psi_comp;
float gx_f,gy_f,gz_f;
float ax_f,ay_f,az_f;
float theta_f,phi_f,psi_f;
float dt,dt2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int i,k,p,T;
uint8_t f=0,esp_flag;
float kt,theta_p,theta_c,phi_c,prev_heartbeat;
float micros;

///kalman variables
//float KG = 10.0,Eest = 1.0,est = 10.0,Em = 2.0;
//float p11,p12,p21,p22;
//float q1 = 40.0,q2 = 8.0;
//float x1,x2,y;
uint32_t cnt,cnt2;
uint8_t kalman;
float R  = 10.0;
float pid_theta_out,pid_phi_out,pid_psi_out;

PIDController pid_theta = { 0, 0.0, 0,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };
PIDController pid_phi = { 0, 0.0, 0,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };
PIDController pid_psi = { 0, 0.0, 0,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
head.h		= 1203.1203;

phi.kp		= 0.0012;
theta.kp	= 0.0012;
psi.kp		= 0.0;

phi.kd		= 0.003599;
theta.kd	= 0.003599;
psi.kd		= 0.0;

phi.ki		= 0.00;
theta.ki	= 0.00;
psi.ki		= 0.0;

kt= 0.3;


PIDController_Init(&pid_theta);
PIDController_Init(&pid_phi);
PIDController_Init(&pid_psi);
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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart6,stm.bytes,1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim2);
 // HAL_UART_Receive_DMA (&huart6, stm.bytes, 44);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Timer1Callback, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* creation of myTimer02 */
  myTimer02Handle = osTimerNew(Timer2Callback, osTimerPeriodic, NULL, &myTimer02_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PID_RTOS */
  PID_RTOSHandle = osThreadNew(PID_R, NULL, &PID_RTOS_attributes);

  /* creation of Heartbeat_RTOS */
  Heartbeat_RTOSHandle = osThreadNew(Heartbeat_R, NULL, &Heartbeat_RTOS_attributes);

  /* creation of IMU_filter */
  IMU_filterHandle = osThreadNew(IMU_filter_RT, NULL, &IMU_filter_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if(kalman == 1)
//	  {
//		kalman = 0;
//
//		y = stm.rx.pitch - x1;
//		x1 = x1 + y*p11/(p11 + R);
//		x2 = gx_f*180.0f/PI + y*p21/(p11+R);
//
//		p22 = p22 - p21*p12/(p11 + R);
//		p12 = p12*p11/(p11 + R);
//		p21 = p21*p11/(p11 + R);
//		p11 = R*p11/(p11 + R);
//
//		p11 = p11*dt + dt*(p12 + p11 + dt*p22) + q1;
//		p21 = p21 + dt*p22;
//		p12 = p12 + dt*p22;
//		p22 = p22 + q2;
//
//		x1 = x1 + dt*x2;
//	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 216-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 215;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void gain_select(void){
	  switch( (int)stm.rx.select){

	  		  	  case 97 :
	  			   	   	   	   	  phi.kp  = stm.rx.gain/400.0f;
	  			   	   	   	   	  break;
	  		  	  case 98 :
	  		  		  	  	  	  phi.kd  = stm.rx.gain/6000.0f;
	  		  		  	  	  	  break;
	  		  	  case 99 :
	  		  			  		  phi.ki  = stm.rx.gain/5000.0f;
	  		  			  		  break;
	  		  	  case 100 :
	  		  			  		  theta.kp  = stm.rx.gain/400.0f;
	  		  			  		  break;
	  		  	  case 101 :
	  		  			  		  theta.kd  = stm.rx.gain/6000.0f;
	  		  			  		  break;
	  		  	  case 102 :
	  		  			  		  theta.ki  = stm.rx.gain/5000.0f;
	  		  			  		  break;
	  		  	  case 103 :
	  		  			  		  psi.kp  = stm.rx.gain/400.0f;
	  		  			  		  break;
	  		  	  case 104 :
	  		  			  		  psi.kd  = stm.rx.gain/2500.0f;
	  		  			  		  break;
	  		  	  case 105 :
	  		  			  		  psi.ki  = stm.rx.gain/10000.0f;
	  		  			  		  break;
	  		  	  case 106 :
	  		  			  		  kt  = stm.rx.gain/100.0f;
	  		  			  		  break;
	  		  	  }
	  pid_theta.Kp	=	phi.kp;
	  pid_theta.Kd	=	phi.kd*1.5;
	  pid_theta.Ki	=	phi.ki;

	  pid_phi.Kp	=	phi.kp;
	  pid_phi.Kd	=	phi.kd*1.5;
	  pid_phi.Ki	=	phi.ki;

	  pid_psi.Kp	=	psi.kp;
	  pid_psi.Kd	=	psi.kd*1.5;
	  pid_psi.Ki	=	psi.ki;

}

void control_law(float throttle,float angle_desz,float angle_desx,float angle_desy,float phi_pid,float theta_pid,float psi_pid)
{
	        phi_des		= 0.30*(angle_desx*sin((psi_f-psi_des)*PI/180.00)-angle_desy*cos((psi_f-psi_des)*PI/180.00));
	        theta_des	= 0.30*(angle_desx*cos((psi_f-psi_des)*PI/180.00)+angle_desy*sin((psi_f-psi_des)*PI/180.00));


//	        phi.error[1]	= phi_des - b;
//	        theta.error[1] 	= theta_des - a;
//	        psi.error[1]	= stm.rx.throttle_x - c;
//
//	        theta_p = theta.kp*theta.error[1];
//
//	        phi.errord		= -phi.kd*d*180/PI;
//	        theta.errord	= -theta.kd*e*180/PI ;//theta.kd*(theta.error[1] - theta.error[0])/dt;
//	        psi.errord		= -psi.kd*f*180/PI;
//
//	        phi.error[0]	= phi.error[1];
//	        theta.error[0]	= theta.error[1];
//	        psi.error[0]	= psi.error[1];
//
//	        phi.errori		+= phi.error[1]*dt;
//	        theta.errori	+= theta.error[1]*dt;
//	        psi.errori		+= psi.error[1]*dt;
//
//	        if (theta.errori>20.0)
//	        	theta.errori=20.0;
//	        if (theta.errori<-20.0)
//	        	theta.errori=-20.0;
//
//	        u2		=	2.778*(phi.kp*(phi.error[1]) + phi.errord + phi.ki*phi.errori);
//	        u3		=	2.778*(theta.kp*(theta.error[1]) + theta.errord + theta.ki*theta.errori);
//
//	       u4		=	4.2589*(psi.kp*(psi.error[1]) + psi.errord + psi.ki*psi.errori);
	        u1		=	kt*throttle;
	        u2		=	2.7778*phi_pid;
	        u3 		=	2.7778*theta_pid;
	        u4		=	4.2589*psi_pid;

	        ws1 =	u1 - u3 + u4;
	        ws2 = 	u1 + u2 - u4;
	        ws3 =	u1 + u3 + u4;
	        ws4 = 	u1 - u2 - u4;

	        if(ws1<0)
	        	ws1 = 0;
	        if(ws2<0)
	           	ws2 = 0;
	        if(ws3<0)
 	        	ws3 = 0;
	        if(ws4<0)
 	        	ws4 = 0;

	         w1 = kf*sqrt(ws1);
	         w2 = kf*sqrt(ws2);
	         w3 = kf*sqrt(ws3);
	         w4 = kf*sqrt(ws4);


	         p1 = 0.0009698*w1*w1*w1-0.1218*w1*w1 + 12.18*w1+874.7;  //WCCW
	         p2 = 0.0007340*w2*w2*w2-0.0896*w2*w2 + 11.69*w2+837.2; //WCW
	         p3 = 0.0008200*w3*w3*w3-0.1073*w3*w3 + 12.08*w3+836.7; //BCCW
	        // p4 = 0.0006939*w4*w4*w4-0.09048*w4*w4 + 11.77*w4+860.0;//BCW
	         p4 = 0.0007340*w4*w4*w4-0.0896*w4*w4 + 11.69*w4+837.2; //WCW

	  	               //u1=Ft-F,u2 = Ft+F;
	         if(p1<1000)
	            p1= 1000;

	         if(p2<1000)
	            p2= 1000;

	         if(p1>2000)
	            p1= 2000;

	         if(p2>2000)
	            p2= 2000;

	         if(p3<1000)
	        	 p3= 1000;

	         if(p4<1000)
	             p4= 1000;

	         if(p3>2000)
	             p3= 2000;

	         if(p4>2000)
	 	         p4= 2000;
	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p3); //BCCW
	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p4); //BCW
	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p2); //WCW
	         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p1); //WCCW
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);



	if(esp_flag == 0)
	{
		if(stm.bytes[0]==head.bytes[0])
	  {
		 HAL_UART_Receive_DMA(&huart6,&stm.bytes[1],NO_OF_RECIVED_BYTES-1);
		 esp_flag = 1;
	  }

	 HAL_UART_Receive_IT(&huart6,stm.bytes,1);
	}
	if(esp_flag == 1)
	{

		HAL_UART_Receive_DMA(&huart6,stm.bytes,NO_OF_RECIVED_BYTES);
		kalman = 1;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_PID_R */
/**
  * @brief  Function implementing the PID_RTOS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_PID_R */
void PID_R(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {	gain_select();

	  if(f==1 )
	  {
		  dt2	= (float)(TIM2->CNT - cnt2)/500000.00;
		  cnt2	= TIM2->CNT;

		  if(esp_flag==0)
			  f=0;

		  PIDController_Update(&pid_theta, theta_des, theta_comp -theta_c);
		  PIDController_Update(&pid_phi, phi_des, phi_comp - phi_c);
		  PIDController_Update(&pid_psi, -psi_des, -psi_f);
		  pid_phi_out	=	pid_phi.out;
		  pid_theta_out =	pid_theta.out;
		  pid_psi_out	 =	pid_psi.out;
		  control_law(stm.rx.throttle_y,stm.rx.throttle_x,stm.rx.angle_x,stm.rx.angle_y,pid_phi.out,pid_theta.out,pid_psi.out);

	  }
	  if(esp_flag ==0)
	  {

		  HAL_UART_Receive_IT(&huart6,stm.bytes,1);

		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);

	  }
    osDelay(4);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Heartbeat_R */
/**
* @brief Function implementing the Heartbeat_RTOS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Heartbeat_R */
void Heartbeat_R(void *argument)
{
  /* USER CODE BEGIN Heartbeat_R */
  /* Infinite loop */
  for(;;)
  {
	  T++;

	  if((stm.rx.heartbeat - prev_heartbeat)== 0)
	  		  {
	  			  esp_flag =0;
	  			  stm.bytes[0] = 0;
	  		  }
	  		  prev_heartbeat = stm.rx.heartbeat;
    osDelay(500);
  }
  /* USER CODE END Heartbeat_R */
}

/* USER CODE BEGIN Header_IMU_filter_RT */
/**
* @brief Function implementing the IMU_filter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_filter_RT */
void IMU_filter_RT(void *argument)
{
  /* USER CODE BEGIN IMU_filter_RT */
  /* Infinite loop */
  for(;;)
  {
	  if(f==0)
	  {		psi_des = stm.rx.yaw;
		  theta_c = theta_f;
		  phi_c = phi_f;
	  }
	  if((stm.rx.throttle_y <10.00)&&(esp_flag==1)&&(stm.rx.throttle_x<-80))
		  f =1;

//	  ax_f = 0.9995 * ax_f + stm.rx.ax* 0.0005;
//	  ay_f = 0.9995 * ay_f + stm.rx.ay* 0.0005;
//	  az_f = 0.9995 * az_f + stm.rx.az* 0.0005;

//	  phi_acc	=	-atan2(ax_f,sqrt(ay_f*ay_f + az_f*az_f ))*180/PI;
//	  theta_acc	=	 atan2(ay_f,sqrt(ax_f*ax_f + az_f*az_f ))*180/PI;
	  dt	= (float)(TIM2->CNT - cnt)/500000.00;

	  cnt	= TIM2->CNT;

	  gx_f = 0.9996 * gx_f + stm.rx.gx* 0.0004;
	  gy_f = 0.9996 * gy_f + stm.rx.gy* 0.0004;
	  gz_f = 0.9996 * gz_f + stm.rx.gz* 0.0004;

	  theta_f 	= (0.9996 * theta_f	- stm.rx.pitch 	*0.0004);
	  phi_f 	= (0.9996 * phi_f 	- stm.rx.roll	*0.0004);
	  psi_f 	= (0.9996 * psi_f	+ stm.rx.yaw	*0.0004);

	  theta_comp 	= 0.999991*(theta_comp	+ gy_f*dt*180.0f/PI)   + 0.000009*theta_f ;
	  phi_comp		= 0.999991*(phi_comp 	+ gx_f*dt*180.0f/PI)   + 0.000009*phi_f ;
	  psi_comp		= 0.999991*(psi_comp 	- gz_f*dt*180.0f/PI)   + 0.000009*psi_f ;

	  gw_phi 	= 	gx_f + ( gy_f * sin(phi_comp*PI/180.0f) -  gz_f* cos(phi_comp*PI/180.0f) )*tan(theta_comp*PI/180.0f);
	  gw_theta 	=			 gy_f * cos(phi_comp*PI/180.0f) +  gz_f* sin(phi_comp*PI/180.0f);
	  gw_psi  	= 			 gy_f *(sin(phi_comp*PI/180.0f) -  gz_f* cos(phi_comp*PI/180.0f) )/cos(theta_comp*PI/180.0f);

    osDelay(0.5);
  }
  /* USER CODE END IMU_filter_RT */
}

/* Timer1Callback function */
void Timer1Callback(void *argument)
{
  /* USER CODE BEGIN Timer1Callback */

  /* USER CODE END Timer1Callback */
}

/* Timer2Callback function */
void Timer2Callback(void *argument)
{
  /* USER CODE BEGIN Timer2Callback */

  /* USER CODE END Timer2Callback */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
