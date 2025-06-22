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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "step.h"
#include "lcd.h"
#include "delay_us.h"
#include "fnd.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

enum ELEVATOR_STATE
{
  ELEVATOR_STATE_INIT,    // 엘리베이터 초기화 상태
  ELEVATOR_STATE_MOVING,  // 엘리베이터 이동 중 상태
  ELEVATOR_STATE_STOP     // 엘리베이터 정지 상태
};
enum ELEVATOR_STATE         current_elevator_state;


#define ELEVATOR_STATE_ACTION_START 0    // 상태 진입 시 최초 1회 실행되는 동작
#define ELEVATOR_STATE_ACTION_PROGRESS 1 // 상태 내에서 반복 실행되는 동작
uint16_t  current_elevator_state_action;

#define MOVE_STATE_UP     0
#define MOVE_STATE_DOWN   1
uint16_t current_move_state;

// #1.Photo Event : 1F(GPIO_PIN_10), 2F(GPIO_PIN_3), 3F(GPIO_PIN_5)
#define FLOOR_1F          GPIO_PIN_10
#define FLOOR_2F          GPIO_PIN_3
#define FLOOR_3F          GPIO_PIN_5
uint16_t current_floor;

void gpio_pin_uart_log(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_10){
    uint8_t logText[] = "GPIO_PIN_10\n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

  if(GPIO_Pin == GPIO_PIN_3){
    uint8_t logText[] = "GPIO_PIN_3\n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

  if(GPIO_Pin == GPIO_PIN_5){
    uint8_t logText[] = "GPIO_PIN_5\n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

  if(GPIO_Pin == GPIO_PIN_8){
    uint8_t logText[] = "GPIO_PIN_8\n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

  if(GPIO_Pin == GPIO_PIN_6){
    uint8_t logText[] = "GPIO_PIN_6 \n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

  if(GPIO_Pin == GPIO_PIN_9){
    uint8_t logText[] = "GPIO_PIN_9\n";
    HAL_UART_Transmit(&huart2, logText, sizeof(logText), 100);
  }

}

// 현재 설정값에 따라 엘리베이터 이동 함수
void moving_elevator()
{
  if (current_move_state == MOVE_STATE_DOWN)
  {
    rotateDegrees(50, DIR_CW);
  }
  else if(current_move_state == MOVE_STATE_UP){
    rotateDegrees(50, DIR_CCW);
  }
}

// [Received input Event]
// #1.Photo Event : 1F(GPIO_PIN_10), 2F(GPIO_PIN_3), 3F(GPIO_PIN_5)
//    ==> ? 확인
// #2.Button Event : 1F(GPIO_PIN_8), 2F(GPIO_PIN_6), 3F(GPIO_PIN_9)
//    ==> ? Toggle On 상태 정보 기억
// #3.Door open/close : Open (GPIO_PIN_12), Close (GPIO_PIN_11)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // TODO : 모든 이밴트를 수신하는 인터럽트 함수 GPIO_Pin 값으로 수신 정보 확인 가능. 
  // gpio pin log : UART 로 Pin 수신정보를 디버깅하기 위해 추가.
  gpio_pin_uart_log(GPIO_Pin);

  if(current_elevator_state == ELEVATOR_STATE_INIT)
  {
    // NOTICE: ELEVATOR_STATE_INIT 상태에서 Photo Pin 정보를 체크    
    // Photo Pin 정보 수신 시 ELEVATOR 정지시킴 (ELEVATOR_STATE_STOP)
    bool is_elevator_stop_transition_allowed = (GPIO_Pin == GPIO_PIN_10) | (GPIO_Pin == GPIO_PIN_3) | (GPIO_Pin == GPIO_PIN_5);
    if(is_elevator_stop_transition_allowed)
    { 
      current_elevator_state = ELEVATOR_STATE_STOP;
      current_elevator_state_action = ELEVATOR_STATE_ACTION_START;
      current_floor = GPIO_Pin;
    }
  }
  else if (current_elevator_state == ELEVATOR_STATE_STOP)
  {
    // 버튼 이밴트 [ELEVATOR_STATE_STOP 현재 상태에 따른 이동]
    
    // #1. door open - close 명령 처리..

    // #2. floor 버튼 toogle 정보 저장
    // change_toogle_botton(GPIO_Pin); // 미구현

    // #3. 현제 위치 와 버튼상태를 확인해 진행 방향을 검토해야 함.
    bool isButtonFin = (GPIO_Pin == GPIO_PIN_8) ||  (GPIO_Pin == GPIO_PIN_6) || (GPIO_Pin == GPIO_PIN_5);
    if (isButtonFin){
      current_elevator_state = ELEVATOR_STATE_MOVING;
      current_elevator_state_action = ELEVATOR_STATE_ACTION_START;
    }
  }
  else if (current_elevator_state == ELEVATOR_STATE_MOVING)
  {
    // #2. floor 버튼 toogle 정보 저장
    // change_toogle_botton(GPIO_Pin); // 미구현


    // [TODO]: 종료 조건에 버튼상태까지 확인하는 코드 추가 필요함..... 
    // [ELEVATOR_STATE_MOVING 현재 상태에 따른 이동]
    bool is_elevator_stop_transition_allowed = (GPIO_Pin == GPIO_PIN_10) | (GPIO_Pin == GPIO_PIN_3) | (GPIO_Pin == GPIO_PIN_5);
    if(is_elevator_stop_transition_allowed) 
    {
      current_elevator_state = ELEVATOR_STATE_STOP;
      current_elevator_state_action = ELEVATOR_STATE_ACTION_START;
      current_floor = GPIO_Pin;
    }
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim11);

  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  // TIM3->CCR3 = 500;
  //  HAL_UART_Receive_IT(&huart2, &rxData, sizeof(rxData));
  //  HAL_UART_Transmit_IT(&huart2, &rxData, sizeof(rxData));
  
    
  { /* 초기 설정값 넣기 */ 
    // 현재 동작 시작상태
    current_elevator_state = ELEVATOR_STATE_INIT;
    current_elevator_state_action = ELEVATOR_STATE_ACTION_START;

    // 앨리베이터 위치
    current_floor = FLOOR_1F;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // check 루틴 순서..
    // 1. current_elevator_state -->
    // NOTICE: [ELEVATOR_STATE_INIT -->  ELEVATOR_STATE_STOP <--> ELEVATOR_STATE_MOVING] 
    // 상태가 변경 되면 이 상태안에서
    //   1. ELEVATOR_STATE_ACTION_START(1회)
    //   2. ELEVATOR_STATE_ACTION_PROGRESS (N회)
    // 순으로 loop 실행.

    // NOTICE : current_elevator_state 상태는 HAL_GPIO_EXTI_Callback 함수에서만 변경하세요.

    if (current_elevator_state == ELEVATOR_STATE_INIT)
    {
      switch (current_elevator_state_action) 
      {
        case ELEVATOR_STATE_ACTION_START:
          // NOTICE: INIT 상태에서 ONE Time 실행 되어야 되는 경우 여기서 처리 
          // (예) LCD 출력 등 //
          // LCD 출력 문구 ....

          // 엘리베이터 방향 지정
          current_move_state = MOVE_STATE_DOWN;

          // ELEVATOR_STATE_ACTION_PROGRESS 상태로 이동
          current_elevator_state_action = ELEVATOR_STATE_ACTION_PROGRESS;
          break;
          
        case ELEVATOR_STATE_ACTION_PROGRESS:
          // 엘리베이터를 이동 함수호출
          moving_elevator();
          break;
      }
    }
    else if(current_elevator_state == ELEVATOR_STATE_STOP)
    {
      switch (current_elevator_state_action)
      {
        case ELEVATOR_STATE_ACTION_START:
          // NOTICE: INIT 상태에서 ONE Time 실행 되어야 되는 경우 여기서 처리 (예) LCD 출력 등 // 
          // (예) 도어 오픈 - 클로즈 명령 //
          
          // 엘리베이터 상하 이동을 위해 상태변경 
          //current_elevator_state_action = ELEVATOR_STATE_ACTION_PROGRESS;
          break;
        
        case ELEVATOR_STATE_ACTION_PROGRESS:
          // door open closed 서보 모터 동작 처리 
          break;
      }
    }
    else if(current_elevator_state == ELEVATOR_STATE_MOVING)
    {
      switch (current_elevator_state_action)
      {
        case ELEVATOR_STATE_ACTION_START:
          // 상태 변경 진입 시 한번만 호출

          // for test
          // !!!! test 방향 바꿈 코드 삭제 필요
          if (current_move_state == MOVE_STATE_UP) current_move_state = MOVE_STATE_DOWN;
          else current_move_state = MOVE_STATE_UP;

          current_elevator_state_action = ELEVATOR_STATE_ACTION_PROGRESS;
          break;
        
        case ELEVATOR_STATE_ACTION_PROGRESS:
          // 엘리베이터를 이동 함수호출
          moving_elevator();
          break;
      }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
