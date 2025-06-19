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


enum ELEVATOR_STATE{
  ELEVATOR_STATE_INIT_PROGRESS,
  ELEVATOR_STATE_INIT_COMPLETE,
  ELEVATOR_STATE_MOVING,
  ELEVATOR_STATE_STOP
};

enum DOOR_STATE{
  DOOR_STATE_CLOSED,
  DOOR_STATE_OPENED,
};

enum BUTTON_STATE{
  BUTTON_STATE_ON,
  BUTTON_STATE_OFF,
};

enum MOVE_STATE{
  MOVE_STATE_UP,
  MOVE_STATE_DOWN,
};

enum ELEVATOR_STATE current_elevator_state;
enum DOOR_STATE current_door_state;
enum BUTTON_STATE current_1f_state;
enum BUTTON_STATE current_2f_state;
enum BUTTON_STATE current_3f_state;
enum MOVE_STATE current_move_state;

void elevator_state_init_progress(){
  current_elevator_state = ELEVATOR_STATE_INIT_PROGRESS;
  current_door_state = DOOR_STATE_CLOSED;

  current_1f_state =BUTTON_STATE_OFF;
  current_2f_state =BUTTON_STATE_OFF;
  current_3f_state =BUTTON_STATE_OFF;

  current_move_state = MOVE_STATE_DOWN;
}


void elevator_state_init_complete(){
  current_elevator_state = ELEVATOR_STATE_INIT_COMPLETE;
  current_door_state = DOOR_STATE_CLOSED;

  current_1f_state =BUTTON_STATE_OFF;
  current_2f_state =BUTTON_STATE_OFF;
  current_3f_state =BUTTON_STATE_OFF;

  current_move_state = MOVE_STATE_DOWN;
}


void elevator_state_stop_transition(){
  current_elevator_state = ELEVATOR_STATE_STOP;
  current_door_state = DOOR_STATE_CLOSED;

  current_1f_state =BUTTON_STATE_OFF;
  current_2f_state =BUTTON_STATE_OFF;
  current_3f_state =BUTTON_STATE_OFF;

  current_move_state = MOVE_STATE_DOWN;
}


void check_init_elevator_state(uint16_t GPIO_Pin)
{
  // [Received input Event]
  // #1.Photo Event : 1F(GPIO_PIN_10), 2F(GPIO_PIN_3), 3F(GPIO_PIN_5)
  //    ==> ? 확인
  // #2.Button Event : 1F(GPIO_PIN_8), 2F(GPIO_PIN_6), 3F(GPIO_PIN_5)
  //    ==> ? Toggle On 상태 정보 기억
  // #3.Door open/close : Open (GPIO_PIN_12), Close (GPIO_PIN_11)

  // 상태 종료 이동  조건  ELEVATOR_STATE_STOP
  /* if(GPIO_Pin == GPIO_PIN_10) */
  bool is_elevator_stop_transition_allowed = (GPIO_Pin == GPIO_PIN_10);
  if(is_elevator_stop_transition_allowed)
  {
    elevator_state_stop_transition();
  }

}


 void check_moving_elevator_state(uint16_t GPIO_Pin){
   // [ELEVATOR_STATE_MOVING 현재 상태에 따른 이동]
   // #2. else if (current_elevator_state == ELEVATOR_STATE_MOVING)
   //    2.1 - [SKIP] Door open/close : Open (GPIO_PIN_12), Close (GPIO_PIN_11)
   //    2.2 - [UP] & Photo Event[1F, 2F, 3F]
   //          -> 2.2.1 (true)[1F] & Button [1F] On (GPIO_PIN_8)
   //             ==> LCD output("1F")
   //             ==> Door Open
   //             ==> current_elevator_state = ELEVATOR_STATE_STOP & Return //상태변경
   //          -> 2.2.1 (false) [1F] & Button [1F] Off (GPIO_PIN_8)
   //             ==> LCD output("1F")
   //    2.3 - [Down] & Photo Event[1F, 2F, 3F]
   //          -> 2.3.1 (true)[1F] & Button [1F] On (GPIO_PIN_8)
   //             ==> LCD output("1F")
   //             ==> Door Open
   //             ==> current_elevator_state = ELEVATOR_STATE_STOP & Return //상태변경
   //          -> 2.3.1 (false) [1F] & Button [1F] Off (GPIO_PIN_8)
   //             ==> LCD output("1F")
 }

 void check_stop_elevator_state(uint16_t GPIO_Pin){
   // [ELEVATOR_STATE_STOP 현재 상태에 따른 이동]
   // #3. else if (current_elevator_state == ELEVATOR_STATE_STOP)
   //    3.1 - Door close & UP & Photo Event[1F, 2F, 3F]
   //            ==>
   //    3.2 - Door close & up & Photo Event[1F, 2F, 3F]
   //            ==>
   //    3.3 - Door open & up & Photo Event[1F, 2F, 3F]
   //            ==>
   //    3.4 - Door close & Down & Photo Event[1F, 2F, 3F]
   //            ==>
   //
 }


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // [Received input Event]
  // #1.Photo Event : 1F(GPIO_PIN_10), 2F(GPIO_PIN_3), 3F(GPIO_PIN_5)
  //    ==> ? 확인
  // #2.Button Event : 1F(GPIO_PIN_8), 2F(GPIO_PIN_6), 3F(GPIO_PIN_5)
  //    ==> ? Toggle On 상태 정보 기억
  // #3.Door open/close : Open (GPIO_PIN_12), Close (GPIO_PIN_11)

  if(current_elevator_state == ELEVATOR_STATE_INIT_COMPLETE){
    // error case... 호출되면 뭔가 잘못됐음.
    check_init_elevator_state(GPIO_Pin);
  }else if (current_elevator_state == ELEVATOR_STATE_MOVING){
    check_moving_elevator_state(GPIO_Pin);
  }else if (current_elevator_state == ELEVATOR_STATE_STOP){
    check_stop_elevator_state(GPIO_Pin);
  }else{
    // error case... 호출되면 뭔가 잘못됐음.
    //
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

  // 초기 설정값 넣기
  elevator_state_init_progress();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (current_elevator_state == ELEVATOR_STATE_INIT_PROGRESS){
      //...

      // 작성
      //door_close();
      //current_door_state = DOOR_STATE_CLOSED;

      // 작성
      //elevator_move_down();
      //current_move_state = MOVE_STATE_DOWN;

      elevator_state_init_complete();

    }
//    else if(current_elevator_state == ELEVATOR_STATE_MOVING)
//    {
//      //door_close;
//    }

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
