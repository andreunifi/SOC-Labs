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
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "cs43l22.h"
#include "math.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_SIZE 256

// macros to define the sine signal
#define FAST_SIN_FREQ 1000
#define SLOW_SIN_FREQ 500

#define SAMPLING_RATE 48000
#define AUDIO_BUFFER_LENGTH SAMPLING_RATE / SLOW_SIN_FREQ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t ISR_FLAG_RX    = 0x01;  // Received data
const uint8_t ISR_FLAG_TIM10 = 0x02;  // Timer 10 Period elapsed
const uint8_t ISR_FLAG_TIM11 = 0x04;  // Timer 10 Period elapsed
const uint8_t ISR_BUTTON_PRESS = 0x08;  // User button pressed


// This is the only variable that we will modify both in the ISR and in the main loop
// It has to be declared volatile to prevent the compiler from optimizing it out.
volatile uint8_t isr_flags = 0;

// Commands that we will receive from the PC
int status = 0;
int manual = 0;
const int frequ[3] = {10000,200000,300000}; // Possible periods for the PWM (in timer ticks)
int duty_cycle= 50;
int period = 2000000; //in timer ticks
const int duty[3] = {25,50,75}; // Possible duty cycles for the PWM
unsigned long int ticks_elapsed = 0;
char is_recording = 0;
// Command Description
// stop Put Board in stop mode
// standby Put Board in standby mode
// changefreq Set the PWM frequency to the next possible value
// changedut Set the PWM duty cycle to the next possible value
// pwmman Set PWM frequency with user button
// ledpwm Set the LED BLINK to PWM mode
// ledman Set the LED BLINK to manual mode
// ledon Turn ON the LED BLINK
// ledoff Turn OFF the LED BLINK
// accon Enable accelerometer readings
// accoff Disable accelerometer readings
// mute Mute audible signal
// unmute Un-mute audible signa

const uint8_t COMMAND_STOP[]        = "stop"; // Go to stop mode
const uint8_t COMMAND_STANDBY[]     = "standby"; // Go to standby mode
const uint8_t COMMAND_CHANGE_FREQ[] = "changefreq"; // Change the PWM frequency
const uint8_t COMMAND_CHANGE_DUTY[] = "changedut"; // Change the PWM duty cycle
const uint8_t COMMAND_PWM_MAN[]    = "pwmman"; // Set PWM frequency with user button
const uint8_t COMMAND_LED_PWM[]    = "ledpwm"; // Set the LED BLINK to PWM mode
const uint8_t COMMAND_LED_MAN[]    = "ledman"; // Set the LED BLINK to manual mode
const uint8_t COMMAND_LED_ON[]     = "ledon"; // Turn ON the LED BLINK
const uint8_t COMMAND_LED_OFF[]    = "ledoff"; // Turn OFF the LED BLINK
const uint8_t COMMAND_ACC_ON[]     = "accon"; // Enable accelerometer readings
const uint8_t COMMAND_ACC_OFF[]    = "accoff"; // Disable accelerometer readings
const uint8_t COMMAND_MUTE[]       = "mute"; // Mute audible signal
const uint8_t COMMAND_UNMUTE[]     = "unmute"; // Un-mute audible signal




// Buffer for command
static uint8_t line_ready_buffer[LINE_BUFFER_SIZE]; // Stable buffer for main

// Audio buffer
int16_t buffer_audio[2 * AUDIO_BUFFER_LENGTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Interrupt handlers
void handle_new_line();


void handle_timer10(void);

void handle_timer11(void);

void manual_sample(void);

// Commands
void go_to_stop();

// Helper functions
void init_codec_and_play();


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim ){
if ( htim -> Instance == htim10.Instance ) {
isr_flags |= ISR_FLAG_TIM10 ;
  }
  else if ( htim -> Instance == htim11.Instance ) {
isr_flags |= ISR_FLAG_TIM11 ;
  }
};
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  BSP_LED_Init(LED5); // Initialize LED5 for indication
  BSP_LED_Init(LED4); // Initialize LED4 for indication
  BSP_LED_Init(LED3); // Initialize LED3 for indication
  BSP_LED_Init(LED6); // Initialize LED6 for indication
  // LED4 = 0,
  // LED3 = 1,
  // LED5 = 2,
  // LED6 = 3
   // Indicate that the system is starting
  /* USER CODE BEGIN 2 */
  CDC_Transmit_FS((uint8_t*)"Unknown command\r\n", 17);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // We check if that there are NO interrupts pending before going into sleep
    if (isr_flags == 0)
    {
      // Go to sleep, waiting for interrupt (WFI).
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
    }

    // This is needed for the UART transmission 
    if (isr_flags & ISR_FLAG_RX)
    {
      isr_flags &= ~ISR_FLAG_RX;
      handle_new_line();
    }

    if (isr_flags & ISR_FLAG_TIM10)
    {
      isr_flags &= ~ ISR_FLAG_TIM10 ;
      handle_timer10();
    }

    if (isr_flags & ISR_FLAG_TIM11)
    {
      isr_flags &= ~ ISR_FLAG_TIM11 ;
      handle_timer11();
    }

    if( isr_flags & ISR_BUTTON_PRESS)
    {
      isr_flags &= ~ ISR_BUTTON_PRESS ;
      //CDC_Transmit_FS((uint8_t *)"Button Pressed\r\n",16);
      if(manual)
        manual_sample();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// All of this is used to manage commands from serial interface
static uint8_t line_buffer[LINE_BUFFER_SIZE];
static uint32_t line_len = 0;
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
  // Prevent overflow, does not handle the command
  if (line_len + len >= LINE_BUFFER_SIZE)
  {
    line_len = 0;
    return;
  }

  // Append received chunk
  memcpy(&line_buffer[line_len], buf, len);
  line_len += len;

  // Process all complete lines
  while (1)
  {
    // Look for '\n' or '\r' inside line_buffer
    uint8_t *line_feed = memchr(line_buffer, '\n', line_len);
    if (line_feed == NULL){
      line_feed = memchr(line_buffer, '\r', line_len);
      if (line_feed == NULL)
      break;
    }

    // Replace \n or \r by terminator
    *line_feed = '\0';

    // Remove optional '\r' in case input was \r\n
    if (line_feed > line_buffer && *(line_feed - 1) == '\r')
    *(line_feed - 1) = '\0';

    uint32_t processed = (line_feed + 1) - line_buffer;

    // Signal the main that there is a new line ready to be checked
    memcpy(line_ready_buffer, line_buffer, processed);
    isr_flags |= ISR_FLAG_RX;

    // Move leftover bytes to start
    line_len -= processed;
    memmove(line_buffer, line_buffer + processed, line_len);
  }
}


void manual_sample(void) {
  //HAL_TIM_Base_Stop_IT(&htim10);
  static uint32_t last_button_tick = 0;
  const uint32_t debounce_delay = 150; // 50 ms debounce time

  uint32_t current_tick = HAL_GetTick();
  if (current_tick - last_button_tick < debounce_delay) {
      // Ignore button press if within debounce period
      return;
  }
  last_button_tick = current_tick;

  if (manual) {
      if (is_recording == 0) {
          // Start recording
          ticks_elapsed = current_tick;
          is_recording = 1;
          BSP_LED_On(LED5);
      } else {
          // Stop recording
          ticks_elapsed = current_tick - ticks_elapsed;
          is_recording = 0;
          BSP_LED_Off(LED5);
          // Send result to PC
          char msg[50];
          int msg_len = snprintf(msg, sizeof(msg), "Elapsed time: %lu ms\r\n", ticks_elapsed);
          CDC_Transmit_FS((uint8_t*)msg, msg_len);
          //period = ticks_elapsed;
      }
  } else {
      return;
  }
}

void init_PWM(int period, float duty_cycle)
{
  // Stop TIM10
  HAL_TIM_Base_Stop_IT(&htim10);

  // Configure prescaler and period for desired frequency
  uint32_t prescaler = 167; // Assuming APB1 timer clock is 84 MHz
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = prescaler;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;

  int duty_cycle_period= (duty_cycle * period) / 100;

  htim10.Init.Period = duty_cycle_period;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }

  // Reset counter
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  //Start high phase
  status = 1;
  BSP_LED_On(LED5);
  // Start TIM10 with interrupt
  if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}

void change_duty()
{
  static int current_duty_index = 0;
  current_duty_index = (current_duty_index + 1) % 3; // Cycle through 0, 1, 2
  duty_cycle= duty[current_duty_index];
  init_PWM(period, duty_cycle);

  // Send feedback to PC
  char msg[50];
  int msg_len = snprintf(msg, sizeof(msg), "PWM duty cycle changed to %d %%\r\n", duty_cycle);
  CDC_Transmit_FS((uint8_t*)msg, msg_len);
};

void change_freq()
{
  static int current_freq_index = 0;
  current_freq_index = (current_freq_index + 1) % 3; // Cycle through 0, 1, 2
  period= frequ[current_freq_index];
  init_PWM(period, duty_cycle);

  // Send feedback to PC
  char msg[50];
  int msg_len = snprintf(msg, sizeof(msg), "PWM frequency changed to %d ticks\r\n", frequ[current_freq_index]);
  CDC_Transmit_FS((uint8_t*)msg, msg_len);
}


void handle_timer10(void)
{
  // Clear the flag
  isr_flags &= ~ISR_FLAG_TIM10;
  //CDC_Transmit_FS((uint8_t*)"Ping\r\n", 17);


  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  //HAL_GPIO_TogglePin ( LED5_GPIO_PORT, LED5_PIN ) ; //This was adeded to toggle the LED on timer interrupt


  if (manual)
  {
      return;
  } else {

    if (status == 0)
  {
      uint32_t high_ticks = ((duty_cycle) * period) / 100;
      __HAL_TIM_SET_AUTORELOAD(&htim10, high_ticks);
      status = 1;
      BSP_LED_On(LED5);
  }
  else
  {
      uint32_t low_ticks = ((100 - duty_cycle) * period) / 100;
      __HAL_TIM_SET_AUTORELOAD(&htim10, low_ticks);
      status = 0;
      BSP_LED_Off(LED5);
  }

  __HAL_TIM_SET_COUNTER (& htim10 ,0) ;

  }


}

void handle_timer11(void)
{
  return;
}

/**
* @brief  Handle possible new command
* @retval None
*/
void handle_new_line()
{
  if (memcmp(line_ready_buffer, COMMAND_CHANGE_FREQ, sizeof(COMMAND_CHANGE_FREQ)) == 0)
  {
    change_freq();
  }

  else if (memcmp(line_ready_buffer, COMMAND_STOP, sizeof(COMMAND_STOP)) == 0)
  {
    go_to_stop();
  }

  else if (memcmp(line_ready_buffer, COMMAND_PWM_MAN, sizeof(COMMAND_PWM_MAN)) == 0)
{
  manual = 1;
  status = 0;
  HAL_TIM_Base_Stop_IT(&htim10);

  

} else if (memcmp(line_ready_buffer, COMMAND_LED_PWM, sizeof(COMMAND_LED_PWM)) == 0)
  {
    init_PWM(period, duty_cycle);
    
  } else if (memcmp(line_ready_buffer, COMMAND_LED_MAN, sizeof(COMMAND_LED_MAN)) == 0)
  {
    
    manual = 0;
    period = ticks_elapsed * 1000; // Convert ms to timer ticks 
    init_PWM(period, 50);
    char msg[100];
    int msg_len = snprintf(msg, sizeof(msg), "Setting LED to manual mode with last recorded time: %lu ticks\r\n", ticks_elapsed*1000);
    CDC_Transmit_FS((uint8_t*)msg, msg_len);

    
  } else if (memcmp(line_ready_buffer, COMMAND_LED_ON, sizeof(COMMAND_LED_ON)) == 0)
  {
    
  } else if (memcmp(line_ready_buffer, COMMAND_LED_OFF, sizeof(COMMAND_LED_OFF)) == 0)
  {
    
  } else if (memcmp(line_ready_buffer, COMMAND_ACC_ON, sizeof(COMMAND_ACC_ON)) == 0)
  {
    
  } else if (memcmp(line_ready_buffer, COMMAND_ACC_OFF, sizeof(COMMAND_ACC_OFF)) == 0)
  {
    
  } else if (memcmp(line_ready_buffer, COMMAND_MUTE, sizeof(COMMAND_MUTE)) == 0)
  {
    
  } else if (memcmp(line_ready_buffer, COMMAND_UNMUTE, sizeof(COMMAND_UNMUTE)) == 0)
  {
    
  } else if(memcmp(line_ready_buffer, COMMAND_CHANGE_DUTY, sizeof(COMMAND_CHANGE_DUTY)) == 0)
  {
    change_duty();
  }

  else
  {
    // If we receive an unknown command, we send an error message back to the PC
    CDC_Transmit_FS((uint8_t*)"Unknown command\r\n", 17);
  }
}

void init_codec_and_play()
{
  cs43l22_init();
  // sine signal
  for(int i = 0; i < AUDIO_BUFFER_LENGTH;i++)
  {
    buffer_audio[2 * i] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
    buffer_audio[2 * i + 1] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
  }
  cs43l22_play(buffer_audio, 2 * AUDIO_BUFFER_LENGTH);
}

/**
* @brief Go to stop mode
* @retval None
*/
void go_to_stop()
{
  // TODO: Make sure all user LEDS are off
  // To avoid noise during stop mode
  cs43l22_stop();

  // Required otherwise the audio wont work after wakeup
  HAL_I2S_DeInit(&hi2s3);

  // We disable the systick interrupt before going to stop (1ms tick)
  // Otherwise we would be woken up every 1ms
  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // We need to reconfigure the system clock after waking up.
  // After exiting from stop mode, the system clock is reset to the default
  // which is not the same we configure in cubeMx, so we do it as done
  // during the init, by calling SystemClock_Config().
  SystemClock_Config();
  HAL_ResumeTick();

  // Required otherwise the audio wont work after wakeup
  MX_I2S3_Init();

  init_codec_and_play();
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
