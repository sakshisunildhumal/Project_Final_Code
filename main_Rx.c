/*CAN RECEIVER*/
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "SEGGER_SYSVIEW.h"
#include "string.h"
#include "stdio.h"
#include "core_cm4.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// This enum and struct should match the transmitting code for proper reception
typedef enum {
    CAN_SENSOR_ULTRASONIC = 0x101,
    CAN_SENSOR_LDR = 0x102,
    CAN_SENSOR_LIS3DSH = 0x103,
    CAN_SENSOR_DHT11 = 0x104
} SensorType_t;

typedef struct {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
} CAN_Message_t;

// A union is a safe way to convert a byte array to a floating point number and back.
typedef union {
    float float_value;
    uint8_t byte_array[4];
} FloatConverter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MESSAGE_QUEUE_LENGTH 10 // Size of the CAN message queue
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
QueueHandle_t xCanRxQueue;

// Global variables to hold the latest sensor data
volatile uint32_t latest_distance = 0;
volatile float latest_ldr_voltage = 0.0;
volatile uint8_t latest_accel_x = 0;
volatile uint8_t latest_accel_y = 0;
volatile uint8_t latest_accel_z = 0;
volatile uint8_t latest_temp_int = 0;
volatile uint8_t latest_temp_frac = 0;
volatile uint8_t latest_hum_int = 0;
volatile uint8_t latest_hum_frac = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void vCanAndLcdTask(void *pvParameters);
void vToggleLEDs(void);

// LCD Functions
void timer_delay_us(uint32_t us);
void LCD_Send4Bits(uint8_t nibble);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);
void LCD_Init(void);
void LCD_String(char *str);
void LCD_PrintInt(int32_t num);
void LCD_PrintFloat(float num, uint8_t precision);
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
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();

      // Configure CAN filter to accept messages from the specified IDs
      CAN_filterConfig();

      // Create the FreeRTOS message queue for CAN messages
      xCanRxQueue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CAN_Message_t));
      if (xCanRxQueue == NULL) {
          Error_Handler();
      }

      // Start the CAN peripheral
      if (HAL_CAN_Start(&hcan1) != HAL_OK)
      {
        Error_Handler();
      }

      HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);

      // Activate the CAN RX FIFO 0 message pending interrupt
      if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      {
          Error_Handler();
      }

      // Start the TIM2 peripheral, used for LCD delays.
      if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
      {
          Error_Handler();
      }

      // Create a single task to handle both CAN reception processing and LCD display.
      if(xTaskCreate(vCanAndLcdTask, "CAN_and_LCD_Task", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
      {
          Error_Handler();
      }

      // Start the FreeRTOS scheduler
      vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  // Prescaler changed from 16 to 47 for a 1us delay with a 48MHz timer clock
  htim2.Init.Prescaler = 96;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Configure PB8 and PB9 for CAN1
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

/**
 * @brief Configures the CAN filter to accept specific IDs.
 */
void CAN_filterConfig(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

  // Filter 1: Accept Ultrasonic and LDR messages
  sFilterConfig.FilterIdHigh = CAN_SENSOR_ULTRASONIC << 5; // 0x101 << 5 = 0x2020
  sFilterConfig.FilterIdLow = CAN_SENSOR_LDR << 5;         // 0x102 << 5 = 0x2040
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Filter 2: Accept LIS3DSH and DHT11 messages (use a different filter bank)
  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterIdHigh = CAN_SENSOR_LIS3DSH << 5;    // 0x103 << 5 = 0x2060
  sFilterConfig.FilterIdLow = CAN_SENSOR_DHT11 << 5;        // 0x104 << 5 = 0x2080
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // All messages go to FIFO0
  sFilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief This is the Interrupt Service Routine (ISR) callback for CAN Rx.
 * It is triggered when a new message is received in FIFO 0.
 * The function reads the message and sends it to the single FreeRTOS queue.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Message_t rxMessage;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMessage.RxHeader, rxMessage.RxData) == HAL_OK)
    {
        // Update global variables with the latest sensor data
        switch (rxMessage.RxHeader.StdId) {
            case CAN_SENSOR_ULTRASONIC:
                memcpy((void*)&latest_distance, rxMessage.RxData, sizeof(latest_distance));
                break;
            case CAN_SENSOR_LDR:
            {
                FloatConverter converter;
                memcpy(converter.byte_array, rxMessage.RxData, sizeof(float));
                latest_ldr_voltage = converter.float_value;
                break;
            }
            case CAN_SENSOR_LIS3DSH:
                latest_accel_x = (uint8_t)(((uint8_t)rxMessage.RxData[1] << 8) | rxMessage.RxData[0]);
                latest_accel_y = (uint8_t)(((uint8_t)rxMessage.RxData[3] << 8) | rxMessage.RxData[2]);
                latest_accel_z = (uint8_t)(((uint8_t)rxMessage.RxData[5] << 8) | rxMessage.RxData[4]);
                break;
            case CAN_SENSOR_DHT11:
                latest_temp_int = rxMessage.RxData[0];
                latest_temp_frac = rxMessage.RxData[2];
                latest_hum_int = rxMessage.RxData[1];
                latest_hum_frac = rxMessage.RxData[3];
                break;
            default:
                break;
        }

        if (xQueueSendFromISR(xCanRxQueue, &rxMessage, &xHigherPriorityTaskWoken) != pdPASS)
        {
            // The queue is full.
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Toggles all LEDs together to indicate a successful reception.
 */
void vToggleLEDs(void)
{
    static GPIO_PinState pin_state = GPIO_PIN_RESET;
    uint16_t all_pins = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

    // Toggle all four LEDs
    pin_state = (pin_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOD, all_pins, pin_state);
}

//----------------------------------------------------------------------------------------------------------
// LCD Functions using direct GPIO
//----------------------------------------------------------------------------------------------------------
void timer_delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
    HAL_TIM_Base_Stop(&htim2);
}

void LCD_Send4Bits(uint8_t nibble)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (nibble >> 0) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, (nibble >> 1) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (nibble >> 2) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (nibble >> 3) & 0x01);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // EN pin high
    timer_delay_us(1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // EN pin low
    timer_delay_us(1);
}

void LCD_Command(uint8_t cmd)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); // RS pin low (command)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // RW pin low (write)

    LCD_Send4Bits(cmd >> 4);
    LCD_Send4Bits(cmd & 0x0F);
    timer_delay_us(50);
}

void LCD_Data(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); // RS pin high (data)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // RW pin low (write)

    LCD_Send4Bits(data >> 4);
    LCD_Send4Bits(data & 0x0F);
    timer_delay_us(50);
}

void LCD_String(char *str)
{
    while (*str)
    {
        LCD_Data(*str++);
    }
}

void LCD_PrintInt(int32_t num)
{
    char buffer[10];
    int8_t i = 0;
    int8_t is_negative = 0;

    if (num == 0) {
        LCD_Data('0');
        return;
    }

    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    while (num > 0) {
        buffer[i++] = (num % 10) + '0';
        num /= 10;
    }

    if (is_negative) {
        LCD_Data('-');
    }

    for (int8_t j = i - 1; j >= 0; j--) {
        LCD_Data(buffer[j]);
    }
}

void LCD_PrintFloat(float num, uint8_t precision)
{
    int32_t int_part = (int32_t)num;
    LCD_PrintInt(int_part);

    if (precision > 0) {
        LCD_Data('.');
        float frac_part = num - (float)int_part;
        for (uint8_t i = 0; i < precision; i++) {
            frac_part *= 10;
        }
        LCD_PrintInt((int32_t)(frac_part + 0.5f)); // Round to nearest integer
    }
}

void LCD_Init(void)
{
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

    LCD_Send4Bits(0x03);
    HAL_Delay(5);

    LCD_Send4Bits(0x03);
    HAL_Delay(5);

    LCD_Send4Bits(0x03);
    HAL_Delay(1);

    LCD_Send4Bits(0x02);

    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x06);
    LCD_Command(0x01);
    HAL_Delay(2);
}

/**
 * @brief Combined task to handle CAN reception processing, condition checks, and LCD display.
 */
void vCanAndLcdTask(void* pvParameters)
{
    CAN_Message_t receivedMessage;
    FloatConverter converter;

    LCD_Init();
    LCD_Command(0x01); // Clear LCD
    LCD_Command(0x80); // Set cursor to row 0, col 0
    LCD_String("Waiting CAN...");
    LCD_Command(0xC0); // Set cursor to row 1, col 0
    LCD_String("Waiting CAN...");
    SEGGER_SYSVIEW_PrintfHost("LCD initialized. Waiting for CAN messages.\n");

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
    	if (xQueueReceive(xCanRxQueue, &receivedMessage, portMAX_DELAY) == pdPASS)
        {
            vToggleLEDs();

            // Update global variables with the latest sensor data
            switch (receivedMessage.RxHeader.StdId) {
                case CAN_SENSOR_ULTRASONIC:
                    memcpy((void*)&latest_distance, receivedMessage.RxData, sizeof(latest_distance));
                    SEGGER_SYSVIEW_PrintfHost("Ultrasonic: %lu cm\n", latest_distance);
                    break;
                case CAN_SENSOR_LDR:
                    memcpy(converter.byte_array, receivedMessage.RxData, sizeof(float));
                    latest_ldr_voltage = converter.float_value;
                    // Correctly printing LDR float value to SEGGER
                    int ldr_int_part = (int)latest_ldr_voltage;
                    int ldr_frac_part = (int)((latest_ldr_voltage - ldr_int_part) * 100);
                    SEGGER_SYSVIEW_PrintfHost("LDR: %d.%02d V\n", ldr_int_part, ldr_frac_part);
                    break;
                case CAN_SENSOR_LIS3DSH:
                    latest_accel_x = (int16_t)(((uint16_t)receivedMessage.RxData[1] << 8) | receivedMessage.RxData[0]);
                    latest_accel_y = (int16_t)(((uint16_t)receivedMessage.RxData[3] << 8) | receivedMessage.RxData[2]);
                    latest_accel_z = (int16_t)(((uint16_t)receivedMessage.RxData[5] << 8) | receivedMessage.RxData[4]);
                    SEGGER_SYSVIEW_PrintfHost("LIS3DSH - X:%d, Y:%d, Z:%d\n", latest_accel_x, latest_accel_y, latest_accel_z);
                    break;
                case CAN_SENSOR_DHT11:
                    latest_temp_int = receivedMessage.RxData[0];
                    latest_temp_frac = receivedMessage.RxData[2];
                    latest_hum_int = receivedMessage.RxData[1];
                    latest_hum_frac = receivedMessage.RxData[3];
                    // Correctly printing DHT11 values to SEGGER
                    SEGGER_SYSVIEW_PrintfHost("DHT11 - T:%d.%d C, H:%d.%d %%\n", latest_temp_int, latest_temp_frac, latest_hum_int, latest_hum_frac);
                    break;
                default:
                    break;
            }
        }

        // --- Conditional Logic for Operations ---
        // 1st condition: "Upper On"
        if ((latest_ldr_voltage > 2.5f) &&
            (latest_accel_x <= 70 && latest_accel_y <= 70 && latest_accel_z <= 70) &&
            (latest_distance < 25))
        {
            // Corrected GPIO pins to match MX_GPIO_Init
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

            LCD_Command(0x01);
            LCD_Command(0x80);
            LCD_String("Upper ON");
            LCD_Command(0xC0);
            LCD_String("Upper ON");
        }
        // 2nd condition: "Depper On"
        else if ((latest_ldr_voltage > 2.5f) &&
                 (latest_accel_x > 70 || latest_accel_y > 70 || latest_accel_z > 70) &&
                 (latest_distance > 25))
        {
        	// Corrected GPIO pins to match MX_GPIO_Init
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

            LCD_Command(0x01);
            LCD_Command(0x80);
            LCD_String("Depper ");
            LCD_Command(0xC0);
            LCD_String("Depper ON");

        }
        // 3rd condition: "Fogg Detected"
        else if ((latest_temp_int <= 12) && (latest_hum_int >= 90))
        {
        	// Corrected GPIO pins to match MX_GPIO_Init
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);


            LCD_Command(0x01);
            LCD_Command(0x80);
            LCD_String("Fogg Det");
            LCD_Command(0xC0);
            LCD_String("Fogg Detected");
        }
        // Default condition: "No Operations needed"
        else
        {
        	// Corrected GPIO pins to match MX_GPIO_Init
        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

            LCD_Command(0x01);
            LCD_Command(0x80);
            LCD_String("No Opera");
            LCD_Command(0xC0);
            LCD_String("No Operation");
        }

        // Add a small delay to prevent the task from running too fast
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//----------------------------------------------------------------------------------------------------------
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
     SEGGER_SYSVIEW_PrintfHost("HAL Error occurred!");
       __disable_irq();
       while (1)
       {
         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Turn on an LED for error
       }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s on line %d\r\n", file, line);
        while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
