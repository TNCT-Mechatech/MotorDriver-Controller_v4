/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>

#include "QEI.hpp"
#include "PID.hpp"
#include "MotorDriver.hpp"
#include "ThermalSensor.hpp"
#include "CurrentSensor.hpp"
#include "NoOperator.hpp"
#include "MDOperator.hpp"
#include "PIDOperator.hpp"

//	Serial Bridge
#include <STM32HardwareSPI.h>
#include <ACAN2517FD.h>
#include "CANSerialBridge.hpp"
//  Serial Bridge Message
#include "MessageID.hpp"
#include "CommandMessage.hpp"
#include "AcknowledgeMessage.hpp"
#include "SettingMessage.hpp"
#include "TargetMessage.hpp"
#include "FeedbackMessage.hpp"

using namespace acan2517fd;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//  Control interval 2[ms]
#define CTRL_INTERVAL (1.0 / 500)
//  SerialBridge interval 20[ms]
#define SB_INTERVAL (1.0 / 50)
//  Ping interval 3[hz]
#define PING_INTERVAL (1.0 / 2)
//  Timeout 1.0[s]
#define TIMEOUT 1.0

#define M1 0U
#define M2 1U
#define M3 2U
#define M4 3U

#define CTRL_FAULT_COUNT_LIMIT 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief tim_countから実時間[マイクロ秒]を取得する
 */
#define TIM_COUNT_US (tim_count * 100UL) //[us]

/**
 * @brief tim_countに値xを入れる
 * @param[in] x 入力値
 */
#define TIM_COUNT_SET(x) (tim_count = x)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//	timer count [100us/count]
volatile long tim_count = 0;

//  controller status. true->active false->stop
volatile bool ctrl_enabled = false;
//  timer count of controlled at last time
volatile long last_ctrl_at = 0;
volatile long last_sb_at = 0;
volatile long last_ping_at = 0;
volatile long last_received_at = 0;


//  fault counter
volatile int ctrl_fault_count = 0;

//	debug mode
volatile bool is_debug = false;

//	device id
volatile uint8_t device_id = 0;

//  MD
MotorDriver *md[4];
//  QEI
QEI *encoder[4];
//  PID
PID *vel_ctrl[4];
//  PID variable (target,feedback,output)
PID::ctrl_variable_t v_vel[4];
//  PID parameter (gain,pid_mode)
PID::ctrl_param_t p_vel[4];
//  Operator
Operator *operators[4];

//  thermal
ThermalSensor *thermal;
//  current sensor
CurrentSensor *current_sensor;


//	SerialBridge
uint32_t get_milliseconds() {
    return (uint32_t) (tim_count / 1E3);
}

STM32HardwareSPI dev_spi(&hspi2, SPI_CS_GPIO_Port, SPI_CS_Pin);
ACAN2517FD dev_can(dev_spi, get_milliseconds);
ACAN2517FDSettings can_settings(ACAN2517FDSettings::OSC_4MHz, 500UL * 1000UL, DataBitRateFactor::x2);
CANSerialBridge serial(&dev_can);

CommandMessage command_msg;
AcknowledgeMessage acknowledge_msg;
SettingMessage setting_msg;
TargetMessage target_msg;
FeedbackMessage feedback_msg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static void Init_Controller(void);

static void Init_Settings(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void reinitCANFDController();

//  LED
inline void toggleAcknowledge();

inline void toggleAlive();

inline void toggleDebug();

//  Emergency Stop
inline void unlockEmergencyStop();

inline void lockEmergencyStop();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern "C" {
int _write(int file, char *ptr, int len) {
    if (is_debug) {
        HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 10);
    }
    return len;
}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    setbuf(stdout, NULL);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
    //  Emergency Stop
    lockEmergencyStop();

    //  initialize controller
    Init_Controller();
    Init_Settings();


    //	CAN Setting
    can_settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

    can_settings.mDriverTransmitFIFOSize = 5;
    can_settings.mDriverReceiveFIFOSize = 5;

    //	initialize can controller
    const uint32_t canInitError = dev_can.begin(can_settings);
    if (canInitError == 0) {
        printf("Initialized CAN FD Controller.\n\r");
        toggleAcknowledge();
    } else {
        printf("Failed to initialize CAN FD Controller. error: 0x%lx\n\r", canInitError);
        return -1;
    }

    //  add frame
    const static uint32_t COMMAND_ID = resolve_id(device_id, MessageID::COMMAND);
    const static uint32_t ACKNOWLEDGE_ID = resolve_id(device_id, MessageID::ACKNOWLEDGE);
    const static uint32_t SETTING_ID = resolve_id(device_id, MessageID::SETTING);
    const static uint32_t TARGET_ID = resolve_id(device_id, MessageID::TARGET);
    const static uint32_t FEEDBACK_ID = resolve_id(device_id, MessageID::FEEDBACK);

    if (serial.add_frame(COMMAND_ID, &command_msg) != 0) {
        printf("Failed to register COMMAND message.\n\r");
    }
    if (serial.add_frame(ACKNOWLEDGE_ID, &acknowledge_msg) != 0) {
        printf("Failed to register ACKNOWLEDGE message.\n\r");
    }
    if (serial.add_frame(SETTING_ID, &setting_msg) != 0) {
        printf("Failed to register SETTING message.\n\r");
    }
    if (serial.add_frame(TARGET_ID, &target_msg) != 0) {
        printf("Failed to register TARGET message.\n\r");
    }
    if (serial.add_frame(FEEDBACK_ID, &feedback_msg) != 0) {
        printf("Failed to register FEEDBACK message.\n\r");
    }

    printf("registered messages.\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        //  poll CAN FD Controller
        dev_can.poll();

        double ping_tim = (TIM_COUNT_US - last_ping_at) / 1E6;
        if (ping_tim > PING_INTERVAL) {
            //  set ping command
            command_msg.data.command = Command::PING;
            command_msg.data.timestamp = TIM_COUNT_US / 65536;

            serial.write(COMMAND_ID);
            last_ping_at = TIM_COUNT_US;
        }

        double sb_tim = (TIM_COUNT_US - last_sb_at) / 1E6;
        if (sb_tim > SB_INTERVAL) {
            serial.update();

            //  command message
            if (command_msg.was_updated()) {
                if (command_msg.data.command == Command::RESET) {
                    for (int i = 0; i < 4; ++i) {
                        //  reset operator
                        operators[i]->reset();
                        operators[i] = new NoOperator();
                        //  md
                        md[i]->set(0);
                        //  encoder
                        encoder[i]->reset_count();
                        //  pid
                        vel_ctrl[i]->reset();
                    }
                }
                else if (command_msg.data.command == Command::ENCODER_1_COUNT_RESET) {
                    encoder[M1]->reset_count();
                    vel_ctrl[M1]->reset();
                }
                else if (command_msg.data.command == Command::ENCODER_2_COUNT_RESET) {
                    encoder[M2]->reset_count();
                    vel_ctrl[M2]->reset();
                }
                else if (command_msg.data.command == Command::ENCODER_3_COUNT_RESET) {
                    encoder[M3]->reset_count();
                    vel_ctrl[M3]->reset();
                }
                else if (command_msg.data.command == Command::ENCODER_4_COUNT_RESET) {
                    encoder[M4]->reset_count();
                    vel_ctrl[M4]->reset();
                }

                //  acknowledge
                acknowledge_msg.data.timestamp = command_msg.data.timestamp;
                serial.write(ACKNOWLEDGE_ID);

                //  update timestamp last received at
                last_received_at = TIM_COUNT_US;
                //  toggle acknowledge
                toggleAcknowledge();
            }

            if (acknowledge_msg.was_updated()) {
                //  update timestamp last received at
                last_received_at = TIM_COUNT_US;
                //  toggle acknowledge
                toggleAcknowledge();
            }

            //  setting message
            if (setting_msg.was_updated()) {
                if ((uint8_t) setting_msg.data.motorId < 4) {
                    uint32_t id = (uint8_t) setting_msg.data.motorId;
                    //  reset
                    operators[id]->reset();

                    //  set inverse
                    md[id]->set_inverse_dir(setting_msg.data.reverse);

                    //  set scale
                    encoder[id]->set_scale(1.0 / 2.0 / setting_msg.data.scale);

                    //  mode
                    switch (setting_msg.data.mode) {
                        case OperatorMode::NO_OPERATOR:
                            operators[id] = new NoOperator();
                            break;
                        case OperatorMode::MD_OPERATOR:
                            operators[id] = new MDOperator(md[id], encoder[id], &v_vel[id],
                                                           setting_msg.data.encoderType);
                            break;
                        case OperatorMode::PID_OPERATOR:
                            operators[id] = new PIDOperator(md[id], encoder[id], vel_ctrl[id], &v_vel[id],
                                                            setting_msg.data.encoderType);
                            break;
                    }

                    //  gain
                    p_vel[id].kp = setting_msg.data.kp;
                    p_vel[id].ki = setting_msg.data.ki;
                    p_vel[id].kd = setting_msg.data.kd;
                    p_vel[id].forward_gain = setting_msg.data.forward_gain;

                    //  reset pid
                    vel_ctrl[id]->reset();

                    //  acknowledge
                    acknowledge_msg.data.timestamp = setting_msg.data.timestamp;
                    serial.write(ACKNOWLEDGE_ID);

                    //  update timestamp last received at
                    last_received_at = TIM_COUNT_US;
                    //  toggle acknowledge
                    toggleAcknowledge();

                    printf("node %lu was updated\n\r", id);
                }
            }

            //  target message
            if (target_msg.was_updated()) {
                for (int i = 0; i < 4; ++i) {
                    v_vel[i].target = (double) target_msg.data.target[i];
                }

                //  reply feedback
                if (ctrl_enabled) {
                    for (int i = 0; i < 4; i++) {
                        feedback_msg.data.node[i].angle = (float) encoder[i]->get_angle();
                        feedback_msg.data.node[i].velocity = (float) encoder[i]->get_velocity();
                    }

                    //  write
                    serial.write(FEEDBACK_ID);
                }

                //  update timestamp last received at
                last_received_at = TIM_COUNT_US;
                //  toggle acknowledge
                toggleAcknowledge();
            }

            last_sb_at = TIM_COUNT_US;
        }

        if (ctrl_enabled) {
            double tim = (TIM_COUNT_US - last_ctrl_at) / 1E6;

            double elapsed_time = (TIM_COUNT_US - last_received_at) / 1E6;
            if (elapsed_time > TIMEOUT) {
                for (auto &i: md) {
                    i->set(0.0);
                }

                //  reset controller
                reinitCANFDController();

                last_received_at = TIM_COUNT_US;
                //  for minimal diff time
                last_ctrl_at = TIM_COUNT_US;

                continue;
            }

            if (tim >= CTRL_INTERVAL) {
                operators[M1]->step(tim);
                operators[M2]->step(tim);
                operators[M3]->step(tim);
                operators[M4]->step(tim);

//                if ((operators[0]->mode() == OperatorMode::PID_OPERATOR && v_vel[M1].target * v_vel[M1].feedback < 0)
//                    || (operators[1]->mode() == OperatorMode::PID_OPERATOR && v_vel[M2].target * v_vel[M2].feedback < 0)
//                    || (operators[2]->mode() == OperatorMode::PID_OPERATOR && v_vel[M3].target * v_vel[M3].feedback < 0)
//                    || (operators[3]->mode() == OperatorMode::PID_OPERATOR && v_vel[M4].target * v_vel[M4].feedback < 0)
//                        ) {
//                    ctrl_fault_count++;
//                } else {
//                    ctrl_fault_count = 0;
//                }
//
//                //  too many fault, shut down
//                if (ctrl_fault_count > CTRL_FAULT_COUNT_LIMIT)
//                    Error_Handler();

                //  update last controlled time
                last_ctrl_at = TIM_COUNT_US;

                //  toggle alive led
                toggleAlive();
            }
        }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 640;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
//  HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EMERGENCT_STOP_Pin|DEBUG_LED_Pin|COM_LED_Pin|ALIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIR_1_Pin|DIR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_3_Pin|DIR_4_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EMERGENCT_STOP_Pin DEBUG_LED_Pin COM_LED_Pin ALIVE_LED_Pin */
  GPIO_InitStruct.Pin = EMERGENCT_STOP_Pin|DEBUG_LED_Pin|COM_LED_Pin|ALIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_1_Pin DIR_2_Pin */
  GPIO_InitStruct.Pin = DIR_1_Pin|DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_3_Pin DIR_4_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = DIR_3_Pin|DIR_4_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP_SW_4_Pin DIP_SW_3_Pin DIP_SW_2_Pin */
  GPIO_InitStruct.Pin = DIP_SW_4_Pin|DIP_SW_3_Pin|DIP_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP_SW_1_Pin */
  GPIO_InitStruct.Pin = DIP_SW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIP_SW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_BUTTON_Pin DEBUG_SWITCH_Pin */
  GPIO_InitStruct.Pin = DEBUG_BUTTON_Pin|DEBUG_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void Init_Controller(void) {
    //  init MD
    md[M1] = new MotorDriver(
            &htim2,
            TIM_CHANNEL_1,
            DIR_1_GPIO_Port,
            DIR_1_Pin,
            true,
            1.0,
            0.05
    );
    md[M2] = new MotorDriver(
            &htim2,
            TIM_CHANNEL_2,
            DIR_2_GPIO_Port,
            DIR_2_Pin,
            true,
            1.0,
            0.05
    );
    md[M3] = new MotorDriver(
            &htim2,
            TIM_CHANNEL_3,
            DIR_3_GPIO_Port,
            DIR_3_Pin,
            false,
            1.0,
            0.05
    );
    md[M4] = new MotorDriver(
            &htim2,
            TIM_CHANNEL_4,
            DIR_4_GPIO_Port,
            DIR_4_Pin,
            false,
            1.0,
            0.05
    );

    //  init QEI
    encoder[M1] = new QEI(
            &htim1,
            1.0 / 2048 / 2
    );
    encoder[M2] = new QEI(
            &htim3,
            1.0 / 2048 / 2
    );
    encoder[M3] = new QEI(
            &htim4,
            1.0 / 2048 / 2
    );
    encoder[M4] = new QEI(
            &htim8,
            1.0 / 2048 / 2
    );

    //  pid velocity
    v_vel[M1] = PID::ctrl_variable_t{0, 0, 0};
    v_vel[M2] = PID::ctrl_variable_t{0, 0, 0};
    v_vel[M3] = PID::ctrl_variable_t{0, 0, 0};
    v_vel[M4] = PID::ctrl_variable_t{0, 0, 0};

    //  pid parameter
    p_vel[M1] = PID::ctrl_param_t{0.0, 0.0, 0.0, 0.0, false};
    p_vel[M2] = PID::ctrl_param_t{0.0, 0.0, 0.0, 0.0, false};
    p_vel[M3] = PID::ctrl_param_t{0.0, 0.0, 0.0, 0.0, false};
    p_vel[M4] = PID::ctrl_param_t{0.0, 0.0, 0.0, 0.0, false};

    //  init pid
    vel_ctrl[M1] = new PID(&v_vel[M1], &p_vel[M1]);
    vel_ctrl[M2] = new PID(&v_vel[M2], &p_vel[M2]);
    vel_ctrl[M3] = new PID(&v_vel[M3], &p_vel[M3]);
    vel_ctrl[M4] = new PID(&v_vel[M4], &p_vel[M4]);

    //  current
    current_sensor = new CurrentSensor(&hadc1, 4);

    //  Operators
    operators[M1] = new NoOperator();
    operators[M2] = new NoOperator();
    operators[M3] = new NoOperator();
    operators[M4] = new NoOperator();

    //  thermal
    thermal = new ThermalSensor(&hadc2);

    //  completed initializing
    ctrl_enabled = true;

    //  unlock emergency stop
    unlockEmergencyStop();
}

void Init_Settings() {
    //	check if it is debug mode
    is_debug = HAL_GPIO_ReadPin(DEBUG_SWITCH_GPIO_Port, DEBUG_SWITCH_Pin);

    //	read Device ID
    bool dip[4];
    dip[0] = HAL_GPIO_ReadPin(DIP_SW_1_GPIO_Port, DIP_SW_1_Pin);
    dip[1] = HAL_GPIO_ReadPin(DIP_SW_2_GPIO_Port, DIP_SW_2_Pin);
    dip[2] = HAL_GPIO_ReadPin(DIP_SW_3_GPIO_Port, DIP_SW_3_Pin);
    dip[3] = HAL_GPIO_ReadPin(DIP_SW_4_GPIO_Port, DIP_SW_4_Pin);
    device_id = dip[3] * 8 + dip[2] * 4 + dip[1] * 2 + dip[0];

    printf("device id: %d\n\r", device_id);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        tim_count++;
    }
}

void reinitCANFDController() {
    dev_can.end();
    //  init
    dev_can.begin(can_settings);

    printf("reinit controller\n\r");
}


inline void toggleAcknowledge() {
    HAL_GPIO_TogglePin(COM_LED_GPIO_Port, COM_LED_Pin);
}

inline void toggleAlive() {
    HAL_GPIO_TogglePin(ALIVE_LED_GPIO_Port, ALIVE_LED_Pin);
}

inline void toggleDebug() {
    HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
}

//  Emergency Stop
inline void unlockEmergencyStop() {
    HAL_GPIO_WritePin(EMERGENCT_STOP_GPIO_Port, EMERGENCT_STOP_Pin, GPIO_PIN_SET);
}

inline void lockEmergencyStop() {
    HAL_GPIO_WritePin(EMERGENCT_STOP_GPIO_Port, EMERGENCT_STOP_Pin, GPIO_PIN_RESET);
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
    while (1) {
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
