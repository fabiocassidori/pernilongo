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
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "config_robo.h"
#include "controle_motor.h"
#include "controle_succao.h"
#include "sensor_linha.h"
#include "marcador_lateral.h"
#include "controlador_pid.h"
#include "interface_usuario.h"
#include "comunicacao_bluetooth.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*
 * Máquina de estados simplificada em relação ao código original:
 * sem encoders não há mapeamento de pista, aceleração em retas por distância
 * nem desaceleração por velocidade medida. O restante do fluxo é o mesmo.
 */
typedef enum {
    ESTADO_INICIO,
    ESTADO_CALIBRANDO,
    ESTADO_AGUARDANDO_CORRIDA,
    ESTADO_CORRIDA,
    ESTADO_FORA_DA_PISTA,
    ESTADO_FINALIZADO,
    ESTADO_FRENANDO_FINAL,
    ESTADO_AGUARDANDO_REINICIO,
    ESTADO_ERRO
} EstadoRobo;

static volatile EstadoRobo g_estado_robo = ESTADO_INICIO;
static uint8_t g_fim_corrida = 0;
volatile bool g_flag_loop_1ms = false;
volatile bool g_flag_loop_10ms = false;
static volatile int g_contador_loop_lento = 0;
static volatile int g_contador_frenagem_ms = 0;
long range_posicao_sensor = SENSOR_POSICAO_MAX - SENSOR_POSICAO_MIN;
long range_posicao_interpolada = POSICAO_ENTRADA_MAX - POSICAO_ENTRADA_MIN;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void executar_loop_rapido(int pos_mapeada);
void executar_loop_lento(void);
static void aguardar_ms_atualizando(uint32_t duracao_ms);
static void desligar_tudo(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    iu_inicializar();
    motor_inicializar(&htim8);
    succao_inicializar(&htim2);
    sensor_linha_inicializar(&hadc1);
    marcador_lateral_inicializar(sensor_linha_obter_buffer_dma());
    pid_inicializar();
    bt_inicializar(&huart1);
    motor_parar(true);
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  sensor_linha_depuracao();          // Descomente para ver os 14 canais na bancada
//	  HAL_Delay(200);

	  // --- LOOP RÁPIDO (1ms) ---
	      if (g_flag_loop_1ms) {
	        g_flag_loop_1ms = false;
			int pos_bruta = sensor_linha_ler_posicao();
			int pos_mapeada = (int)(((long)(pos_bruta - SENSOR_POSICAO_MIN) * range_posicao_interpolada) / range_posicao_sensor + POSICAO_ENTRADA_MIN);

	          if (g_estado_robo == ESTADO_CORRIDA) {
	        	  //Verifica se saiu fora da pista
	        	  if (sensor_linha_is_robo_fora_da_pista(pos_mapeada)) {
					 // Se saiu, muda o estado e para os motores.
					 g_estado_robo = ESTADO_FORA_DA_PISTA;
					 desligar_tudo();
				 } else {
					 executar_loop_rapido(pos_mapeada);
				 }
	          } else if (g_estado_robo == ESTADO_FRENANDO_FINAL) {
	        	  executar_loop_rapido(pos_mapeada);
	          }
	      }

		   //--- LOOP LENTO (10ms) ---
		  if (g_flag_loop_10ms) {
			  g_flag_loop_10ms = false;
			  iu_atualizar();      // Atualiza buzina
			  succao_atualizar();  // Rampa do ventilador
			  if (g_estado_robo == ESTADO_CORRIDA) {
				  executar_loop_lento(); // Verifica marcadores de pista
			  }
		  }

		  // Máquina de estados principal
	  	  switch (g_estado_robo) {
	  		  case ESTADO_INICIO:
	  			  iu_led_verde(false);
	  			  iu_led_vermelho(false);
	  			  printf("Pressione o botao para calibrar.\r\n");
	  			  iu_aguardar_botao();
	  			  iu_bip_bloqueante(100);
	  			  g_estado_robo = ESTADO_CALIBRANDO;
	  			  break;

	  		  case ESTADO_CALIBRANDO:
	  			  iu_led_vermelho(true);   // Calibrando: mova o robô sobre a linha
	  			  marcador_lateral_resetar_calibracao();
	  			  sensor_linha_calibrar();
	  			  marcador_lateral_finalizar_calibracao();
	  			  iu_led_vermelho(false);
	  			  g_estado_robo = ESTADO_AGUARDANDO_CORRIDA;
	  			  break;

	  		  case ESTADO_AGUARDANDO_CORRIDA:
	  			  iu_led_verde(true);      // Pronto para largar
	  			  printf("Pressione para iniciar a corrida.\r\n");
	  			  iu_aguardar_botao();
	  			  iu_buzina_temporizada(150);
	  			  // Liga a sucção no botão: ela sobe em rampa durante a espera.
	  			  succao_definir_potencia(POTENCIA_SUCCAO_PCT);
	  			  aguardar_ms_atualizando(TEMPO_ESPERA_LARGADA_MS);
	  			  pid_inicializar();
	  			  g_fim_corrida = 0;
	  			  motor_parar(false);
	  			  g_estado_robo = ESTADO_CORRIDA;
	  			  break;

	  		  case ESTADO_CORRIDA:
	  			  break;

	  		  case ESTADO_FORA_DA_PISTA:
				  desligar_tudo(); // Mantém os motores e a sucção desligados
				  iu_led_verde(false);
				  iu_led_vermelho(true);
				  printf("ROBO PERDEU A LINHA!\r\n");
				  iu_buzina_temporizada(200);
				  iu_atualizar();
				  iu_aguardar_botao();
				  HAL_Delay(400);
				  //Reseta a um estado inicial seguro
				  g_estado_robo = ESTADO_INICIO;
				  break;

	  		  case ESTADO_FINALIZADO:
	  			  // Inicia a frenagem por contra-corrente (tratada no loop rápido)
	  			  g_contador_frenagem_ms = TEMPO_FRENAGEM_MS;
	  			  g_estado_robo = ESTADO_FRENANDO_FINAL;
	  			  break;

	  		  case ESTADO_FRENANDO_FINAL:
	  			  // A transição é feita pelo loop rápido quando o tempo de frenagem acaba.
	  			  break;

	  		  case ESTADO_AGUARDANDO_REINICIO:
	  			  iu_led_verde(false);
	  			  printf("Corrida finalizada. Pressione o botao para reiniciar.\r\n");
	  			  iu_aguardar_botao();
	  			  g_estado_robo = ESTADO_INICIO;
	  			  break;

	  		  case ESTADO_ERRO:
	  			   desligar_tudo();
	  			   iu_led_vermelho(true);
	  			   printf("ERRO! Sistema parado.\n");
	  			   while(1) {
	  				   iu_buzina_temporizada(100);
	                   iu_atualizar();
	  				   HAL_Delay(500);
	  			   }
	  			  break;
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 14;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
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
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIRA_Pin|DIRB_Pin|BUZINA_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWM_SENSOR_GPIO_Port, PWM_SENSOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIRA_Pin DIS_Pin DIRB_Pin BUZINA_Pin
                           LED_R_Pin */
  GPIO_InitStruct.Pin = DIRA_Pin|DIS_Pin|DIRB_Pin|BUZINA_Pin
                          |LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_G_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_SENSOR_Pin */
  GPIO_InitStruct.Pin = PWM_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWM_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTAO_Pin */
  GPIO_InitStruct.Pin = BOTAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOTAO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void executar_loop_rapido(int pos_mapeada) {

    // ESTADO DE CORRIDA: mesma lógica do original (potência base ± correção do PID),
    // porém em malha aberta, pois não há encoders para medir a velocidade.
	if (g_estado_robo == ESTADO_CORRIDA) {
		int correcao_direcao = pid_calcular_correcao(pos_mapeada);
		motor_definir_potencia(POTENCIA_BASE_CORRIDA - correcao_direcao, POTENCIA_BASE_CORRIDA + correcao_direcao);
	}
    // FRENAGEM FINAL: contra-corrente por TEMPO_FRENAGEM_MS e depois desliga tudo
	else if (g_estado_robo == ESTADO_FRENANDO_FINAL) {
		if (g_contador_frenagem_ms > 0) {
			motor_frear();
		} else {
			desligar_tudo();
			iu_buzina_temporizada(1000); // Som de finalização
			g_estado_robo = ESTADO_AGUARDANDO_REINICIO;
		}
	}
}

// Loop de 10ms
void executar_loop_lento(void) {
    // Contém a verificação de marcadores e outras tarefas não críticas
    TipoMarcador marcador = marcador_lateral_verificar();
    if (marcador == MARCADOR_NENHUM) {
        return;
    }

	if (marcador == MARCADOR_ESQUERDA || marcador == MARCADOR_DIREITA) {
		iu_buzina_temporizada(30);
		if (marcador == MARCADOR_DIREITA) {
			iu_buzina_temporizada(100);
			g_fim_corrida++;
		}
		// Verificação de fim de corrida (largada + chegada)
		if (g_fim_corrida >= MARCADORES_DIREITA_FIM_CORRIDA) {
			g_estado_robo = ESTADO_FINALIZADO;
		}
	} else if (marcador == MARCADOR_AMBOS) {
		//iu_buzina_temporizada(100);   // Cruzamento: ignorado, como no original
	}
}

/* Desliga motores (DIS alto) e sucção. */
static void desligar_tudo(void) {
	motor_parar(true);
	succao_desligar();
}

/* Espera sem bloquear a rampa da sucção nem a buzina (substitui HAL_Delay). */
static void aguardar_ms_atualizando(uint32_t duracao_ms) {
	uint32_t inicio = HAL_GetTick();
	while ((HAL_GetTick() - inicio) < duracao_ms) {
		if (g_flag_loop_10ms) {
			g_flag_loop_10ms = false;
			iu_atualizar();
			succao_atualizar();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		// A interrupção acontece a cada 1ms
		// 1. Aciona o loop rápido toda vez
		g_flag_loop_1ms = true;

		// 2. Decrementa o timer de frenagem se ele estiver ativo
		   if (g_contador_frenagem_ms > 0) {
			   g_contador_frenagem_ms--;
		   }

		/*3. O timer para lógica de reconhecimento de linha perdida
		   no arquivo sensor_linha.c*/
		   sensor_linha_tick_1ms();

		// 4. Usa um contador para acionar o loop lento a cada 10ms
		g_contador_loop_lento++;
		if (g_contador_loop_lento >= 10) {
		  g_flag_loop_10ms = true;
		  g_contador_loop_lento = 0;
		}
	  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	bt_callback_rx(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	bt_callback_erro(huart);
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
