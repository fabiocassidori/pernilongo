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
#include "leitor_encoder.h"
#include "sensor_linha.h"
#include "marcador_lateral.h"
#include "controlador_pid.h"
#include "controle_velocidade.h"
#include "mapa_pista.h"
#include "monitor_bateria.h"
#include "interface_usuario.h"

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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

typedef enum {
    ESTADO_INICIO,
    ESTADO_CALIBRANDO,
    ESTADO_AGUARDANDO_MAPEAMENTO,
    ESTADO_MAPEAMENTO,
    ESTADO_AGUARDANDO_LOG,
    ESTADO_LOGANDO,
    ESTADO_AGUARDANDO_CORRIDA,
    ESTADO_CORRIDA,
	ESTADO_ACELERANDO_RETA,
	ESTADO_FRENAGEM,
	ESTADO_FORA_DA_PISTA,
    ESTADO_FINALIZADO,
	ESTADO_DESACELERANDO_FINAL,
	ESTADO_AGUARDANDO_LOG_VELOCIDADE,
	ESTADO_LOGANDO_VELOCIDADE,
    ESTADO_ERRO
} EstadoRobo;

static EstadoRobo g_estado_robo = ESTADO_INICIO;

// +++  VARIÁVEL PARA VELOCIDADE MÁXIMA +++
static float g_velocidade_maxima_registrada_mmps = 0.0f;

static int32_t g_dist_entrada_segmento_pulsos = 0;
static uint8_t g_fim_corrida = 0;
static uint8_t g_fim_mapeamento = 0;

volatile bool g_flag_loop_1ms = false;
volatile bool g_flag_loop_10ms = false;
static volatile int g_contador_loop_lento = 0;
static volatile int g_contador_frenagem_ms = 0;
static volatile bool g_frenagem_realizada_no_segmento = false;
long range_posicao_sensor = SENSOR_POSICAO_MAX - SENSOR_POSICAO_MIN;
long range_posicao_interpolada = POSICAO_ENTRADA_MAX - POSICAO_ENTRADA_MIN;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void executar_loop_rapido(int pos_mapeada);
void executar_loop_lento(void);
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
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    iu_inicializar();
    motor_inicializar(&htim8);
    encoder_inicializar(&htim4, &htim3);
    velocidade_inicializar();
    sensor_linha_inicializar(&hadc1);
    marcador_lateral_inicializar(sensor_linha_obter_buffer_dma());
    pid_inicializar();
    mapa_inicializar();
    bateria_inicializar(&hadc2);

    motor_parar(true);
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  encoder_atualizar_posicoes();

//	  bool esta_na_linha = sensor_linha_esta_na_linha();
//	  int pos_linha = sensor_linha_ler_posicao();
//	  printf("posicao: %d\n", pos_linha);
//	  printf("esta na linha: %s\n", esta_na_linha ? "true" : "false");//


	  // --- LOOP RÁPIDO (1ms) ---
	      if (g_flag_loop_1ms) {
	        g_flag_loop_1ms = false;
			// A lógica de leitura e mapeamento deve estar aqui para a verificação
			int pos_bruta = sensor_linha_ler_posicao();
//			printf("posicao bruta: %d\n", pos_bruta);
			int pos_mapeada = (int)(((long)(pos_bruta - SENSOR_POSICAO_MIN) * range_posicao_interpolada) / range_posicao_sensor + POSICAO_ENTRADA_MIN);
//			printf("posicao mapeada: %d\n", pos_mapeada);
//			HAL_Delay(200);

	          if (g_estado_robo == ESTADO_MAPEAMENTO ||
				  g_estado_robo == ESTADO_CORRIDA ||
				  g_estado_robo == ESTADO_ACELERANDO_RETA ||
				  g_estado_robo == ESTADO_FRENAGEM) {

	        	  //Verifica se saiu forda da pista
	        	  if (sensor_linha_is_robo_fora_da_pista(pos_mapeada)) {
					 // Se saiu, muda o estado e para os motores.
					 // Nenhuma outra ação de controle de movimento será executada neste ciclo.
					 g_estado_robo = ESTADO_FORA_DA_PISTA;
					 motor_parar(true); // Este comando agora não será sobreposto
				 } else {
					 // Se NÃO saiu, executa a lógica normal de controle de velocidade e direção.
					 executar_loop_rapido(pos_mapeada);
				 }
	          }
	      }

		   //--- LOOP LENTO (10ms) ---
		  if (g_flag_loop_10ms) {

			  g_flag_loop_10ms = false;
			  bateria_atualizar();
			  iu_atualizar(); // Atualiza buzina

			  if (g_estado_robo == ESTADO_MAPEAMENTO ||
			      g_estado_robo == ESTADO_CORRIDA ||
			      g_estado_robo == ESTADO_ACELERANDO_RETA) {

				  executar_loop_lento(); // Verifica marcadores de pista
			  }
		  }

		  // Máquina de estados principal
	  	  switch (g_estado_robo) {
	  		  case ESTADO_INICIO:
	  			  printf("Pressione o botao para calibrar.\r\n");
	  			  iu_aguardar_botao();
	  			  iu_bip_bloqueante(100);
	  			  g_estado_robo = ESTADO_CALIBRANDO;
	  			  break;

	  		  case ESTADO_CALIBRANDO:
	  			  marcador_lateral_resetar_calibracao();
	  			  sensor_linha_calibrar();
	  			  marcador_lateral_finalizar_calibracao();
	  			  g_estado_robo = ESTADO_AGUARDANDO_MAPEAMENTO;
	  			  break;

	  		  case ESTADO_AGUARDANDO_MAPEAMENTO:
	  			  printf("Pressione para mapear a pista.\r\n");
	  			  iu_aguardar_botao();
	  			  iu_buzina_temporizada(150);
	  			  HAL_Delay(3000);
	  			  motor_parar(false);
	  			  encoder_resetar_posicoes();
	  			  g_fim_mapeamento = 0;
	  			  g_estado_robo = ESTADO_MAPEAMENTO;
	  			  break;

	  		  case ESTADO_MAPEAMENTO:
	  			    if (g_fim_mapeamento >= 2) {
	  			        motor_parar(true);
	  			        printf("Mapeamento concluido.\r\n");
	  			        iu_buzina_temporizada(300);
	  			        g_estado_robo = ESTADO_AGUARDANDO_LOG;
	  			    }
	  			  break;

	  		  case ESTADO_AGUARDANDO_LOG:
	  			  printf("Pressione para imprimir o mapa via SWO.\r\n");
	  			  iu_aguardar_botao();
	  			  g_estado_robo = ESTADO_LOGANDO;
	  			  break;

	  		  case ESTADO_LOGANDO:
	  			  mapa_logar_dados_swo();
	  			  iu_buzina_temporizada(500);
	  			  g_estado_robo = ESTADO_AGUARDANDO_CORRIDA;
	  			  break;

	  		  case ESTADO_AGUARDANDO_CORRIDA:
	  			  printf("Pressione para iniciar a corrida.\r\n");
	  			  iu_aguardar_botao();
	  			  iu_buzina_temporizada(150);
	  			  HAL_Delay(1000);
	  			  motor_parar(false);
	  			  mapa_resetar_para_corrida();
	  			  // +++ RESETAR A VELOCIDADE MÁXIMA ANTES DA CORRIDA +++
	  			  g_velocidade_maxima_registrada_mmps = 0.0f;
	  			  g_dist_entrada_segmento_pulsos = 0;
	  			  g_fim_corrida = 0;
	  			  if (mapa_segmento_atual_e_reta_longa()) {
					  g_estado_robo = ESTADO_ACELERANDO_RETA;
				  } else {
					  g_estado_robo = ESTADO_CORRIDA; // Caso contrário, começa em modo Corrida
				  }
	  			  break;

	  		  case ESTADO_CORRIDA:
	  			  break;
	  		  case ESTADO_ACELERANDO_RETA:
	  			  break;
	  		  case ESTADO_FRENAGEM:
	  			  break;

	  		  case ESTADO_FORA_DA_PISTA:
					motor_parar(true); // Mantém os motores parados
					printf("ROBO PERDEU A LINHA!\r\n");
					iu_buzina_temporizada(200);
					iu_atualizar();
					iu_aguardar_botao();
					HAL_Delay(400);
					//Reseta a um estado inicial seguro
					g_estado_robo = ESTADO_INICIO;

			  break;
	  		  case ESTADO_FINALIZADO:
				   g_estado_robo = ESTADO_DESACELERANDO_FINAL;
				  break;

	  		  case ESTADO_DESACELERANDO_FINAL:
				// A lógica de controle de movimento para este estado
				// será tratada no loop rápido.
				// Aqui, apenas aguardamos a transição para o próximo estado,
				// que será feita pelo loop rápido quando a velocidade for zero.
				break;

				  // +++ LÓGICA PARA OS NOVOS ESTADOS +++
			  case ESTADO_AGUARDANDO_LOG_VELOCIDADE:
				  printf("Pressione para ver a velocidade maxima registrada.\r\n");
				  iu_aguardar_botao();
				  g_estado_robo = ESTADO_LOGANDO_VELOCIDADE;
				  break;

			  case ESTADO_LOGANDO_VELOCIDADE:
				  // Imprime o valor da velocidade máxima (convertendo para int para simplicidade)
				  printf("=============================================\r\n");
				  printf("Velocidade Maxima Atingida: %d mm/s\r\n", (int)g_velocidade_maxima_registrada_mmps);
				  printf("=============================================\r\n");
				  iu_buzina_temporizada(1000);

				  // Após logar, pode ir para um estado final ou reiniciar
				  printf("Pressione o botao para reiniciar.\r\n");
				  iu_aguardar_botao();
				  g_estado_robo = ESTADO_INICIO; // Ou outro estado desejado
				  break;

	  		  case ESTADO_ERRO:
	  			   motor_parar(true);
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
  hadc1.Init.NbrOfConversion = 8;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 8;
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
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  htim8.Init.Period = PERIODO_PWM_MOTOR;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOB, AIN2_Pin|AIN1_Pin|STBY_Pin|BIN1_Pin
                          |BIN2_Pin|BUZINA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2VENT_Pin|AIN1VENT_Pin|STBYVENT_Pin|BIN1VENT_Pin
                          |BIN2VENT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOTAO_Pin */
  GPIO_InitStruct.Pin = BOTAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOTAO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin STBY_Pin BIN1_Pin
                           BIN2_Pin BUZINA_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|STBY_Pin|BIN1_Pin
                          |BIN2_Pin|BUZINA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2VENT_Pin AIN1VENT_Pin STBYVENT_Pin BIN1VENT_Pin
                           BIN2VENT_Pin */
  GPIO_InitStruct.Pin = AIN2VENT_Pin|AIN1VENT_Pin|STBYVENT_Pin|BIN1VENT_Pin
                          |BIN2VENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK_Pin */
  GPIO_InitStruct.Pin = SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MISO_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = MISO_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void executar_loop_rapido(int pos_mapeada) {

	velocidade_atualizar();

	// +++ REGISTRO DA VELOCIDADE MÁXIMA +++
	// Apenas registra quando estamos no modo de aceleração em reta
	if (g_estado_robo == ESTADO_ACELERANDO_RETA) {
		float velocidade_atual_mmps = velocidade_obter_media_mmps();
		if (velocidade_atual_mmps > g_velocidade_maxima_registrada_mmps) {
			g_velocidade_maxima_registrada_mmps = velocidade_atual_mmps;
		}
	}

    int correcao_direcao  = 0;
    float velocidade_alvo_mmps = 0;

    // ESTADO DE MAPEAMENTO
    if (g_estado_robo ==  ESTADO_MAPEAMENTO ) {
    	velocidade_alvo_mmps  = VELOCIDADE_MAPEAMENTO_MMPS;
    	correcao_direcao  = pid_calcular_correcao(pos_mapeada);

    }

    // ESTADO DE CORRIDA
	else if (g_estado_robo == ESTADO_CORRIDA) {
		velocidade_alvo_mmps  = VELOCIDADE_CORRIDA_MMPS;
		correcao_direcao = pid_calcular_correcao(pos_mapeada);
	}

    // ESTADO DE ACELERAÇÃO EM RETA
	else if (g_estado_robo == ESTADO_ACELERANDO_RETA) {
		float velocidade_atual_mmps = velocidade_obter_media_mmps();
		if (velocidade_atual_mmps > g_velocidade_maxima_registrada_mmps) {
			g_velocidade_maxima_registrada_mmps = velocidade_atual_mmps;
		}


		// Define a potência alvo para máxima de reta
		velocidade_alvo_mmps  = VELOCIDADE_RETA_MMPS;
		// Usa o PID específico para retas
		correcao_direcao = pid_calcular_correcao_customizada(pos_mapeada, PID_KP_RETA, PID_KD_RETA);

		// Calcula a distância percorrida no segmento atual
		int32_t dist_total_segmento_pulsos = mapa_obter_distancia_pulsos_segmento_atual();
		int32_t limiar_aceleracao_pulsos = (dist_total_segmento_pulsos * PORCENTAGEM_PULSOS_EM_VRETA) / 100;
		int32_t dist_percorrida_no_segmento_pulsos = ((encoder_obter_posicao_esquerda() + encoder_obter_posicao_direita()) / 2) - g_dist_entrada_segmento_pulsos;

		// Verifica se já percorreu a distância de aceleração
		if (dist_percorrida_no_segmento_pulsos >= limiar_aceleracao_pulsos) {
			 // Distância percorrida, hora de frear.
			 if (!g_frenagem_realizada_no_segmento) {
				 g_frenagem_realizada_no_segmento = true; // Marca que a frenagem foi feita
				 g_estado_robo = ESTADO_FRENAGEM;
				 g_contador_frenagem_ms = TEMPO_FRENAGEM_MS; // Define o tempo
				 motor_frear();
				 return; // Sai do loop para se dedicar à frenagem
			 }
		}
		// Se ainda não atingiu o limiar, continua acelerando (a lógica de potência abaixo cuidará disso)
	}

	// ESTADO DE FRENAGEM: Lógica para controlar a duração da frenagem
	else if (g_estado_robo == ESTADO_FRENAGEM) {
	   motor_frear(); // Mantém os freios acionados
			  if (g_contador_frenagem_ms <= 0) {
				  // Tempo de frenagem acabou, volta para o estado de corrida normal
				  g_estado_robo = ESTADO_CORRIDA;
			  } else {
				  // Se ainda está freando, não faz mais nada neste ciclo
				  return;
			  }
	}
    // +++ LÓGICA PARA O NOVO ESTADO DE DESACELERAÇÃO FINAL +++
	else if (g_estado_robo == ESTADO_DESACELERANDO_FINAL) {
	        // 1. Define o alvo de velocidade como ZERO.
	        velocidade_alvo_mmps = 0.0f;

	        // 2. Ignora a correção de direção para parar reto.
	        correcao_direcao = 0;

	        // 3. Verifica se o robô já está praticamente parado.
	        if (velocidade_obter_media_mmps() < 5.0f) {
	            motor_parar(true); // Desliga os motores completamente
	            iu_buzina_temporizada(1000); // Emite o som de finalização

	            // Transiciona para o próximo estado
	            g_estado_robo = ESTADO_AGUARDANDO_LOG_VELOCIDADE;

	            // Importante: retorna para não aplicar mais potência neste ciclo.
	            return;
	        }
	    }

    // --- NOVO Controle de Potência (Malha Fechada) ---
    if (g_estado_robo != ESTADO_FRENAGEM) {
    	// 1. Calcula a potência base necessária para atingir a velocidade alvo
		int potencia_base = velocidade_controlar_potencia(velocidade_alvo_mmps);

        motor_definir_potencia(potencia_base - correcao_direcao, potencia_base + correcao_direcao);
    }
}


// Loop de 10ms
void executar_loop_lento(void) {
    // Contém a verificação de marcadores e outras tarefas não críticas
    TipoMarcador marcador = marcador_lateral_verificar();
    if (marcador == MARCADOR_NENHUM) {
        return;
    }

    // A lógica de Mapeamento
    if (g_estado_robo == ESTADO_MAPEAMENTO) {
        if (marcador == MARCADOR_ESQUERDA || marcador == MARCADOR_DIREITA) {
             mapa_gravar_segmento();
            if (marcador == MARCADOR_DIREITA) {
            	iu_buzina_temporizada(100);
            	g_fim_mapeamento++;
            }
            if (marcador == MARCADOR_ESQUERDA) {
            	iu_buzina_temporizada(30);
            }
        } else if (marcador == MARCADOR_AMBOS) {
            //iu_buzina_temporizada(80);
        }
    }
    else if (g_estado_robo == ESTADO_CORRIDA || g_estado_robo == ESTADO_ACELERANDO_RETA) {
    	if (marcador == MARCADOR_ESQUERDA || marcador == MARCADOR_DIREITA) {
			iu_buzina_temporizada(30);
			// Antes de avançar para o próximo segmento, salve a distância atual
			g_dist_entrada_segmento_pulsos = (encoder_obter_posicao_esquerda() + encoder_obter_posicao_direita()) / 2;
			mapa_avancar_segmento();
			 // Ao avançar para um novo segmento, permite que a frenagem ocorra novamente.
			g_frenagem_realizada_no_segmento = false;

			if (marcador == MARCADOR_DIREITA) {
				iu_buzina_temporizada(100);
				g_fim_corrida++;
			}

			// Transição para o próximo estado correto
			if (mapa_segmento_atual_e_reta_longa()) {
				g_estado_robo = ESTADO_ACELERANDO_RETA;
			} else {
				g_estado_robo = ESTADO_CORRIDA;
			}

			// A verificação de fim de corrida é feita aqui mesmo
            if (mapa_corrida_terminou(g_fim_corrida)) {
            	motor_parar(true); // Para os motores ao cruzar a linha final
                g_estado_robo = ESTADO_FINALIZADO;
            }


        } else if (marcador == MARCADOR_AMBOS) {
            //iu_buzina_temporizada(100);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		// A interrupção agora acontece a cada 1ms

		// 1. Aciona o loop rápido toda vez
		g_flag_loop_1ms = true;


		// 2. Decrementa o timer de frenagem se ele estiver ativo
		   if (g_contador_frenagem_ms > 0) {
			   g_contador_frenagem_ms--;
		   }

		/*3. O timer para lógica de reconhecimento de linha perdida
		   no arquivo sensor_linha.c*/
		   sensor_linha_tick_1ms(); // Chama a função de tick do módulo do sensor

		// 4. Usa um contador para acionar o loop lento a cada 10ms
		g_contador_loop_lento++;
		if (g_contador_loop_lento >= 10) {
		  g_flag_loop_10ms = true;
		  g_contador_loop_lento = 0;
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
