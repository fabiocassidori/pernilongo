/*
 * config_robo.h
 *
 * Created on: Jun 5, 2025
 * Author: Fabio Couto
 */

#ifndef INC_CONFIG_ROBO_H_
#define INC_CONFIG_ROBO_H_

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURAÇÕES GERAIS E FÍSICAS
// =============================================================================
#define NUM_SENSORES_LINHA 6
/* Quantidade de leituras para calibrar o sensor de linha
  Para achar o tempo em milissegungos é só multiplicar por CICLOS_CALIBRACAO x DELAY_CALIBRACAO
 */
#define CICLOS_CALIBRACAO 100
#define DELAY_CALIBRACAO 50
#define INTERVALO_LOOP_CONTROLE_S 0.001f  // Intervalo em segundos

// --- Física do Robô ---
#define DIAMETRO_RODA_MM 22.2f
#define CONSTANTE_PI 3.1415926535f
#define ENCODER_PPR 120              // Pulsos por revolução do encoder
// Converte pulsos do encoder para milímetros. (DIAMETRO_RODA_MM * PI) / ENCODER_PPR
#define PULSOS_PARA_MM ((DIAMETRO_RODA_MM * CONSTANTE_PI) / ENCODER_PPR)

// =============================================================================
// CONFIGURAÇÕES DOS SENSORES
// =============================================================================
#define ADC_VALOR_MAXIMO 1023.0f
//TRATAMENTO DE VALORES
#define SENSOR_POSICAO_MIN   0 // EXTREMA DIREITA DO SENSOR DE LINHA
#define SENSOR_POSICAO_MAX   5000 // EXTREMA ESQUERDA DO SENSOR DE LINHA
#define POSICAO_ENTRADA_MIN  -255 //INTERPOLACAO - MAXIMA ESQUERDA
#define POSICAO_ENTRADA_MAX  255 //INTERPOLACAO - MAXIMA DIREITA

// --- Sensores Laterais (Marcadores) ---
 /* LIMIAR_SENSOR_LATERAL só é usado quando há falha na calibração.
 * calibrando os sensores laterais, usa-se valor mínimo + (valor maximo - valor minimo)/2
 */
#define LIMIAR_SENSOR_LATERAL 450    // Valor ADC para considerar uma marcação detectada

// =============================================================================
// CONFIGURAÇÕES DO CONTROLE E MOTORES
// =============================================================================

// +++ CONSTANTES DE VELOCIDADE ALVO (em mm/s) +++
#define VELOCIDADE_MAPEAMENTO_MMPS 1000
#define VELOCIDADE_RETA_MMPS 2900 //2905 VELOCIDADE MÁXIMA COM 8.23V BATERIA
#define VELOCIDADE_CORRIDA_MMPS 1200

#define POTENCIA_MAX_MOTOR 255
#define PERIODO_PWM_MOTOR 3359

// --- PID MAPEAMENTO/CURVAS ---
#define PID_SETPOINT 0
#define PID_KP 310 //270//310
#define PID_KD 2900 //2500 //2900
#define PID_KI 0

// ---  PID EM RETAS ---
#define PID_KP_RETA 190
#define PID_KD_RETA 4500

// +++ GANHOS PARA O PID DE VELOCIDADE +++
#define PID_VEL_KP 100   // Valor inicial, necessita de ajuste
#define PID_VEL_KI 20    // Valor inicial, necessita de ajuste
#define PID_VEL_KD 0     // Geralmente não é necessário para velocidade
#define PID_VEL_DIVISOR 1000
#define PID_VEL_LIMITE_INTEGRAL 5000 // Limita o termo integral


#define PID_DIVISOR 100
/*
 * Se PID_LIMITE_SAIDA = 150, CORREÇAO MÁXIMA É IGUAL A 150
 * LOGO SE V_CURVA É 80 -> 80 + 150 = POTENCIA MAXIMA É 230,
 * OU SEJA, NAO SATURA OS 8BITS.
 * EM VRETA NÃO TEM PROBLEMA SATURAR
 */
#define PID_LIMITE_SAIDA 150
// =============================================================================
// CONFIGURAÇÕES DO MAPEAMENTO
// =============================================================================
#define TAMANHO_MAPA 50
// Distância (em mm) a partir da qual uma reta é considerada "longa"
// o suficiente para aplicar potência máxima.
#define LIMIAR_DISTANCIA_RETA_LONGA 800
// Raio calculado a partir do qual se considera uma reta
#define LIMIAR_CURVA_RETA 70
#define PORCENTAGEM_PULSOS_EM_VRETA 75
#define TEMPO_FRENAGEM_MS 50
#define TEMPO_MAX_LINHA_PERDIDA_MS 100
#define POS_LINHA_PERDIDA 250

// =============================================================================
// CONFIGURAÇÕES DA BATERIA
// =============================================================================
#define BATERIA_ADC_TENSAO_REF 3.3f
#define BATERIA_R1_DIV 10000.0f
#define BATERIA_R2_DIV 33000.0f
#define BATERIA_RELACAO_DIVISOR (BATERIA_R1_DIV / (BATERIA_R1_DIV + BATERIA_R2_DIV))
#define BATERIA_MULTIPLICADOR_TENSAO (1.0f / BATERIA_RELACAO_DIVISOR)
#define LIMIAR_BATERIA_FRACA_VOLTS 6.80f // Para 2S. Mudar para ~10.5V para 3S.
#define BATERIA_FILTRO_ALPHA 0.1f
#define BATERIA_INTERVALO_MONITOR_MS 200

#endif
