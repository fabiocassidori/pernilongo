/*
 * controle_motor.c
 *
 * Tabela-verdade do IFX9201SG (datasheet Rev. 1.1, Tabela 4-1):
 *   DIS PWM DIR | OUT1 OUT2
 *    1   X   X  |  Z    Z    desligado (tristate)
 *    0   1   1  |  H    L    frente
 *    0   1   0  |  L    H    re
 *    0   0   X  |  roda livre
 * Nao existe freio ativo: a frenagem e por contra-corrente (potencia negativa).
 *
 * Ligacoes: U3 (lado DIREITO) PWMA = PC6 = TIM8_CH1, DIRA = PB12
 *           U2 (lado ESQUERDO) PWMB = PC7 = TIM8_CH2, DIRB = PB14
 *           DIS = PB13 (comum)
 */

#include "controle_motor.h"
#include "config_robo.h"
#include "main.h"
#include <stdlib.h>

#if MOTOR_TROCAR_LADOS
  #define ESQ_CANAL      TIM_CHANNEL_1
  #define ESQ_DIR_PORT   DIRA_GPIO_Port
  #define ESQ_DIR_PIN    DIRA_Pin
  #define DIR_CANAL      TIM_CHANNEL_2
  #define DIR_DIR_PORT   DIRB_GPIO_Port
  #define DIR_DIR_PIN    DIRB_Pin
#else
  #define ESQ_CANAL      TIM_CHANNEL_2   /* U2 / J6 */
  #define ESQ_DIR_PORT   DIRB_GPIO_Port
  #define ESQ_DIR_PIN    DIRB_Pin
  #define DIR_CANAL      TIM_CHANNEL_1   /* U3 / J3 */
  #define DIR_DIR_PORT   DIRA_GPIO_Port
  #define DIR_DIR_PIN    DIRA_Pin
#endif

static TIM_HandleTypeDef *g_htim = NULL;

/* Visiveis no Live Expressions */
volatile int16_t motor_esq_live = 0, motor_dir_live = 0;

void motor_inicializar(TIM_HandleTypeDef *htim_pwm)
{
    g_htim = htim_pwm;
    motor_parar(true);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_2);
}

void motor_parar(bool parar)
{
    if (parar) {
        if (g_htim) {
            __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_2, 0);
        }
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);
        motor_esq_live = motor_dir_live = 0;
    } else {
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_RESET);
    }
}

static void aplicar(uint32_t canal, GPIO_TypeDef *porta, uint16_t pino, uint8_t nivel_frente, int pot)
{
    uint8_t nivel = (pot >= 0) ? nivel_frente : (uint8_t)!nivel_frente;
    HAL_GPIO_WritePin(porta, pino, nivel ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int a = abs(pot);
    if (a > POTENCIA_MAX_MOTOR) a = POTENCIA_MAX_MOTOR;
    uint32_t ccr = ((uint32_t)a * (PERIODO_PWM_MOTOR + 1U)) / POTENCIA_MAX_MOTOR;
    if (ccr > PERIODO_PWM_MOTOR + 1U) ccr = PERIODO_PWM_MOTOR + 1U;
    __HAL_TIM_SET_COMPARE(g_htim, canal, ccr);
}

void motor_definir_potencia(int esq, int dir)
{
    if (!g_htim) return;
    if (esq >  POTENCIA_MAX_MOTOR) esq =  POTENCIA_MAX_MOTOR;
    if (esq < -POTENCIA_MAX_MOTOR) esq = -POTENCIA_MAX_MOTOR;
    if (dir >  POTENCIA_MAX_MOTOR) dir =  POTENCIA_MAX_MOTOR;
    if (dir < -POTENCIA_MAX_MOTOR) dir = -POTENCIA_MAX_MOTOR;
    aplicar(ESQ_CANAL, ESQ_DIR_PORT, ESQ_DIR_PIN, MOTOR_ESQ_DIR_FRENTE, esq);
    aplicar(DIR_CANAL, DIR_DIR_PORT, DIR_DIR_PIN, MOTOR_DIR_DIR_FRENTE, dir);
    motor_esq_live = (int16_t)esq;
    motor_dir_live = (int16_t)dir;
}
