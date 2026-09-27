/*
 * controle_motor.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 *
 * Adaptado de TB6612FNG para 2x IFX9201SG.
 *
 * Tabela-verdade do IFX9201SG (datasheet Rev. 1.1, Tabela 4-1):
 *   DIS PWM DIR | OUT1 OUT2
 *    1   X   X  |  Z    Z    desligado (tristate)
 *    0   1   1  |  H    L    frente
 *    0   1   0  |  L    H    ré
 *    0   0   1  |  H    Z    roda livre pelo high-side
 *    0   0   0  |  Z    H    roda livre pelo high-side
 *
 * Diferenças em relação ao TB6612:
 *  - Direção por UM pino (DIR), não dois (IN1/IN2).
 *  - DIS é ativo em ALTO (desliga). O STBY do TB6612 era ativo em BAIXO.
 *    DIS tem pull-up interno: a ponte já nasce desligada.
 *  - Não existe freio ativo (curto nos terminais). Frenagem = contra-corrente.
 *
 * Ligações na placa:
 *   U3 (lado DIREITO): PWMA = PC6 = TIM8_CH1, DIRA = PB12, saída J3
 *   U2 (lado ESQUERDO): PWMB = PC7 = TIM8_CH2, DIRB = PB14, saída J6
 *   DIS = PB13, comum aos dois
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
  #define ESQ_CANAL      TIM_CHANNEL_2   // U2 / J6
  #define ESQ_DIR_PORT   DIRB_GPIO_Port
  #define ESQ_DIR_PIN    DIRB_Pin
  #define DIR_CANAL      TIM_CHANNEL_1   // U3 / J3
  #define DIR_DIR_PORT   DIRA_GPIO_Port
  #define DIR_DIR_PIN    DIRA_Pin
#endif

static TIM_HandleTypeDef *g_htim_pwm = NULL;

static uint16_t interpolar(int valor, int entrada_min, int entrada_max, int saida_min, int saida_max);
static void motor_aplicar(uint32_t canal, GPIO_TypeDef *porta_dir, uint16_t pino_dir,
                          uint8_t nivel_frente, int potencia);

void motor_inicializar(TIM_HandleTypeDef *htim_pwm) {
    g_htim_pwm = htim_pwm;
    motor_parar(true);                       // Começa desligado (DIS alto)
    __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(g_htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim_pwm, TIM_CHANNEL_2);
}

void motor_parar(bool parar) {
    if (parar) {
        // Zera o PWM antes de desabilitar, para a ponte não religar com carga ao habilitar.
        if (g_htim_pwm) {
            __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_2, 0);
        }
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);    // DIS=1: tristate
    } else {
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_RESET);  // DIS=0: habilitado
    }
}

void motor_definir_potencia(int potencia_esquerda, int potencia_direita) {

    motor_aplicar(ESQ_CANAL, ESQ_DIR_PORT, ESQ_DIR_PIN, MOTOR_ESQ_DIR_FRENTE, potencia_esquerda);
    motor_aplicar(DIR_CANAL, DIR_DIR_PORT, DIR_DIR_PIN, MOTOR_DIR_DIR_FRENTE, potencia_direita);
}

static uint16_t interpolar(int valor, int entrada_min, int entrada_max, int saida_min, int saida_max) {
    if (entrada_min == entrada_max) return saida_min;
    return (uint16_t)(((int32_t)(valor - entrada_min) * (saida_max - saida_min)) / (entrada_max - entrada_min) + saida_min);
}

/*
 * Mesma lógica do TB6612: sinal da potência -> direção, módulo -> duty.
 * Aqui a direção é um único pino DIR, com o nível de "frente" configurável
 * por lado (os motores são montados espelhados).
 */
static void motor_aplicar(uint32_t canal, GPIO_TypeDef *porta_dir, uint16_t pino_dir,
                          uint8_t nivel_frente, int potencia) {
    uint8_t nivel = (potencia > 0) ? nivel_frente : (uint8_t)!nivel_frente;
    HAL_GPIO_WritePin(porta_dir, pino_dir, nivel ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int potencia_abs = abs(potencia);
    if (potencia_abs > POTENCIA_MAX_MOTOR) potencia_abs = POTENCIA_MAX_MOTOR;
    uint16_t valor_pwm = interpolar(potencia_abs, 0, POTENCIA_MAX_MOTOR, 0, PERIODO_PWM_MOTOR);
    __HAL_TIM_SET_COMPARE(g_htim_pwm, canal, valor_pwm);
}

void motor_frear(void) {
    // O IFX9201SG não tem freio ativo (com PWM=0 a ponte fica em roda livre).
    // Frenagem por contra-corrente: aciona os dois motores para trás.
    // Usar por pouco tempo (TEMPO_FRENAGEM_MS) para não inverter o movimento.
    HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_RESET);
    motor_definir_potencia(-POTENCIA_FRENAGEM, -POTENCIA_FRENAGEM);
}
