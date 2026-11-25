/*
 * controle_succao.h
 *
 * Ventilador de succao: Q3 (IRLML6344) pelo PA15 = TIM2_CH1 (PWM_VACUO), 20 kHz.
 *
 * SEGURANCA: depois do reset o PA15 e o JTDI e vem com pull-up (~40k). Com o
 * R7 de 47k o gate do Q3 fica em ~1,8 V e o motor LIGA sozinho. Por isso:
 *  - succao_forcar_desligado() poe o PA15 como saida em 0 (com pull-down)
 *    direto nos registradores. Chamar ANTES do HAL_Init e depois das inits.
 *  - O PA15 so volta para AF1 (TIM2) quando a succao e ligada de verdade, e
 *    volta para GPIO em 0 sempre que ela e desligada.
 */

#ifndef INC_CONTROLE_SUCCAO_H_
#define INC_CONTROLE_SUCCAO_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void    succao_forcar_desligado(void);
void    succao_inicializar(TIM_HandleTypeDef *htim_pwm);
/* Potencia-alvo 0..100 % (limitada a SUCCAO_MAX_PCT). Sobe em rampa. */
void    succao_definir_potencia(uint8_t potencia_pct);
void    succao_desligar(void);
/* Chamar a cada 10 ms: rampa + compensacao da bateria. */
void    succao_atualizar(void);
bool    succao_estabilizada(void);
uint8_t succao_potencia_atual(void);
/* Potencia do EVA-RT para uma velocidade base (IMP = 180 + 14 a cada +5). */
uint8_t succao_pct_para_velocidade(int velocidade_base);

#endif /* INC_CONTROLE_SUCCAO_H_ */
