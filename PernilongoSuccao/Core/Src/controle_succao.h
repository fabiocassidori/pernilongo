/*
 * controle_succao.h
 *
 * Controle do ventilador de sucção: Q3 (IRLML6344) acionado por
 * PA15 = TIM2_CH1 (PWM_VACUO), com diodo de roda livre D7.
 *
 * Módulo novo: o código anterior configurava pinos de ventilador num segundo
 * TB6612, mas nunca os acionava.
 */

#ifndef INC_CONTROLE_SUCCAO_H_
#define INC_CONTROLE_SUCCAO_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void succao_inicializar(TIM_HandleTypeDef *htim_pwm);
/** Define a potência-alvo (0..100 %). A potência sobe em rampa via succao_atualizar(). */
void succao_definir_potencia(uint8_t potencia_pct);
/** Desliga imediatamente (sem rampa). */
void succao_desligar(void);
/** Avança a rampa. Chamar a cada 10 ms (loop lento). */
void succao_atualizar(void);
/** true quando a potência atual chegou ao alvo. */
bool succao_estabilizada(void);

#endif /* INC_CONTROLE_SUCCAO_H_ */
