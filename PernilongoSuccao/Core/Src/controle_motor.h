/*
 * controle_motor.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 *
 * Adaptado para 2x IFX9201SG (controle PWM/DIR). A interface pública é a
 * mesma do driver TB6612, para manter a lógica do main.c.
 */

#ifndef INC_CONTROLE_MOTOR_H_
#define INC_CONTROLE_MOTOR_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void motor_inicializar(TIM_HandleTypeDef *htim_pwm);
/** parar=true: DIS alto (pontes desligadas, saídas em tristate).
 *  parar=false: DIS baixo (pontes habilitadas). */
void motor_parar(bool parar);
/** Potências de -POTENCIA_MAX_MOTOR a +POTENCIA_MAX_MOTOR (negativo = ré). */
void motor_definir_potencia(int potencia_esquerda, int potencia_direita);
/** Frenagem por contra-corrente (o IFX9201SG não tem freio ativo em PWM/DIR).
 *  Deve ser mantida por TEMPO_FRENAGEM_MS e depois seguida de motor_parar(true). */
void motor_frear(void);

#endif /* INC_CONTROLE_MOTOR_H_ */
