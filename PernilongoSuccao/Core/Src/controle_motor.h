/*
 * controle_motor.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_CONTROLE_MOTOR_H_
#define INC_CONTROLE_MOTOR_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void motor_inicializar(TIM_HandleTypeDef *htim_pwm);
void motor_parar(bool parar);
void motor_definir_potencia(int potencia_esquerda, int potencia_direita);
void motor_frear(void);

#endif /* INC_CONTROLE_MOTOR_H_ */
