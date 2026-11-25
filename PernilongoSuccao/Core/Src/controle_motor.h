/*
 * controle_motor.h
 *
 * 2x IFX9201SG (PWM/DIR), TIM8 a 20 kHz. Mesmo modulo validado no teste
 * motores/main.c. Potencia de -255 a +255 (negativo = re).
 */

#ifndef INC_CONTROLE_MOTOR_H_
#define INC_CONTROLE_MOTOR_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void motor_inicializar(TIM_HandleTypeDef *htim_pwm);
/* parar=true: PWM 0 e DIS = 1 (pontes em tristate). parar=false: DIS = 0. */
void motor_parar(bool parar);
void motor_definir_potencia(int potencia_esquerda, int potencia_direita);

#endif /* INC_CONTROLE_MOTOR_H_ */
