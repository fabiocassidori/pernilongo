/*
 * leitor_encoder.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_LEITOR_ENCODER_H_
#define INC_LEITOR_ENCODER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

void encoder_inicializar(TIM_HandleTypeDef *htim_esq, TIM_HandleTypeDef *htim_dir);
void encoder_atualizar_posicoes(void);
int32_t encoder_obter_posicao_esquerda(void);
int32_t encoder_obter_posicao_direita(void);
void encoder_resetar_posicoes(void);

#endif /* INC_LEITOR_ENCODER_H_ */
