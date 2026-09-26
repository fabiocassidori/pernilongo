/*
 * monitor_bateria.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_MONITOR_BATERIA_H_
#define INC_MONITOR_BATERIA_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void bateria_inicializar(ADC_HandleTypeDef *hadc);
void bateria_atualizar(void);
float bateria_obter_tensao(void);
bool bateria_esta_baixa(void);
float bateria_obter_tensao_instantanea(void);

#endif /* INC_MONITOR_BATERIA_H_ */
