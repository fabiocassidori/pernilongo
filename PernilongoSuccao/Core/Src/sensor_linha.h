/*
 * sensor_linha.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_SENSOR_LINHA_H_
#define INC_SENSOR_LINHA_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void sensor_linha_inicializar(ADC_HandleTypeDef *hadc);
void sensor_linha_calibrar(void);
int sensor_linha_ler_posicao(void);
volatile uint16_t* sensor_linha_obter_buffer_dma(void);

/**
 * @brief Liga/desliga os emissores IR da régua (Q5, pino PA8 = PWM_SENSOR).
 * Na placa atual os LEDs da régua só acendem por este pino.
 */
void sensor_linha_definir_emissores(bool ligar);

/**
 * @brief Atualiza o estado interno do contador de linha perdida.
 * Deve ser chamada a cada 1ms por uma interrupção de timer.
 */
void sensor_linha_tick_1ms(void);

/**
 * @brief Verifica, com base na posição mapeada, se o robô perdeu a linha.
 * Esta função gerencia um contador interno para evitar falsos positivos.
 * @param pos_mapeada A posição da linha, já mapeada (ex: -255 a 255).
 * @return true se o tempo limite fora da linha foi atingido, false caso contrário.
 */
bool sensor_linha_is_robo_fora_da_pista(int pos_mapeada);

/**
 * @brief Imprime os valores brutos de todos os canais (depuração de bancada).
 */
void sensor_linha_depuracao(void);

/**
 * @brief Imprime, para cada QTR, o valor com LED aceso, apagado e a diferença
 * usada na leitura (aceso-apagado, cancelando luz ambiente). Só tem efeito
 * com SENSOR_PULSADO=1. Útil para calibrar PULSO_TEMPO_ESTAB_US na bancada.
 */
void sensor_linha_depuracao_pulso(void);

#endif /* INC_SENSOR_LINHA_H_ */
