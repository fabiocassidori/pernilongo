/*
 * robo.h
 *
 * Maquina de estados do seguidor de linha (robotrace) Pernilongo 5.
 *
 *  AGUARDA_CALIB --botao--> CALIBRANDO --ok--> PRONTO --botao--> LARGADA --> CORRIDA
 *        ^                     |falha                ^  (longo: +5 vel.)       |
 *        |                     v                     |                         | 2a marca direita
 *        +---------------- CALIB_FALHOU               +---- FIM <-- FRENANDO <--+ ou botao
 *                                                     +---- FORA_PISTA <-- perdeu a linha
 */

#ifndef INC_ROBO_H_
#define INC_ROBO_H_

#include "stm32f4xx_hal.h"

typedef enum {
    ROBO_AGUARDA_CALIB = 0,
    ROBO_CALIBRANDO,
    ROBO_CALIB_FALHOU,
    ROBO_PRONTO,
    ROBO_LARGADA,
    ROBO_CORRIDA,
    ROBO_FRENANDO,
    ROBO_FIM,
    ROBO_FORA_PISTA
} estado_robo_t;

void          robo_inicializar(ADC_HandleTypeDef *hadc_linha, TIM_HandleTypeDef *htim_motor,
                               TIM_HandleTypeDef *htim_succao, UART_HandleTypeDef *huart_bt);
/* Chamar no HAL_TIM_PeriodElapsedCallback do TIM6 (1 kHz). */
void          robo_tick_1ms_isr(void);
/* Chamar sem parar dentro do while(1). Nunca bloqueia (exceto ~10 ms na calibracao). */
void          robo_executar(void);
estado_robo_t robo_estado(void);
/* Desliga motores e succao na hora (Error_Handler, falhas). */
void          robo_desligar_tudo(void);

#endif /* INC_ROBO_H_ */
