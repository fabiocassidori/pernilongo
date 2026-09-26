/*
 * leitor_encoder.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */


#include "leitor_encoder.h"

static TIM_HandleTypeDef *g_htim_esq = NULL;
static TIM_HandleTypeDef *g_htim_dir = NULL;

static volatile uint16_t g_ultima_contagem_hw_esq = 0;
static volatile uint16_t g_ultima_contagem_hw_dir = 0;
static volatile int32_t g_posicao_esq = 0;
static volatile int32_t g_posicao_dir = 0;

void encoder_inicializar(TIM_HandleTypeDef *htim_esq, TIM_HandleTypeDef *htim_dir) {
    g_htim_esq = htim_esq;
    g_htim_dir = htim_dir;
    HAL_TIM_Encoder_Start(g_htim_esq, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(g_htim_dir, TIM_CHANNEL_ALL);
    encoder_resetar_posicoes();
}

void encoder_atualizar_posicoes(void) {
    uint16_t contagem_atual_dir = __HAL_TIM_GET_COUNTER(g_htim_dir);
    int16_t delta_dir = (int16_t)(contagem_atual_dir - g_ultima_contagem_hw_dir);
    g_posicao_dir += delta_dir;
    g_ultima_contagem_hw_dir = contagem_atual_dir;

    uint16_t contagem_atual_esq = __HAL_TIM_GET_COUNTER(g_htim_esq);
    int16_t delta_esq = (int16_t)(contagem_atual_esq - g_ultima_contagem_hw_esq);
    g_posicao_esq += delta_esq;
    g_ultima_contagem_hw_esq = contagem_atual_esq;
}

int32_t encoder_obter_posicao_esquerda(void) {
    return g_posicao_esq;
}

int32_t encoder_obter_posicao_direita(void) {
    return g_posicao_dir;
}

void encoder_resetar_posicoes(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_posicao_esq = 0;
    g_ultima_contagem_hw_esq = __HAL_TIM_GET_COUNTER(g_htim_esq);
    g_posicao_dir = 0;
    g_ultima_contagem_hw_dir = __HAL_TIM_GET_COUNTER(g_htim_dir);
    if (!primask) {
        __enable_irq();
    }
}
