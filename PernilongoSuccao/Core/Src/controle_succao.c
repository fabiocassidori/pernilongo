/*
 * controle_succao.c
 */

#include "controle_succao.h"
#include "config_robo.h"

static TIM_HandleTypeDef *g_htim_succao = NULL;
static volatile uint8_t g_potencia_alvo_pct = 0;
static volatile uint8_t g_potencia_atual_pct = 0;

static void aplicar_pwm(uint8_t pct) {
    if (!g_htim_succao) return;
    uint32_t compare = ((uint32_t)pct * (PERIODO_PWM_SUCCAO + 1U)) / 100U;
    __HAL_TIM_SET_COMPARE(g_htim_succao, TIM_CHANNEL_1, compare);
}

void succao_inicializar(TIM_HandleTypeDef *htim_pwm) {
    g_htim_succao = htim_pwm;
    g_potencia_alvo_pct = 0;
    g_potencia_atual_pct = 0;
    aplicar_pwm(0);
    HAL_TIM_PWM_Start(g_htim_succao, TIM_CHANNEL_1);
}

void succao_definir_potencia(uint8_t potencia_pct) {
#if SUCCAO_HABILITADA
    if (potencia_pct > 100) potencia_pct = 100;
    g_potencia_alvo_pct = potencia_pct;
#else
    (void)potencia_pct;
    g_potencia_alvo_pct = 0;
#endif
}

void succao_desligar(void) {
    g_potencia_alvo_pct = 0;
    g_potencia_atual_pct = 0;
    aplicar_pwm(0);
}

void succao_atualizar(void) {
    uint8_t atual = g_potencia_atual_pct;
    uint8_t alvo = g_potencia_alvo_pct;
    if (atual < alvo) {
        atual = (uint8_t)((alvo - atual > SUCCAO_RAMPA_PASSO_PCT) ? atual + SUCCAO_RAMPA_PASSO_PCT : alvo);
    } else if (atual > alvo) {
        atual = alvo;   // Reduzir é imediato
    }
    g_potencia_atual_pct = atual;
    aplicar_pwm(atual);
}

bool succao_estabilizada(void) {
    return g_potencia_atual_pct == g_potencia_alvo_pct;
}
