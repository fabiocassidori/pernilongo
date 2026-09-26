/*
 * monitor_bateria.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */


// Core/Src/monitor_bateria.c

#include "monitor_bateria.h"
#include "config_robo.h"
#include "interface_usuario.h"

static ADC_HandleTypeDef *g_hadc_bateria = NULL;
static float g_tensao_filtrada = -1.0f; // Inicia como inválido
static uint32_t g_proximo_monitoramento_ms = 0;
static bool g_alerta_bateria_baixa_ativo = false;

static float ler_tensao_instantanea(void);

void bateria_inicializar(ADC_HandleTypeDef *hadc) {
    g_hadc_bateria = hadc;
    HAL_ADC_Start(g_hadc_bateria);
    g_proximo_monitoramento_ms = HAL_GetTick() + BATERIA_INTERVALO_MONITOR_MS;
}

void bateria_atualizar(void) {
    uint32_t tick_atual = HAL_GetTick();
    if (tick_atual < g_proximo_monitoramento_ms) {
        return; // Ainda não é hora
    }
    g_proximo_monitoramento_ms = tick_atual + BATERIA_INTERVALO_MONITOR_MS;

    float tensao_atual = ler_tensao_instantanea();

    // Se a leitura for inválida (próxima de zero) e o filtro ainda não foi inicializado,
    // simplesmente ignora esta medição e aguarda a próxima.
    if (tensao_atual < 1.0f && g_tensao_filtrada < 0.0f) {
        return;
    }

    // Inicializa o filtro na primeira leitura VÁLIDA
    if (g_tensao_filtrada < 0.0f) {
        g_tensao_filtrada = tensao_atual;
    } else {
        // Aplica o filtro de média móvel exponencial normalmente
        g_tensao_filtrada = BATERIA_FILTRO_ALPHA * tensao_atual + (1.0f - BATERIA_FILTRO_ALPHA) * g_tensao_filtrada;
    }

    // Lógica de Alerta
    // Agora que a inicialização é segura, a verificação simples funciona corretamente
    if (g_tensao_filtrada < LIMIAR_BATERIA_FRACA_VOLTS) {
        if (!g_alerta_bateria_baixa_ativo) {
            iu_definir_buzina_persistente(true);
            g_alerta_bateria_baixa_ativo = true;
        }
    } else {
        if (g_alerta_bateria_baixa_ativo) {
            iu_definir_buzina_persistente(false);
            g_alerta_bateria_baixa_ativo = false;
        }
    }
}

float bateria_obter_tensao(void) {
    return g_tensao_filtrada;
}

bool bateria_esta_baixa(void) {
    return g_alerta_bateria_baixa_ativo;
}

static float ler_tensao_instantanea(void) {
    if (!g_hadc_bateria) return 0.0f;
    uint32_t adc_cru = HAL_ADC_GetValue(g_hadc_bateria);
    float tensao_pino = ((float)adc_cru / ADC_VALOR_MAXIMO) * BATERIA_ADC_TENSAO_REF;
    return tensao_pino * BATERIA_MULTIPLICADOR_TENSAO;
}

float bateria_obter_tensao_instantanea(void) {
    // Esta é uma cópia da função estática interna para permitir o debug a partir do main.c
    if (!g_hadc_bateria) return 0.0f;
    uint32_t adc_cru = HAL_ADC_GetValue(g_hadc_bateria);
    float tensao_pino = ((float)adc_cru / ADC_VALOR_MAXIMO) * BATERIA_ADC_TENSAO_REF;
    return tensao_pino * BATERIA_MULTIPLICADOR_TENSAO;
}
