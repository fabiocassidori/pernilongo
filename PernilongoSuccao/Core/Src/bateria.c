/*
 * bateria.c
 *
 * Vbat = adc / 4095 x VDDA x (R22 + R23) / R23
 * Com R22 = 15k e R23 = 10k: Vbat = adc x 3,3 / 4095 x 2,5 (fundo de escala 8,25 V).
 * Com R22 = 18k (recomendado):  Vbat = adc x 3,3 / 4095 x 2,8 (fundo de escala 9,24 V).
 */

#include "bateria.h"
#include "config_robo.h"
#include "main.h"

#define BAT_FATOR   (BAT_VDDA / 4095.0f * (BAT_R_SUPERIOR_OHM + BAT_R_INFERIOR_OHM) / BAT_R_INFERIOR_OHM)
#define BAT_V_MINIMA_PRESENTE 3.0f    /* abaixo disso nao ha bateria (so USB/ST-LINK) */

static ADC_HandleTypeDef g_hadc2;
static bool     g_ok = false;
static bool     g_primeira = true;
static float    g_v_rapida = 0.0f, g_v_lenta = 0.0f;
static uint16_t g_adc = 0;
static uint32_t g_ms_abaixo = 0;
static bool     g_baixa = false;

/* Visiveis no Live Expressions */
volatile float bat_tensao_live = 0.0f;
volatile uint8_t bat_baixa_live = 0;

void bateria_inicializar(void)
{
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef canal = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    gpio.Pin  = GPIO_PIN_1;            /* PC1 = BAT_REF */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    g_hadc2.Instance = ADC2;
    g_hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* comum: igual ao ADC1 */
    g_hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    g_hadc2.Init.ScanConvMode = DISABLE;
    g_hadc2.Init.ContinuousConvMode = DISABLE;
    g_hadc2.Init.DiscontinuousConvMode = DISABLE;
    g_hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_hadc2.Init.NbrOfConversion = 1;
    g_hadc2.Init.DMAContinuousRequests = DISABLE;
    g_hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&g_hadc2) != HAL_OK) return;

    canal.Channel = ADC_CHANNEL_11;
    canal.Rank = 1;
    canal.SamplingTime = ADC_SAMPLETIME_480CYCLES;  /* 23 us: folga para o divisor sem capacitor */
    if (HAL_ADC_ConfigChannel(&g_hadc2, &canal) != HAL_OK) return;

    g_ok = (HAL_ADC_Start(&g_hadc2) == HAL_OK);
}

void bateria_tick_10ms(bool em_corrida)
{
    if (!g_ok) return;
    if (!__HAL_ADC_GET_FLAG(&g_hadc2, ADC_FLAG_EOC)) {
        HAL_ADC_Start(&g_hadc2);          /* conversao perdida: dispara de novo */
        return;
    }

    g_adc = (uint16_t)HAL_ADC_GetValue(&g_hadc2);   /* le e limpa o EOC */
    HAL_ADC_Start(&g_hadc2);                         /* a proxima fica pronta em 25 us */

    float v = (float)g_adc * BAT_FATOR;
    if (g_primeira) {
        g_v_rapida = g_v_lenta = v;
        g_primeira = false;
    }
    g_v_rapida += (v - g_v_rapida) / 8.0f;    /* tau = 8 x 10 ms  = 80 ms  */
    g_v_lenta  += (v - g_v_lenta)  / 64.0f;   /* tau = 64 x 10 ms = 640 ms */

    float limiar = em_corrida ? BAT_LIMIAR_BAIXA_CORRIDA_V : BAT_LIMIAR_BAIXA_V;
    if (bateria_presente() && g_v_lenta < limiar) {
        g_ms_abaixo += 10U;
        if (g_ms_abaixo >= BAT_TEMPO_CONFIRMA_MS) g_baixa = true;   /* trava */
    } else {
        g_ms_abaixo = 0;
    }

    bat_tensao_live = g_v_lenta;
    bat_baixa_live = g_baixa ? 1U : 0U;
}

float    bateria_tensao(void)       { return g_v_rapida; }
float    bateria_tensao_lenta(void) { return g_v_lenta; }
bool     bateria_presente(void)     { return !g_primeira && g_v_lenta > BAT_V_MINIMA_PRESENTE; }
bool     bateria_baixa(void)        { return g_baixa; }
uint16_t bateria_adc(void)          { return g_adc; }
