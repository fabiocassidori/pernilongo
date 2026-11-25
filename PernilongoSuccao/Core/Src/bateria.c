/*
 * bateria.c
 *
 * Vbat = adc / 4095 x VDDA x (R22 + R23) / R23 x BAT_FATOR_AJUSTE
 * Com R22 = 15k e R23 = 10k: Vbat = adc x 3,3 / 4095 x 2,5 (fundo de escala 8,25 V).
 * Com R22 = 18k (recomendado):  Vbat = adc x 3,3 / 4095 x 2,8 (fundo de escala 9,24 V).
 *
 * Valores esperados com 18k / 10k:
 *   Vbat 8,4 V -> pino 3000 mV -> adc 3723
 *   Vbat 7,0 V -> pino 2500 mV -> adc 3102   (limiar de repouso)
 *   Vbat 6,4 V -> pino 2286 mV -> adc 2836   (limiar em corrida)
 *
 * Live Expressions (diagnostico):
 *   bat_adc_bruto    leitura crua do ADC2 (0..4095)
 *   bat_pino_mv      tensao no pino PC1 calculada a partir do bruto
 *   bat_tensao_calc  Vbat da ultima amostra, sem filtro
 *   bat_tensao_live  Vbat filtrada (tau 0,64 s), a que decide o alarme
 *   bat_baixa_live   1 = alarme ativo
 *   bat_saturado     1 = adc no fundo de escala (divisor nao serve para essa bateria)
 *   bat_adc2_reinits quantas vezes a configuracao do ADC2 foi refeita
 */

#include <stdio.h>
#include "bateria.h"
#include "config_robo.h"
#include "main.h"

#define BAT_FATOR   (BAT_VDDA / 4095.0f * (BAT_R_SUPERIOR_OHM + BAT_R_INFERIOR_OHM) / BAT_R_INFERIOR_OHM * BAT_FATOR_AJUSTE)
#define BAT_V_MINIMA_PRESENTE 3.0f    /* abaixo disso nao ha bateria (so USB/ST-LINK) */
#define BAT_ADC_SATURADO      4090U

static ADC_HandleTypeDef g_hadc2;
static bool     g_ok = false;
static bool     g_primeira = true;
static float    g_v_rapida = 0.0f, g_v_lenta = 0.0f;
static uint16_t g_adc = 0;
static uint32_t g_ms_vida = 0;
static uint32_t g_ms_abaixo = 0;
static uint32_t g_ms_acima = 0;
static bool     g_baixa = false;
static float    g_v_disparo = 0.0f;
static uint16_t g_adc_disparo = 0;

/* Visiveis no Live Expressions */
volatile uint16_t bat_adc_bruto = 0;
volatile uint16_t bat_pino_mv = 0;
volatile float    bat_tensao_calc = 0.0f;
volatile float    bat_tensao_live = 0.0f;
volatile uint8_t  bat_baixa_live = 0;
volatile uint8_t  bat_saturado = 0;
volatile uint16_t bat_adc2_reinits = 0;

static bool adc2_configurar(void)
{
    ADC_ChannelConfTypeDef canal = {0};

    g_hadc2.Instance = ADC2;
    g_hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* comum: igual ao ADC1 */
    g_hadc2.Init.Resolution = ADC_RESOLUTION_12B;             /* ADC1 e 10 bits; o ADC2 tem o seu RES */
    g_hadc2.Init.ScanConvMode = DISABLE;
    g_hadc2.Init.ContinuousConvMode = DISABLE;
    g_hadc2.Init.DiscontinuousConvMode = DISABLE;
    g_hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_hadc2.Init.NbrOfConversion = 1;
    g_hadc2.Init.DMAContinuousRequests = DISABLE;
    g_hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&g_hadc2) != HAL_OK) return false;

    canal.Channel = ADC_CHANNEL_11;                 /* PC1 = BAT_REF (pino 9 do LQFP64) */
    canal.Rank = 1;
    canal.SamplingTime = ADC_SAMPLETIME_480CYCLES;  /* 23 us: folga para o divisor sem capacitor */
    if (HAL_ADC_ConfigChannel(&g_hadc2, &canal) != HAL_OK) return false;

    return HAL_ADC_Start(&g_hadc2) == HAL_OK;
}

/* Confere se ninguem mexeu no ADC2 (canal, resolucao, ligado). Se mexeu, refaz. */
static bool adc2_conferir(void)
{
    uint32_t sqr3 = ADC2->SQR3 & ADC_SQR3_SQ1;
    uint32_t res  = ADC2->CR1 & ADC_CR1_RES;
    uint32_t adon = ADC2->CR2 & ADC_CR2_ADON;
    if (sqr3 == 11U && res == ADC_RESOLUTION_12B && adon != 0U) return true;

    bat_adc2_reinits++;
    g_hadc2.State = HAL_ADC_STATE_RESET;
    g_ok = adc2_configurar();
    return false;
}

void bateria_inicializar(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    gpio.Pin  = GPIO_PIN_1;            /* PC1 = BAT_REF */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    g_ok = adc2_configurar();
    if (!g_ok) printf("BAT: falha ao iniciar o ADC2. Monitor de bateria desligado.\r\n");
}

void bateria_tick_10ms(bool em_corrida)
{
    if (!g_ok) return;
    if (!adc2_conferir()) return;                 /* reconfigurado: le no proximo tick */
    if (!__HAL_ADC_GET_FLAG(&g_hadc2, ADC_FLAG_EOC)) {
        HAL_ADC_Start(&g_hadc2);                  /* conversao perdida: dispara de novo */
        return;
    }

    g_adc = (uint16_t)HAL_ADC_GetValue(&g_hadc2); /* le e limpa o EOC */
    HAL_ADC_Start(&g_hadc2);                      /* a proxima fica pronta em 25 us */

    float v = (float)g_adc * BAT_FATOR;
    if (g_primeira) {
        g_v_rapida = g_v_lenta = v;
        g_primeira = false;
    }
    g_v_rapida += (v - g_v_rapida) / 8.0f;    /* tau = 8 x 10 ms  = 80 ms  */
    g_v_lenta  += (v - g_v_lenta)  / 64.0f;   /* tau = 64 x 10 ms = 640 ms */
    if (g_ms_vida < BAT_TEMPO_PARTIDA_MS) g_ms_vida += 10U;

    /* Alarme com histerese: liga abaixo do limiar por 2 s, apaga acima de
     * limiar + 0,3 V por 3 s. O 1o segundo e ignorado. */
    float limiar = em_corrida ? BAT_LIMIAR_BAIXA_CORRIDA_V : BAT_LIMIAR_BAIXA_V;
    bool pode_avaliar = BAT_ALARME_HABILITADO && bateria_presente() && (g_ms_vida >= BAT_TEMPO_PARTIDA_MS);

    if (pode_avaliar && g_v_lenta < limiar) {
        g_ms_acima = 0;
        if (!g_baixa) {
            g_ms_abaixo += 10U;
            if (g_ms_abaixo >= BAT_TEMPO_CONFIRMA_MS) {
                g_baixa = true;
                g_v_disparo = g_v_lenta;
                g_adc_disparo = g_adc;
            }
        }
    } else {
        g_ms_abaixo = 0;
        if (g_baixa && (!BAT_ALARME_HABILITADO || g_v_lenta > limiar + BAT_HISTERESE_V)) {
            g_ms_acima += 10U;
            if (g_ms_acima >= BAT_TEMPO_LIBERA_MS) { g_baixa = false; g_ms_acima = 0; }
        } else {
            g_ms_acima = 0;
        }
    }

    bat_adc_bruto   = g_adc;
    bat_pino_mv     = (uint16_t)((uint32_t)g_adc * 3300U / 4095U);
    bat_tensao_calc = v;
    bat_tensao_live = g_v_lenta;
    bat_baixa_live  = g_baixa ? 1U : 0U;
    bat_saturado    = (g_adc >= BAT_ADC_SATURADO) ? 1U : 0U;
}

void bateria_imprimir_diagnostico(void)
{
    uint16_t adc = g_adc;
    printf("BAT: adc=%u pino=%u mV Vbat=%.2f V (filtrada %.2f V) fator=%.5f%s%s\r\n",
           (unsigned)adc, (unsigned)((uint32_t)adc * 3300U / 4095U),
           (double)((float)adc * BAT_FATOR), (double)g_v_lenta, (double)BAT_FATOR,
           (adc >= BAT_ADC_SATURADO) ? " SATURADO: R22 pequeno demais para essa bateria" : "",
           g_ok ? "" : " ADC2 DESLIGADO");
    if (g_baixa) {
        printf("BAT: alarme disparou com %.2f V (adc %u)\r\n",
               (double)g_v_disparo, (unsigned)g_adc_disparo);
    }
}

float    bateria_tensao(void)       { return g_v_rapida; }
float    bateria_tensao_lenta(void) { return g_v_lenta; }
bool     bateria_presente(void)     { return !g_primeira && g_v_lenta > BAT_V_MINIMA_PRESENTE; }
bool     bateria_baixa(void)        { return g_baixa; }
uint16_t bateria_adc(void)          { return g_adc; }
