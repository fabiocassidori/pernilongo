/*
 * sensor_linha.c
 *
 * Leitura pulsada dos 12 QRE1113 por interrupcao do DMA, filtro de mediana,
 * calibracao Pololu, limiar do EVA-RT e media ponderada.
 * Base: main_qtr_pulsado.c (validado) reorganizado para nao bloquear a CPU.
 */

#include "sensor_linha.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Mapa do buffer do DMA (ordem dos Ranks do .ioc), em cada metade:
 *  [0]=SLDIR(PC0)  [1]=QTR12(PC3)  [2]=QTR11(PA0)  [3]=QTR10(PA1)
 *  [4]=QTR09(PA2)  [5]=QTR08(PA3)  [6]=QTR07(PA4)  [7]=QTR06(PA5)
 *  [8]=QTR05(PA6)  [9]=QTR04(PA7)  [10]=QTR03(PC4) [11]=QTR02(PC5)
 *  [12]=QTR01(PB0) [13]=SLESQ(PB1)
 * ------------------------------------------------------------------------- */
static const uint8_t QTR_IDX_DMA[NUM_SENSORES_LINHA] = {
    12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1      /* QTR01 ... QTR12 */
};

/* Canais do ADC1 por grupo (para o tempo de amostragem) */
static const uint8_t CANAIS_QTR[NUM_SENSORES_LINHA] = { 13, 0, 1, 2, 3, 4, 5, 6, 7, 14, 15, 8 };
static const uint8_t CANAIS_LATERAIS[2] = { 10, 9 };

static ADC_HandleTypeDef *g_hadc = NULL;
static volatile uint16_t g_adc_dma[2 * NUM_CANAIS_ADC];

/* ============================ Maquina de pulsagem =========================
 * Roda dentro dos callbacks do DMA, uma vez por varredura (~52 us). */
typedef enum { MODO_PULSO = 0, MODO_CAPTURA } modo_t;
typedef enum { FASE_APAGADO = 0, FASE_ACESO } fase_t;
typedef enum { CAP_ESCURO = 0, CAP_LIGANDO, CAP_DESLIGANDO, CAP_PRONTA } cap_estado_t;

static volatile modo_t   g_modo = MODO_PULSO;
static volatile bool     g_pedido_captura = false;
static volatile bool     g_pedido_reinicio = false;
static fase_t            g_fase = FASE_APAGADO;
static uint16_t          g_n_fase = 0;          /* varreduras desde a troca do LED */
static uint16_t          g_n_acum = 0;          /* varreduras somadas nesta fase   */
static volatile uint16_t g_descarte = 7;        /* varreduras jogadas fora         */
static uint32_t          g_soma[NUM_SENSORES_LINHA];
static uint32_t          g_soma_aceso[NUM_SENSORES_LINHA];
static uint32_t          g_soma_apag_ant[NUM_SENSORES_LINHA];
static bool              g_tem_aceso = false;
static bool              g_tem_apag_ant = false;
static uint32_t          g_lat_soma_d = 0, g_lat_soma_e = 0, g_lat_n = 0;

static volatile uint32_t g_varreduras = 0;
static volatile uint32_t g_reinicios = 0;
static volatile uint32_t g_ultimo_dwt = 0;

/* Leitura publicada (dupla: a ISR escreve numa, o laco le a outra) */
static qtr_leitura_t     g_pub[2];
static volatile uint8_t  g_pub_idx = 0;
static volatile uint32_t g_pub_seq = 0;
static uint32_t          g_lida_seq = 0;

/* Captura do degrau (medicao do tempo de resposta) */
#define CAP_PREP     40   /* ~2,1 ms no escuro antes do degrau  */
#define CAP_N        48   /* ~2,5 ms gravados apos cada degrau  */
static volatile cap_estado_t g_cap_estado = CAP_ESCURO;
static uint16_t g_cap_n = 0;
static uint16_t g_cap_base[NUM_SENSORES_LINHA];
static uint16_t g_cap_on[CAP_N][NUM_SENSORES_LINHA];
static uint16_t g_cap_off[CAP_N][NUM_SENSORES_LINHA];
static uint32_t g_cap_t0 = 0, g_cap_t1 = 0;

static volatile uint32_t g_espera_us = QTR_ESPERA_PADRAO_US;
static volatile uint32_t g_varredura_ns = 52400;

/* ---- Visiveis no Live Expressions ---- */
volatile uint16_t qtr_bruto_live[NUM_SENSORES_LINHA];
volatile uint16_t qtr_cal_live[NUM_SENSORES_LINHA];
volatile uint16_t qtr_posicao_live = 0;

static inline void emissores(bool ligar)
{
    /* Escrita direta no BSRR: 1 ciclo, segura dentro da ISR */
    PWM_SENSOR_GPIO_Port->BSRR = ligar ? PWM_SENSOR_Pin : ((uint32_t)PWM_SENSOR_Pin << 16U);
}

static void pulso_reiniciar(void)
{
    emissores(false);
    g_fase = FASE_APAGADO;
    g_n_fase = 0;
    g_n_acum = 0;
    g_tem_aceso = false;
    g_tem_apag_ant = false;
    memset(g_soma, 0, sizeof(g_soma));
}

static void publicar(const uint32_t *apag_c)
{
    uint8_t w = (uint8_t)(g_pub_idx ^ 1U);
    qtr_leitura_t *p = &g_pub[w];

    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        /* somas de QTR_AMOSTRAS varreduras -> media com arredondamento */
        int32_t apag  = (int32_t)((g_soma_apag_ant[i] + apag_c[i] + QTR_AMOSTRAS) / (2U * QTR_AMOSTRAS));
        int32_t aceso = (int32_t)((g_soma_aceso[i] + QTR_AMOSTRAS / 2U) / QTR_AMOSTRAS);
        int32_t dif   = apag - aceso;
        int32_t v     = ADC_VALOR_MAXIMO - dif;
        if (v < 0) v = 0;
        if (v > ADC_VALOR_MAXIMO) v = ADC_VALOR_MAXIMO;
        p->dif[i]     = (int16_t)dif;
        p->apagado[i] = (uint16_t)apag;
        p->bruto[i]   = (uint16_t)v;
    }
    if (g_lat_n > 0) {
        p->lateral_dir = (uint16_t)(g_lat_soma_d / g_lat_n);
        p->lateral_esq = (uint16_t)(g_lat_soma_e / g_lat_n);
    }
    g_lat_soma_d = g_lat_soma_e = g_lat_n = 0;
    p->seq = g_pub_seq + 1U;
    g_pub_idx = w;
    g_pub_seq = p->seq;
}

static void fim_de_fase(void)
{
    if (g_fase == FASE_ACESO) {
        memcpy(g_soma_aceso, g_soma, sizeof(g_soma));
        g_tem_aceso = g_tem_apag_ant;      /* so vale se houve apagado antes */
        g_fase = FASE_APAGADO;
        emissores(false);
    } else {
        if (g_tem_aceso && g_tem_apag_ant) {
            publicar(g_soma);
        }
        memcpy(g_soma_apag_ant, g_soma, sizeof(g_soma));
        g_tem_apag_ant = true;
        g_tem_aceso = false;
        g_fase = FASE_ACESO;
        emissores(true);
    }
    memset(g_soma, 0, sizeof(g_soma));
    g_n_fase = 0;
    g_n_acum = 0;
}

static void captura_varredura(uint32_t base)
{
    switch (g_cap_estado) {
    case CAP_ESCURO:
        if (++g_cap_n >= CAP_PREP) {
            for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++)
                g_cap_base[i] = g_adc_dma[base + QTR_IDX_DMA[i]];
            g_cap_n = 0;
            g_cap_estado = CAP_LIGANDO;
            g_cap_t0 = DWT->CYCCNT;
            emissores(true);
        }
        break;
    case CAP_LIGANDO:
        for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++)
            g_cap_on[g_cap_n][i] = g_adc_dma[base + QTR_IDX_DMA[i]];
        if (++g_cap_n >= CAP_N) {
            g_cap_t1 = DWT->CYCCNT;
            g_cap_n = 0;
            g_cap_estado = CAP_DESLIGANDO;
            emissores(false);
        }
        break;
    case CAP_DESLIGANDO:
        for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++)
            g_cap_off[g_cap_n][i] = g_adc_dma[base + QTR_IDX_DMA[i]];
        if (++g_cap_n >= CAP_N) {
            g_cap_estado = CAP_PRONTA;
        }
        break;
    default:
        break;
    }
}

/* Chamado uma vez por varredura completa, dentro da interrupcao do DMA */
static void varredura_pronta(uint32_t base)
{
    g_varreduras++;
    g_ultimo_dwt = DWT->CYCCNT;

    /* Reinicio antes da captura: um pedido de captura feito logo depois de
     * um reinicio nao pode ser cancelado por ele. */
    if (g_pedido_reinicio) {
        g_pedido_reinicio = false;
        g_modo = MODO_PULSO;
        pulso_reiniciar();
    }
    if (g_pedido_captura) {
        g_pedido_captura = false;
        g_modo = MODO_CAPTURA;
        g_cap_estado = CAP_ESCURO;
        g_cap_n = 0;
        emissores(false);
    }

    /* Laterais: os LEDs deles ficam sempre acesos, toda varredura vale */
    g_lat_soma_d += g_adc_dma[base + IDX_LATERAL_DIREITO];
    g_lat_soma_e += g_adc_dma[base + IDX_LATERAL_ESQUERDO];
    g_lat_n++;

    if (g_modo == MODO_CAPTURA) {
        captura_varredura(base);
        return;
    }

    /* As primeiras g_descarte varreduras apos a troca do LED comecaram antes
     * do fototransistor estabilizar (a primeira comecou ate antes da troca). */
    if (++g_n_fase <= g_descarte) return;

    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++)
        g_soma[i] += g_adc_dma[base + QTR_IDX_DMA[i]];

    if (++g_n_acum >= QTR_AMOSTRAS) fim_de_fase();
}

/* 1a metade pronta (o DMA ja escreve a 2a) */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) varredura_pronta(0);
}

/* 2a metade pronta (o DMA voltou para a 1a) */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) varredura_pronta(NUM_CANAIS_ADC);
}

/* Overrun ou erro de DMA: o ADC para. Pede reinicio no laco lento. */
static volatile bool g_adc_erro = false;
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) g_adc_erro = true;
}

/* ================================ Inicializacao ========================== */
static uint32_t smp_codigo(uint32_t ciclos)
{
    switch (ciclos) {
    case 3:   return 0U;
    case 15:  return 1U;
    case 28:  return 2U;
    case 56:  return 3U;
    case 84:  return 4U;
    case 112: return 5U;
    case 144: return 6U;
    default:  return 7U;  /* 480 */
    }
}

static void adc_definir_amostragem(uint8_t canal, uint32_t codigo)
{
    if (canal <= 9U) {
        ADC1->SMPR2 = (ADC1->SMPR2 & ~(7U << (3U * canal))) | (codigo << (3U * canal));
    } else {
        uint32_t s = 3U * (canal - 10U);
        ADC1->SMPR1 = (ADC1->SMPR1 & ~(7U << s)) | (codigo << s);
    }
}

static void adc_iniciar(void)
{
    g_adc_erro = false;
    if (HAL_ADC_Start_DMA(g_hadc, (uint32_t *)g_adc_dma, 2U * NUM_CANAIS_ADC) != HAL_OK) {
        Error_Handler();
    }
}

void qtr_inicializar(ADC_HandleTypeDef *hadc)
{
    g_hadc = hadc;

    /* DWT: contador de ciclos para medir tempos em us */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Tempos de amostragem por grupo (o .ioc deixa todos em 15 ciclos) */
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++)
        adc_definir_amostragem(CANAIS_QTR[i], smp_codigo(QTR_ADC_SMP_CICLOS));
    for (uint8_t i = 0; i < 2; i++)
        adc_definir_amostragem(CANAIS_LATERAIS[i], smp_codigo(LATERAL_ADC_SMP_CICLOS));

    /* Varredura teorica: (12 x (56+10) + 2 x (144+10)) ciclos de ADCCLK */
    uint32_t adcclk = HAL_RCC_GetPCLK2Freq() / 4U;
    uint32_t ciclos = NUM_SENSORES_LINHA * (QTR_ADC_SMP_CICLOS + 10U) + 2U * (LATERAL_ADC_SMP_CICLOS + 10U);
    g_varredura_ns = (uint32_t)(((uint64_t)ciclos * 1000000000ULL) / adcclk);

    qtr_calibracao_resetar();
    pulso_reiniciar();
    g_espera_us = QTR_ESPERA_PADRAO_US;
    g_descarte = (uint16_t)((g_espera_us * 1000U + g_varredura_ns - 1U) / g_varredura_ns + 1U);

    /* O buffer de 2 varreduras precisa do DMA em modo circular */
    if (g_hadc->DMA_Handle != NULL && g_hadc->DMA_Handle->Init.Mode != DMA_CIRCULAR) {
        g_hadc->DMA_Handle->Init.Mode = DMA_CIRCULAR;
        if (HAL_DMA_Init(g_hadc->DMA_Handle) != HAL_OK) Error_Handler();
    }
    adc_iniciar();
}

bool qtr_nova_leitura(qtr_leitura_t *destino)
{
    if (g_pub_seq == g_lida_seq) return false;
    __disable_irq();
    *destino = g_pub[g_pub_idx];
    __enable_irq();
    g_lida_seq = destino->seq;
    return true;
}

uint32_t qtr_espera_us(void)          { return g_espera_us; }
uint32_t qtr_varredura_ns(void)       { return g_varredura_ns; }
uint32_t qtr_contador_varreduras(void){ return g_varreduras; }
uint32_t qtr_reinicios_adc(void)      { return g_reinicios; }

uint32_t qtr_periodo_leitura_us(void)
{
    /* uma leitura = fase acesa + fase apagada */
    return (2U * (g_descarte + QTR_AMOSTRAS) * g_varredura_ns) / 1000U;
}

/* ===================== Medicao do tempo de resposta ====================== */
/* Ultimo indice em que a amostra ainda estava fora da faixa final (+-QTR_TOLERANCIA_PCT) */
static int32_t ultimo_fora(uint16_t amostras[][NUM_SENSORES_LINHA], uint8_t i, int32_t final, int32_t tol)
{
    int32_t ult = -1;
    for (int32_t n = 0; n < CAP_N; n++) {
        if (abs((int32_t)amostras[n][i] - final) > tol) ult = n;
    }
    return ult;
}

static int32_t media_final(uint16_t amostras[][NUM_SENSORES_LINHA], uint8_t i)
{
    int32_t s = 0;
    for (int32_t n = CAP_N - 8; n < CAP_N; n++) s += amostras[n][i];
    return (s + 4) / 8;
}

bool qtr_medir_resposta(void)
{
#if QTR_ESPERA_AUTO
    g_pedido_captura = true;
    uint32_t t0 = HAL_GetTick();
    while (g_cap_estado != CAP_PRONTA || g_modo != MODO_CAPTURA) {
        if ((HAL_GetTick() - t0) > 50U) {         /* ADC parado? */
            g_pedido_reinicio = true;
            return false;
        }
    }

    /* Varredura medida de verdade (DWT entre a 1a e a ultima do degrau) */
    uint32_t ciclos = g_cap_t1 - g_cap_t0;
    uint32_t ns = (uint32_t)(((uint64_t)ciclos * 1000ULL) / (SystemCoreClock / 1000000U) / CAP_N);
    if (ns > 10000U && ns < 200000U) g_varredura_ns = ns;

    uint32_t pior = 0;
    uint8_t validos = 0;
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        int32_t fim_on  = media_final(g_cap_on, i);
        int32_t fim_off = media_final(g_cap_off, i);
        int32_t degrau  = fim_off - fim_on;        /* aceso fica mais baixo */
        if (abs(degrau) < QTR_DEGRAU_MIN) continue; /* sensor no preto: sem degrau */
        validos++;
        int32_t tol = abs(degrau) * QTR_TOLERANCIA_PCT / 100;
        if (tol < 3) tol = 3;
        int32_t a = ultimo_fora(g_cap_on, i, fim_on, tol) + 1;   /* varreduras ate entrar na faixa */
        int32_t b = ultimo_fora(g_cap_off, i, fim_off, tol) + 1;
        uint32_t t = (uint32_t)((a > b) ? a : b) * g_varredura_ns / 1000U;
        if (t > pior) pior = t;
    }

    if (validos > 0) {
        uint32_t e = pior + pior / 4U;              /* 1,25 x o pior caso */
        if (e < QTR_ESPERA_MIN_US) e = QTR_ESPERA_MIN_US;
        if (e > QTR_ESPERA_MAX_US) e = QTR_ESPERA_MAX_US;
        g_espera_us = e;
    } else {
        g_espera_us = QTR_ESPERA_PADRAO_US;
    }
    g_descarte = (uint16_t)((g_espera_us * 1000U + g_varredura_ns - 1U) / g_varredura_ns + 1U);
    g_pedido_reinicio = true;                       /* volta para a pulsagem */
    printf("QTR: %u sensores medidos, resposta %lu us, espera %lu us, descarte %u varreduras\r\n",
           validos, (unsigned long)pior, (unsigned long)g_espera_us, g_descarte);
    printf("QTR: varredura %lu ns, 1 leitura a cada %lu us\r\n",
           (unsigned long)g_varredura_ns, (unsigned long)qtr_periodo_leitura_us());
    return validos > 0;
#else
    return true;
#endif
}

/* ============================ Filtro de mediana ========================== */
static uint16_t g_hist[2][NUM_SENSORES_LINHA];
static uint8_t  g_hist_n = 0;

static inline uint16_t mediana3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}

void qtr_filtrar(const uint16_t *bruto, uint16_t *filtrado)
{
#if QTR_FILTRO_MEDIANA
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        uint16_t x = bruto[i];
        if (g_hist_n < 2) filtrado[i] = x;           /* ainda sem historico */
        else filtrado[i] = mediana3(g_hist[0][i], g_hist[1][i], x);
        g_hist[0][i] = g_hist[1][i];
        g_hist[1][i] = x;
    }
    if (g_hist_n < 2) g_hist_n++;
#else
    memcpy(filtrado, bruto, NUM_SENSORES_LINHA * sizeof(uint16_t));
#endif
}

/* =============================== Calibracao ============================== */
static uint16_t g_cal_min[NUM_SENSORES_LINHA];
static uint16_t g_cal_max[NUM_SENSORES_LINHA];
static uint16_t g_lote_max[NUM_SENSORES_LINHA];
static uint16_t g_lote_min[NUM_SENSORES_LINHA];
static uint8_t  g_lote_n = 0;

void qtr_calibracao_resetar(void)
{
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        g_cal_min[i] = 0xFFFF;
        g_cal_max[i] = 0;
    }
    g_lote_n = 0;
}

/* Pololu calibrate(): em lotes de 10 leituras guarda o MAIOR dos minimos e o
 * MENOR dos maximos do lote. Um pico isolado nunca vira o min ou o max. */
void qtr_calibracao_amostrar(const uint16_t *v)
{
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        if (g_lote_n == 0 || v[i] > g_lote_max[i]) g_lote_max[i] = v[i];
        if (g_lote_n == 0 || v[i] < g_lote_min[i]) g_lote_min[i] = v[i];
    }
    if (++g_lote_n >= 10U) {
        for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
            if (g_lote_min[i] > g_cal_max[i]) g_cal_max[i] = g_lote_min[i];
            if (g_lote_max[i] < g_cal_min[i]) g_cal_min[i] = g_lote_max[i];
        }
        g_lote_n = 0;
    }
}

uint8_t qtr_calibracao_finalizar(void)
{
    uint8_t ruins = 0;
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        if (g_cal_max[i] < g_cal_min[i] || (g_cal_max[i] - g_cal_min[i]) < QTR_CONTRASTE_MIN) ruins++;
    }
    return ruins;
}

void qtr_calibracao_imprimir(void)
{
    printf("Sensor:   ");
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) printf(" Q%02u ", i + 1U);
    printf("\r\nMin:      ");
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) printf("%4u ", g_cal_min[i]);
    printf("\r\nMax:      ");
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) printf("%4u ", g_cal_max[i]);
    printf("\r\nContraste:");
    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        int32_t c = (int32_t)g_cal_max[i] - (int32_t)g_cal_min[i];
        printf("%4ld%s", (long)c, (c < QTR_CONTRASTE_MIN) ? "!" : " ");
    }
    printf("\r\n");
}

/* ======================= Processamento da posicao ======================== */
static uint16_t g_ultima_posicao = POSICAO_CENTRO;
static int8_t   g_ultimo_lado = 0;

void qtr_reiniciar_estado(void)
{
    g_ultima_posicao = POSICAO_CENTRO;
    g_ultimo_lado = 0;
    g_hist_n = 0;
}

void qtr_processar(const uint16_t *v, qtr_linha_t *s)
{
    uint32_t num = 0, den = 0;
    s->n_ativos = 0;
    s->na_linha = false;

    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        /* 1. Normaliza 0..1000 com o min/max da calibracao (1000 = preto) */
        int32_t faixa = (int32_t)g_cal_max[i] - (int32_t)g_cal_min[i];
        int32_t x = 0;
        if (faixa > 0) x = (((int32_t)v[i] - (int32_t)g_cal_min[i]) * 1000) / faixa;
        if (x < 0) x = 0;
        if (x > 1000) x = 1000;
#if QTR_LINHA_BRANCA
        x = 1000 - x;                     /* agora 1000 = linha (branco) */
#endif
        s->calibrado[i] = (uint16_t)x;

        /* 2. Limiar do EVA-RT: abaixo de 75 % = 0, acima reescala 0..1000 */
        int32_t a = 0;
        if (x > QTR_LIMIAR_LINHA) a = ((x - QTR_LIMIAR_LINHA) * 1000) / (1000 - QTR_LIMIAR_LINHA);
        s->ativo[i] = (uint16_t)a;
        if (a > 0) {
            s->na_linha = true;
            s->n_ativos++;
            num += (uint32_t)a * (i * 1000U);
            den += (uint32_t)a;
        }
        qtr_cal_live[i] = (uint16_t)x;
        qtr_bruto_live[i] = v[i];
    }

    /* 3. Ultimo lado visto (EVA-RT 1.2.2: sensores 0/1 = esquerda, 10/11 = direita) */
    if (s->na_linha) {
        if (s->ativo[0] || s->ativo[1]) g_ultimo_lado = -1;
        else if (s->ativo[NUM_SENSORES_LINHA - 1] || s->ativo[NUM_SENSORES_LINHA - 2]) g_ultimo_lado = +1;
    }
    s->ultimo_lado = g_ultimo_lado;

    /* 4. Media ponderada */
    if (s->na_linha) {
        g_ultima_posicao = (uint16_t)(num / den);
    } else {
        g_ultima_posicao = (g_ultimo_lado < 0) ? 0 : (g_ultimo_lado > 0 ? POSICAO_MAX : g_ultima_posicao);
    }
    s->posicao = g_ultima_posicao;
    s->erro = (int16_t)((int32_t)s->posicao - POSICAO_CENTRO);
    s->valida = s->na_linha && (s->posicao != 0) && (s->posicao != POSICAO_MAX);

    /* 5. Cruzamento (EVA-RT CORRECT_CROSS_ERROR) */
    s->cruzamento = (s->ativo[QTR_CRUZ_SENSOR_A] && s->ativo[QTR_CRUZ_SENSOR_B]) ||
                    (s->n_ativos >= QTR_CRUZ_MIN_SENSORES);
    qtr_posicao_live = s->posicao;
}

/* ============================== Vigia do ADC ============================= */
void qtr_vigiar_adc(void)
{
    static uint32_t ultima = 0;
    static uint8_t parado = 0;
    uint32_t agora = g_varreduras;

    if (agora == ultima || g_adc_erro) {
        if (++parado >= 2U || g_adc_erro) {        /* 20 ms sem varredura */
            HAL_ADC_Stop_DMA(g_hadc);
            __HAL_ADC_CLEAR_FLAG(g_hadc, ADC_FLAG_OVR);
            g_pedido_reinicio = true;
            g_modo = MODO_PULSO;
            pulso_reiniciar();
            adc_iniciar();
            g_reinicios++;
            parado = 0;
        }
    } else {
        parado = 0;
    }
    ultima = agora;
}
