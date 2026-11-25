/*
 * controle_succao.c
 */

#include "controle_succao.h"
#include "config_robo.h"
#include "bateria.h"
#include "main.h"

static TIM_HandleTypeDef *g_htim = NULL;
static uint8_t g_alvo = 0;
static uint8_t g_atual = 0;
static bool    g_pino_pwm = false;

/* Visivel no Live Expressions */
volatile uint8_t succao_pct_live = 0;

/* PA15 como saida comum em 0, com pull-down. Direto nos registradores:
 * funciona antes do HAL_Init e nao depende do estado da HAL. */
void succao_forcar_desligado(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;
    GPIOA->BSRR    = (1U << (15U + 16U));                         /* ODR15 = 0    */
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U << 30)) | (2U << 30);   /* pull-down    */
    GPIOA->OTYPER &= ~(1U << 15);                                 /* push-pull    */
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << 30)) | (1U << 30);   /* saida        */
    GPIOA->AFR[1] &= ~(0xFU << 28);                               /* tira o AF1   */
    if (RCC->APB1ENR & RCC_APB1ENR_TIM2EN) TIM2->CCR1 = 0;
    g_pino_pwm = false;
}

static void pino_para_pwm(void)
{
    if (g_pino_pwm) return;
    GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFU << 28)) | (1U << 28); /* AF1 = TIM2_CH1 */
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U << 30)) | (2U << 30);    /* mantem pull-down */
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << 30)) | (2U << 30);    /* funcao alternativa */
    g_pino_pwm = true;
}

static void aplicar(uint8_t pct)
{
    if (!g_htim) return;
    if (pct == 0U) {
        __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_1, 0);
        succao_forcar_desligado();
        succao_pct_live = 0;
        return;
    }
    float duty = (float)pct;
#if SUCCAO_COMPENSAR_BATERIA
    float v = bateria_tensao();
    if (bateria_presente() && v > 5.0f) {
        duty = duty * SUCCAO_V_REFERENCIA / v;      /* mesma tensao media no motor */
    }
#endif
    if (duty > (float)SUCCAO_MAX_PCT) duty = (float)SUCCAO_MAX_PCT;
    uint32_t ccr = (uint32_t)(duty * (float)(PERIODO_PWM_SUCCAO + 1U) / 100.0f);
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_1, ccr);
    pino_para_pwm();
    succao_pct_live = (uint8_t)duty;
}

void succao_inicializar(TIM_HandleTypeDef *htim_pwm)
{
    g_htim = htim_pwm;
    g_alvo = g_atual = 0;
    __HAL_TIM_SET_COMPARE(g_htim, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(g_htim, TIM_CHANNEL_1);   /* timer roda; o pino segue em GPIO 0 */
    succao_forcar_desligado();
}

void succao_definir_potencia(uint8_t pct)
{
#if SUCCAO_HABILITADA
    if (pct > SUCCAO_MAX_PCT) pct = SUCCAO_MAX_PCT;
    g_alvo = pct;
#else
    (void)pct;
    g_alvo = 0;
#endif
}

void succao_desligar(void)
{
    g_alvo = g_atual = 0;
    aplicar(0);
}

void succao_atualizar(void)
{
    if (g_atual < g_alvo) {
        g_atual = (uint8_t)(((g_alvo - g_atual) > SUCCAO_RAMPA_PASSO_PCT) ? (g_atual + SUCCAO_RAMPA_PASSO_PCT) : g_alvo);
    } else if (g_atual > g_alvo) {
        g_atual = g_alvo;                         /* reduzir e imediato */
    }
    aplicar(g_atual);                             /* reaplica: compensa a bateria */
}

bool    succao_estabilizada(void)   { return g_atual == g_alvo; }
uint8_t succao_potencia_atual(void) { return g_atual; }

uint8_t succao_pct_para_velocidade(int v)
{
    int imp = SUCCAO_IMP_BASE + ((v - SUCCAO_VEL_REF) / VELOCIDADE_PASSO) * SUCCAO_IMP_PASSO;
    if (imp > SUCCAO_IMP_MAX) imp = SUCCAO_IMP_MAX;
    if (imp < 0) imp = 0;
    int pct = (imp * 100 + 127) / 255;
    if (pct > SUCCAO_MAX_PCT) pct = SUCCAO_MAX_PCT;
    return (uint8_t)pct;
}
