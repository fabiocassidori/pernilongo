/*
 * interface_usuario.c
 */

#include "interface_usuario.h"
#include "config_robo.h"
#include "main.h"

/* ---------------- Botao ---------------- */
static bool     g_bot_estavel = false;   /* true = apertado (depois do debounce) */
static bool     g_bot_bruto_ant = false;
static uint32_t g_bot_t_mudanca = 0;
static uint32_t g_bot_t_aperto = 0;
static bool     g_bot_longo_dado = false;
static volatile evento_botao_t g_bot_evento = BOTAO_NADA;

/* ---------------- Buzina: ate 6 segmentos liga/desliga ---------------- */
static uint16_t g_seg[6];
static uint8_t  g_seg_n = 0, g_seg_i = 0;
static uint32_t g_seg_t = 0;
static bool     g_tocando = false;

/* ---------------- LEDs ---------------- */
static modo_led_t g_modo_verde = LED_APAGADO, g_modo_verm = LED_APAGADO;
static bool       g_flash_verde = false, g_flash_verm = false;
static uint32_t   g_flash_ate = 0;
static bool       g_flash_ativo = false;
static bool       g_alarme = false, g_alarme_bip = false;
static uint32_t   g_alarme_t = 0;
static uint32_t   g_agora = 0;

static inline void buzina(bool on) { HAL_GPIO_WritePin(BUZINA_GPIO_Port, BUZINA_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void led_verde(bool on) { HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void led_verm(bool on)  { HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void iu_inicializar(void)
{
    buzina(false);
    led_verde(false);
    led_verm(false);
}

bool iu_botao_apertado(void) { return g_bot_estavel; }

evento_botao_t iu_botao_evento(void)
{
    evento_botao_t e = g_bot_evento;
    g_bot_evento = BOTAO_NADA;
    return e;
}

static void botao_atualizar(uint32_t agora)
{
    bool bruto = (HAL_GPIO_ReadPin(BOTAO_GPIO_Port, BOTAO_Pin) == GPIO_PIN_RESET);
    if (bruto != g_bot_bruto_ant) {
        g_bot_bruto_ant = bruto;
        g_bot_t_mudanca = agora;
    }
    if ((agora - g_bot_t_mudanca) >= BOTAO_DEBOUNCE_MS && bruto != g_bot_estavel) {
        g_bot_estavel = bruto;
        if (bruto) {                                  /* apertou */
            g_bot_t_aperto = agora;
            g_bot_longo_dado = false;
        } else if (!g_bot_longo_dado) {               /* soltou antes de 1 s */
            g_bot_evento = BOTAO_CURTO;
        }
    }
    if (g_bot_estavel && !g_bot_longo_dado && (agora - g_bot_t_aperto) >= BOTAO_LONGO_MS) {
        g_bot_longo_dado = true;                      /* longo dispara sem soltar */
        g_bot_evento = BOTAO_LONGO;
    }
}

static void tocar(const uint16_t *seg, uint8_t n)
{
    for (uint8_t i = 0; i < n && i < 6U; i++) g_seg[i] = seg[i];
    g_seg_n = n;
    g_seg_i = 0;
    g_seg_t = g_agora;
    g_tocando = true;
    buzina(true);                                     /* segmento 0 = ligado */
}

void iu_bip(uint16_t ms)                       { uint16_t s[1] = { ms }; tocar(s, 1); }
void iu_bip_duplo(uint16_t on, uint16_t off)   { uint16_t s[3] = { on, off, on }; tocar(s, 3); }
void iu_bip_triplo(uint16_t on, uint16_t off)  { uint16_t s[5] = { on, off, on, off, on }; tocar(s, 5); }

void iu_bip_bloqueante(uint16_t ms)
{
    buzina(true);
    HAL_Delay(ms);
    buzina(false);
}

static void buzina_atualizar(uint32_t agora)
{
    if (!g_tocando) return;
    if ((agora - g_seg_t) >= g_seg[g_seg_i]) {
        g_seg_t = agora;
        g_seg_i++;
        if (g_seg_i >= g_seg_n) {
            g_tocando = false;
            buzina(false);
        } else {
            buzina((g_seg_i & 1U) == 0U);             /* pares = ligado */
        }
    }
}

void iu_led_modo(modo_led_t verde, modo_led_t vermelho)
{
    g_modo_verde = verde;
    g_modo_verm = vermelho;
}

void iu_led_flash(bool verde, bool vermelho, uint16_t ms)
{
    g_flash_verde = verde;
    g_flash_verm = vermelho;
    g_flash_ate = g_agora + ms;
    g_flash_ativo = true;
}

void iu_alarme_bateria(bool ativo, bool com_bip)
{
    if (ativo && !g_alarme) g_alarme_t = g_agora - BAT_INTERVALO_ALARME_MS;  /* bipa ja */
    g_alarme = ativo;
    g_alarme_bip = com_bip;
}

static bool estado_led(modo_led_t m, uint32_t agora)
{
    switch (m) {
    case LED_ACESO:        return true;
    case LED_PISCA_LENTO:  return ((agora / 500U) & 1U) != 0U;
    case LED_PISCA_RAPIDO: return ((agora / 100U) & 1U) != 0U;
    default:               return false;
    }
}

void iu_atualizar(uint32_t agora)
{
    g_agora = agora;
    botao_atualizar(agora);
    buzina_atualizar(agora);

    /* Alarme de bateria: bip curto periodico, so quando nada mais esta tocando */
    if (g_alarme && g_alarme_bip && !g_tocando && (agora - g_alarme_t) >= BAT_INTERVALO_ALARME_MS) {
        g_alarme_t = agora;
        iu_bip_duplo(80, 80);
    }

    if (g_flash_ativo && (int32_t)(g_flash_ate - agora) <= 0) g_flash_ativo = false;

    if (g_flash_ativo) {                              /* marcas: prioridade maxima */
        led_verde(g_flash_verde);
        led_verm(g_flash_verm);
    } else {
        led_verde(estado_led(g_modo_verde, agora));
        led_verm(g_alarme ? true : estado_led(g_modo_verm, agora));
    }
}
