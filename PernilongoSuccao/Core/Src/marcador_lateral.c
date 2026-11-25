/*
 * marcador_lateral.c
 */

#include "marcador_lateral.h"
#include "config_robo.h"
#include <stdlib.h>

#define BIT_ESQ 1U
#define BIT_DIR 2U

static bool     g_esq = false, g_dir = false;   /* estado com histerese        */
static uint8_t  g_candidato = 0, g_estavel = 0; /* filtro de LATERAL_FILTRO_MS  */
static uint32_t g_t_candidato = 0;
static bool     g_em_evento = false;
static uint8_t  g_acumulado = 0;                /* OR de tudo que foi visto     */
static bool     g_evento_cruz = false;
static bool     g_evento_invalido = false;
static uint32_t g_t_cruz = 0;
static bool     g_cruz_visto = false;
static bool     g_cruz_anterior = false;
static uint32_t g_t_ultima_intersecao = 0;
static bool     g_intersecao_reportada = false;

/* Visiveis no Live Expressions */
volatile uint16_t sl_dir_live = 0, sl_esq_live = 0;

void marcador_inicializar(void)
{
    g_esq = g_dir = false;
    g_candidato = g_estavel = 0;
    g_em_evento = false;
    g_acumulado = 0;
    g_evento_cruz = g_evento_invalido = false;
    g_cruz_visto = g_cruz_anterior = false;
    g_intersecao_reportada = false;
}

bool marcador_esq_branco(void) { return g_esq; }
bool marcador_dir_branco(void) { return g_dir; }

static bool histerese(bool atual, uint16_t v)
{
    if (atual) return v < (LIMIAR_SENSOR_LATERAL + HISTERESE_LATERAL);
    return v < LIMIAR_SENSOR_LATERAL;
}

tipo_marca_t marcador_atualizar(uint16_t lat_dir, uint16_t lat_esq, int16_t erro,
                                bool cruz_qtr, uint32_t agora)
{
    tipo_marca_t r = MARCA_NENHUMA;
    sl_dir_live = lat_dir;
    sl_esq_live = lat_esq;

    /* 1. Cruzamento visto pela regua: sinaliza na hora (borda de subida) */
    if (cruz_qtr) {
        g_t_cruz = agora;
        g_cruz_visto = true;
        if (!g_cruz_anterior && (agora - g_t_ultima_intersecao) > 150U) {
            g_t_ultima_intersecao = agora;
            g_intersecao_reportada = true;
            r = MARCA_INTERSECAO;
        }
    }
    g_cruz_anterior = cruz_qtr;
    bool janela_cruz = g_cruz_visto && ((agora - g_t_cruz) <= JANELA_CRUZAMENTO_MS);

    /* 2. Limiar com histerese: branco = marca */
    g_esq = histerese(g_esq, lat_esq);
    g_dir = histerese(g_dir, lat_dir);
    uint8_t leitura = (uint8_t)((g_esq ? BIT_ESQ : 0U) | (g_dir ? BIT_DIR : 0U));

    /* 3. Filtro: o estado novo precisa durar LATERAL_FILTRO_MS (main_sl.c) */
    if (leitura != g_candidato) {
        g_candidato = leitura;
        g_t_candidato = agora;
    }
    if ((agora - g_t_candidato) >= LATERAL_FILTRO_MS) g_estavel = g_candidato;

    /* 4. Evento: comeca no primeiro branco e termina quando os dois voltam ao preto */
    if (!g_em_evento) {
        if (g_estavel != 0U) {
            g_em_evento = true;
            g_acumulado = g_estavel;
            g_evento_cruz = janela_cruz;
            g_evento_invalido = (MARCA_ERRO_MAX > 0) && (abs(erro) > MARCA_ERRO_MAX);
        }
        return r;
    }

    if (g_estavel != 0U) {
        g_acumulado |= g_estavel;
        g_evento_cruz |= janela_cruz;
        if ((MARCA_ERRO_MAX > 0) && (abs(erro) > MARCA_ERRO_MAX)) g_evento_invalido = true;
        return r;
    }

    /* Fim do evento: classifica */
    g_em_evento = false;
    if (g_evento_cruz || g_acumulado == (BIT_ESQ | BIT_DIR)) {
        /* intersecao: so reporta se a regua ainda nao reportou esta */
        bool ja = g_intersecao_reportada && ((agora - g_t_ultima_intersecao) <= 300U);
        g_intersecao_reportada = false;
        if (!ja) {
            g_t_ultima_intersecao = agora;
            r = MARCA_INTERSECAO;
        }
    } else if (g_evento_invalido) {
        r = MARCA_IGNORADA;
    } else if (g_acumulado == BIT_DIR) {
        r = MARCA_DIREITA;
    } else if (g_acumulado == BIT_ESQ) {
        r = MARCA_ESQUERDA;
    }
    g_acumulado = 0;
    return r;
}
