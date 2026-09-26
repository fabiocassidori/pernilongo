// Core/Src/mapa_pista.c

#include "mapa_pista.h"
#include "config_robo.h"
#include "leitor_encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include "monitor_bateria.h"

static int g_mapa_dist_esq[TAMANHO_MAPA];
static int g_mapa_dist_dir[TAMANHO_MAPA];
static int g_mapa_dist_media[TAMANHO_MAPA];
static int32_t g_mapa_dist_media_pulsos[TAMANHO_MAPA];
static int g_mapa_raio[TAMANHO_MAPA];
// --- Variável de potência trocada por tipo de segmento ---
static TipoSegmento g_mapa_tipo_segmento[TAMANHO_MAPA];

static int g_segmentos_gravados = 0;
static int g_ponteiro_segmento = 0;

void mapa_inicializar(void) {
    for (int i = 0; i < TAMANHO_MAPA; i++) {
        g_mapa_dist_esq[i] = 0; g_mapa_dist_dir[i] = 0; g_mapa_dist_media[i] = 0;
        g_mapa_raio[i] = 0; g_mapa_dist_media_pulsos[i] = 0;
        g_mapa_tipo_segmento[i] = SEGMENTO_CURVA; // Padrão
    }
    g_segmentos_gravados = 0; g_ponteiro_segmento = 0;
}

void mapa_gravar_segmento(void) {
    if (g_segmentos_gravados >= TAMANHO_MAPA) { printf("ERRO: Mapa cheio!\r\n"); return; }

    int32_t pos_esq_pulsos = encoder_obter_posicao_esquerda();
    int32_t pos_dir_pulsos = encoder_obter_posicao_direita();
    g_mapa_dist_media_pulsos[g_segmentos_gravados] = (pos_esq_pulsos + pos_dir_pulsos) / 2;

    g_mapa_dist_esq[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_esq_pulsos);
    g_mapa_dist_dir[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_dir_pulsos);
    g_mapa_dist_media[g_segmentos_gravados] = (g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados]) / 2;

    int32_t diff = g_mapa_dist_esq[g_segmentos_gravados] - g_mapa_dist_dir[g_segmentos_gravados];
    if (diff == 0) diff = 1;
    int32_t sum = g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados];
    g_mapa_raio[g_segmentos_gravados] = abs((int)(6 * sum / diff));

    // ++: Armazena o TIPO do segmento, não a potência +++
    if (g_mapa_raio[g_segmentos_gravados] > LIMIAR_CURVA_RETA) {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_RETA;
    } else {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_CURVA;
    }

    // Garante que o primeiro segmento nunca seja uma reta de alta velocidade
    if (g_segmentos_gravados == 0) {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_CURVA;
    }

    encoder_resetar_posicoes();
    g_segmentos_gravados++;
}

bool mapa_corrida_terminou(uint8_t contador_fim) {
    return (contador_fim >= 2);
}

void mapa_resetar_para_corrida(void) {
    g_ponteiro_segmento = 0;
    encoder_resetar_posicoes();
}

void mapa_avancar_segmento(void) {
	if (g_ponteiro_segmento < g_segmentos_gravados - 1) {
	    g_ponteiro_segmento++;
	}
}

bool mapa_segmento_atual_e_reta_longa(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return false;
    }

    // +++ LÓGICA CORRIGIDA: Verifica o tipo do segmento e a distância gravada +++
    bool e_reta = (g_mapa_tipo_segmento[g_ponteiro_segmento] == SEGMENTO_RETA);
    if (!e_reta) {
        return false;
    }

    return (g_mapa_dist_media[g_ponteiro_segmento] > LIMIAR_DISTANCIA_RETA_LONGA);
}

int32_t mapa_obter_distancia_pulsos_segmento_atual(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return 0;
    }
    return g_mapa_dist_media_pulsos[g_ponteiro_segmento];
}

void mapa_logar_dados_swo(void) {

	// --- INÍCIO DA SEÇÃO DE TENSÃO DA BATERIA ---
	// Obtém a tensão atual da bateria a partir do módulo de monitoramento.
	float tensao_bateria = bateria_obter_tensao();
	// Imprime a tensão formatada com duas casas decimais.
	printf("Tensao da Bateria (estimada): %.2fV\r\n", tensao_bateria);
	printf("----------------------------------------\r\n"); // Separador para clareza
	// --- FIM DA SEÇÃO DE TENSÃO DA BATERIA ---

    printf("Iniciando impressao dos dados via SWO...\r\n");
    printf("Tipo Segmento (0=Curva, 1=Reta): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_tipo_segmento[i]);
    printf("\nMapesq: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_esq[i]);
    printf("\nMapdir: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_dir[i]);
    printf("\nMapa (media): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_media[i]);
    printf("\nRmapa (raio): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_raio[i]);
    printf("\nDistancia Retas (cm): ;");
        for (int i = 0; i < g_segmentos_gravados; i++) {
            // Verifica se o segmento atual é uma reta
            if (g_mapa_tipo_segmento[i] == SEGMENTO_RETA) {
                // Converte a distância de mm para cm e imprime
                int dist_cm = g_mapa_dist_media[i] / 10;
                printf("%d;", dist_cm);
            } else {
                // Se for uma curva, imprime 0 para manter o formato
                printf("0;");
            }
        }
    printf("\nImpressao concluida.\r\n");
}
