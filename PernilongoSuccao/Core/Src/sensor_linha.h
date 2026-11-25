/*
 * sensor_linha.h
 *
 * Regua de 12 sensores QRE1113 com LEITURA PULSADA feita por interrupcao.
 *
 * O ADC1 converte os 14 canais sem parar (DMA circular, 2 varreduras no
 * buffer). A cada varredura completa, o callback do DMA roda uma pequena
 * maquina de estados que acende e apaga os emissores (PA8) e soma as
 * varreduras certas. O laco principal nunca espera o ADC: ele so pega a
 * leitura pronta com qtr_nova_leitura().
 *
 * Fluxo de uma leitura:  APAGADO(a) -> ACESO(b) -> APAGADO(c)
 *   dif = (a + c)/2 - b      (so o reflexo do LED; a luz ambiente cancela)
 *   bruto = 1023 - dif       (convencao Pololu: maior = mais escuro)
 * A fase APAGADO(c) e reaproveitada como (a) da leitura seguinte.
 */

#ifndef INC_SENSOR_LINHA_H_
#define INC_SENSOR_LINHA_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "config_robo.h"

/* Leitura publicada pela interrupcao. Indice 0 = QTR01 (esquerda) ... 11 = QTR12. */
typedef struct {
    uint16_t bruto[NUM_SENSORES_LINHA];  /* 0..1023, maior = menos reflexo (preto) */
    int16_t  dif[NUM_SENSORES_LINHA];    /* apagado - aceso (reflexo puro do LED)  */
    uint16_t apagado[NUM_SENSORES_LINHA];/* nivel com LED apagado (luz ambiente)    */
    uint16_t lateral_dir;                /* media das varreduras do periodo (0..1023) */
    uint16_t lateral_esq;
    uint32_t seq;                        /* numero da leitura (sobe sempre)         */
} qtr_leitura_t;

/* Resultado do processamento de uma leitura. */
typedef struct {
    uint16_t calibrado[NUM_SENSORES_LINHA]; /* 0..1000, 1000 = linha (ja invertido) */
    uint16_t ativo[NUM_SENSORES_LINHA];     /* acima do limiar do EVA-RT, 0..1000   */
    uint16_t posicao;     /* 0 (QTR01, esquerda) .. 11000 (QTR12, direita)         */
    int16_t  erro;        /* posicao - 5500                                          */
    uint8_t  n_ativos;    /* sensores acima do limiar                                */
    bool     na_linha;    /* algum sensor acima do limiar                            */
    bool     valida;      /* na linha e fora dos extremos (criterio do EVA-RT)       */
    bool     cruzamento;  /* sensores 2 e 9 juntos, ou muitos sensores na linha     */
    int8_t   ultimo_lado; /* -1 = linha saiu pela esquerda, +1 = pela direita       */
} qtr_linha_t;

void     qtr_inicializar(ADC_HandleTypeDef *hadc);
/* Copia a leitura mais nova se ela ainda nao foi lida. Retorna true se havia nova. */
bool     qtr_nova_leitura(qtr_leitura_t *destino);
/* Mede o tempo de resposta dos sensores e ajusta a espera da pulsagem.
 * Leva ~10 ms. Retorna true se ao menos um sensor deu degrau suficiente. */
bool     qtr_medir_resposta(void);
uint32_t qtr_espera_us(void);
uint32_t qtr_varredura_ns(void);
uint32_t qtr_periodo_leitura_us(void);

/* Filtro de mediana de 3 no tempo (por sensor). */
void     qtr_filtrar(const uint16_t *bruto, uint16_t *filtrado);

/* Calibracao estilo Pololu: chame amostrar() a cada leitura durante a varredura. */
void     qtr_calibracao_resetar(void);
void     qtr_calibracao_amostrar(const uint16_t *valores);
/* Retorna a quantidade de sensores com contraste abaixo de QTR_CONTRASTE_MIN. */
uint8_t  qtr_calibracao_finalizar(void);
void     qtr_calibracao_imprimir(void);

/* Calibrado -> limiar -> media ponderada -> posicao e erro. */
void     qtr_processar(const uint16_t *filtrado, qtr_linha_t *saida);
void     qtr_reiniciar_estado(void);

/* Chamar a cada 10 ms: reinicia o ADC/DMA se as varreduras pararem. */
void     qtr_vigiar_adc(void);
uint32_t qtr_contador_varreduras(void);
uint32_t qtr_reinicios_adc(void);

#endif /* INC_SENSOR_LINHA_H_ */
