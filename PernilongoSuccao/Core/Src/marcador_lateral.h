/*
 * marcador_lateral.h
 *
 * Marcas laterais da pista (robotrace): SLDIR (PC0) e SLESQ (PB1).
 * Marca BRANCA = valor baixo. Sem calibracao: limiar fixo LIMIAR_SENSOR_LATERAL
 * com histerese. Um "evento" vai do primeiro sensor que ve branco ate os dois
 * voltarem ao preto; o tipo do evento e decidido no fim (logica OR do codigo
 * original + historico do main_sl.c + janela de cruzamento do EVA-RT).
 */

#ifndef INC_MARCADOR_LATERAL_H_
#define INC_MARCADOR_LATERAL_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MARCA_NENHUMA = 0,
    MARCA_ESQUERDA,
    MARCA_DIREITA,
    MARCA_INTERSECAO,
    MARCA_IGNORADA      /* evento descartado (linha muito fora do centro) */
} tipo_marca_t;

void         marcador_inicializar(void);
/* Chamar a cada leitura nova.
 * lat_dir/lat_esq: medias do ADC (0..1023); erro: erro da linha (-5500..5500);
 * cruz_qtr: a regua viu cruzamento nesta leitura; agora_ms: HAL_GetTick(). */
tipo_marca_t marcador_atualizar(uint16_t lat_dir, uint16_t lat_esq, int16_t erro,
                                bool cruz_qtr, uint32_t agora_ms);
bool         marcador_esq_branco(void);
bool         marcador_dir_branco(void);

#endif /* INC_MARCADOR_LATERAL_H_ */
