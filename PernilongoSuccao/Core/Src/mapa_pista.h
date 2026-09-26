/*
 * mapa_pista.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_MAPA_PISTA_H_
#define INC_MAPA_PISTA_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SEGMENTO_CURVA,
    SEGMENTO_RETA
} TipoSegmento;


void mapa_inicializar(void);
void mapa_gravar_segmento(void);
void mapa_resetar_para_corrida(void);
void mapa_avancar_segmento(void);
bool mapa_corrida_terminou(uint8_t contador_fim);
bool mapa_segmento_atual_e_reta_longa(void);
int32_t mapa_obter_distancia_pulsos_segmento_atual(void);
void mapa_logar_dados_swo(void);

#endif /* INC_MAPA_PISTA_H_ */
