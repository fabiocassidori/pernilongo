/*
 * controle_velocidade.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_CONTROLE_VELOCIDADE_H_
#define INC_CONTROLE_VELOCIDADE_H_

#include <stdint.h>

void velocidade_inicializar(void);
void velocidade_atualizar(void);
int velocidade_controlar_potencia(float velocidade_alvo_mmps);
float velocidade_obter_media_mmps(void);

#endif /* INC_CONTROLE_VELOCIDADE_H_ */
