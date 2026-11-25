/*
 * controlador_pid.h
 *
 * Controle PD de direcao do EVA-RT v1.2.2:
 *   u = KP * e + KD * (e - e_anterior),   e = posicao - 5500
 * A saida e limitada a +-PID_LIMITE_SAIDA (510, igual ao EVA-RT) e somada /
 * subtraida da velocidade base de cada roda.
 */

#ifndef INC_CONTROLADOR_PID_H_
#define INC_CONTROLADOR_PID_H_

#include <stdint.h>

void    pid_inicializar(void);
/* Zera a memoria do termo D. Chame na largada. */
void    pid_resetar(int16_t erro_inicial);
/* Calcula a correcao para um erro novo (chamar uma vez por leitura nova). */
int16_t pid_calcular(int16_t erro);
/* Ultimos termos, para depuracao */
float   pid_termo_p(void);
float   pid_termo_d(void);

#endif /* INC_CONTROLADOR_PID_H_ */
