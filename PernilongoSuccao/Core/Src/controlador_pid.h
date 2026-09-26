/*
 * controlador_pid.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_CONTROLADOR_PID_H_
#define INC_CONTROLADOR_PID_H_

void pid_inicializar(void);
int pid_calcular_correcao(int posicao_atual);
int pid_calcular_correcao_customizada(int posicao_atual, int kp, int kd);

#endif /* INC_CONTROLADOR_PID_H_ */
