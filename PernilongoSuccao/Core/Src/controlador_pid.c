/*
 * controlador_pid.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#include "controlador_pid.h"
#include "config_robo.h"
#include <stdint.h>

static int g_ultimo_erro_pid = 0;

static int calcular(int posicao_atual, int kp, int kd, int ki, int divisor, int* ptr_ultimo_erro);

void pid_inicializar(void) {
    g_ultimo_erro_pid = 0;
}

int pid_calcular_correcao(int posicao_atual) {
    return calcular(posicao_atual, PID_KP, PID_KD, PID_KI, PID_DIVISOR, &g_ultimo_erro_pid);
}

int pid_calcular_correcao_customizada(int posicao_atual, int kp, int kd) {
    return calcular(posicao_atual, kp, kd, 0, PID_DIVISOR, &g_ultimo_erro_pid);
}

static int calcular(int posicao_atual, int kp, int kd, int ki, int divisor, int* ptr_ultimo_erro) {
    int erro = posicao_atual - PID_SETPOINT;
    int derivativo = erro - *ptr_ultimo_erro;
    *ptr_ultimo_erro = erro;

    int32_t termo_p = (int32_t)erro * kp;
    int32_t termo_d = (int32_t)derivativo * kd;

    int correcao = (int)((termo_p + termo_d) / divisor);

    if (correcao > PID_LIMITE_SAIDA) correcao = PID_LIMITE_SAIDA;
    if (correcao < -PID_LIMITE_SAIDA) correcao = -PID_LIMITE_SAIDA;

    return correcao;
}
