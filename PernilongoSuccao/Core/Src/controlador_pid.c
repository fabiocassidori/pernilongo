/*
 * controlador_pid.c
 */

#include "controlador_pid.h"
#include "config_robo.h"

static int16_t g_erro_anterior = 0;
static float   g_p = 0.0f, g_d = 0.0f;

void pid_inicializar(void)
{
    pid_resetar(0);
}

void pid_resetar(int16_t erro_inicial)
{
    /* Com o erro anterior igual ao atual o termo D da 1a leitura e zero:
     * evita um "chute" na largada. */
    g_erro_anterior = erro_inicial;
    g_p = g_d = 0.0f;
}

int16_t pid_calcular(int16_t erro)
{
    int32_t delta = (int32_t)erro - (int32_t)g_erro_anterior;
    g_erro_anterior = erro;

    g_p = PID_KP * (float)erro;
    g_d = PID_KD * (float)delta;
    float u = g_p + g_d;

    if (u >  (float)PID_LIMITE_SAIDA) u =  (float)PID_LIMITE_SAIDA;
    if (u < -(float)PID_LIMITE_SAIDA) u = -(float)PID_LIMITE_SAIDA;
    return (int16_t)u;
}

float pid_termo_p(void) { return g_p; }
float pid_termo_d(void) { return g_d; }
