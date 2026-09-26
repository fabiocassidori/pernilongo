/*
 * controle_velocidade.c
 *
 * Created on: Jun 5, 2025
 * Author: Fábio Couto
 */


#include "controle_velocidade.h"
#include "leitor_encoder.h"
#include "config_robo.h"

static int32_t g_pos_anterior_esq = 0;
static int32_t g_pos_anterior_dir = 0;

static float g_velocidade_media_mmps = 0.0f;

// +++ VARIÁVEIS PARA O PID DE VELOCIDADE +++
static float g_erro_anterior_vel = 0.0f;
static float g_erro_integral_vel = 0.0f;

void velocidade_inicializar(void) {
    g_pos_anterior_esq = encoder_obter_posicao_esquerda();
    g_pos_anterior_dir = encoder_obter_posicao_direita();
}

void velocidade_atualizar(void) {
    int32_t pos_atual_dir = encoder_obter_posicao_direita();
    int32_t pos_atual_esq = encoder_obter_posicao_esquerda();

    int32_t delta_pulsos_dir = pos_atual_dir - g_pos_anterior_dir;
    int32_t delta_pulsos_esq = pos_atual_esq - g_pos_anterior_esq;

    int32_t delta_pulsos_media = (delta_pulsos_dir + delta_pulsos_esq) / 2;

    const float dist_por_pulso_mm = (DIAMETRO_RODA_MM * CONSTANTE_PI) / (float)ENCODER_PPR;

    g_velocidade_media_mmps = ((float)delta_pulsos_media * dist_por_pulso_mm) / INTERVALO_LOOP_CONTROLE_S;

    g_pos_anterior_dir = pos_atual_dir;
    g_pos_anterior_esq = pos_atual_esq;
}



// +++ FUNÇÃO: CONTROLADOR PID DE VELOCIDADE +++
/**
 * @brief Calcula a potência base do motor para atingir uma velocidade alvo.
 * @param velocidade_alvo_mmps A velocidade desejada em mm/s.
 * @return A potência (0-255) a ser usada como base para os motores.
 */
int velocidade_controlar_potencia(float velocidade_alvo_mmps) {
    // A função velocidade_atualizar() já deve ter sido chamada neste ciclo.
    float velocidade_atual_mmps = g_velocidade_media_mmps;

    // 1. Cálculo do Erro
    float erro = velocidade_alvo_mmps - velocidade_atual_mmps;

    // 2. Termo Proporcional
    float termo_p = (float)erro * PID_VEL_KP;

    // 3. Termo Integral (com anti-windup)
    g_erro_integral_vel += erro;
    if (g_erro_integral_vel > PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = PID_VEL_LIMITE_INTEGRAL;
    if (g_erro_integral_vel < -PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = -PID_VEL_LIMITE_INTEGRAL;
    float termo_i = (float)g_erro_integral_vel * PID_VEL_KI;

    // 4. Termo Derivativo
    float derivativo = erro - g_erro_anterior_vel;
    g_erro_anterior_vel = erro;
    float termo_d = (float)derivativo * PID_VEL_KD;

    // 5. Soma e normalização
    int32_t saida_pid = (int32_t)((termo_p + termo_i + termo_d) / PID_VEL_DIVISOR);

    // 6. Limita a saída para o range de potência do motor (0 a 255)
    if (saida_pid > POTENCIA_MAX_MOTOR) saida_pid = POTENCIA_MAX_MOTOR;
    if (saida_pid < 0) saida_pid = 0;

    return (int)saida_pid;
}

float velocidade_obter_media_mmps(void) {
    return g_velocidade_media_mmps;
}
