/*
 * interface_usuario.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */


#include "interface_usuario.h"
#include "main.h"
#include "stm32f4xx_hal.h"

// Flags para controlar os "pedidos" para ligar a buzina
static volatile bool g_buzina_temporizada_ativa = false;
static volatile uint32_t g_buzina_tempo_parada = 0;
static volatile bool g_buzina_persistente_ativa = false;

void iu_inicializar(void) {
    // A inicialização do GPIO é feita pelo CubeMX em main.c
}

void iu_aguardar_botao(void) {
//    // Espera o botão ser solto (caso já esteja pressionado)
//    while (HAL_GPIO_ReadPin(BOTAO_GPIO_Port, BOTAO_Pin) == GPIO_PIN_RESET) {
//        iu_atualizar(); // Permite que a buzina funcione durante a espera
//        HAL_Delay(1);
//    }
    // Espera o botão ser pressionado
    while (HAL_GPIO_ReadPin(BOTAO_GPIO_Port, BOTAO_Pin) != GPIO_PIN_RESET) {
        iu_atualizar();
        HAL_Delay(1);
    }
    HAL_Delay(50); // Debounce

    // Espera o botão ser solto novamente
    while (HAL_GPIO_ReadPin(BOTAO_GPIO_Port, BOTAO_Pin) == GPIO_PIN_RESET) {
        iu_atualizar();
        HAL_Delay(1);
    }
    HAL_Delay(50); // Debounce
}

// Esta função agora APENAS registra o pedido para um bipe temporizado
void iu_buzina_temporizada(uint32_t duracao_ms) {
    g_buzina_tempo_parada = HAL_GetTick() + duracao_ms;
    g_buzina_temporizada_ativa = true;
}

// Esta função APENAS registra o pedido para o alarme persistente
void iu_definir_buzina_persistente(bool ligar) {
    g_buzina_persistente_ativa = ligar;
}

// Função que controla o pino da buzina.
// Ela centraliza a decisão e evita conflitos.
void iu_atualizar(void) {
    bool deve_estar_ligada = false;

    // Condição 1: A buzina deve estar ligada se o alarme persistente estiver ativo
    if (g_buzina_persistente_ativa) {
        deve_estar_ligada = true;
    }

    // Condição 2: A buzina deve estar ligada se um bipe temporizado estiver em andamento
    if (g_buzina_temporizada_ativa) {
        if (HAL_GetTick() < g_buzina_tempo_parada) {
            deve_estar_ligada = true;
        } else {
            // O tempo do bipe acabou, desativa a flag.
            g_buzina_temporizada_ativa = false;
        }
    }

    // Aplica o estado final ao pino GPIO
    if (deve_estar_ligada) {
        HAL_GPIO_WritePin(BUZINA_GPIO_Port, BUZINA_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BUZINA_GPIO_Port, BUZINA_Pin, GPIO_PIN_RESET);
    }
}

void iu_bip_bloqueante(uint32_t duracao_ms)
{
    // Esta função liga, espera e desliga, tudo em um só lugar.
    HAL_GPIO_WritePin(BUZINA_GPIO_Port, BUZINA_Pin, GPIO_PIN_SET);
    HAL_Delay(duracao_ms);
    HAL_GPIO_WritePin(BUZINA_GPIO_Port, BUZINA_Pin, GPIO_PIN_RESET);
}

void iu_led_verde(bool ligar)
{
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, ligar ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void iu_led_vermelho(bool ligar)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, ligar ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
