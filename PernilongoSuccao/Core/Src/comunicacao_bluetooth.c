/*
 * comunicacao_bluetooth.c
 */

#include "comunicacao_bluetooth.h"
#include "config_robo.h"
#include <string.h>

static UART_HandleTypeDef *g_huart = NULL;
static volatile uint8_t g_buffer_rx[BT_TAMANHO_BUFFER_RX];
static volatile uint16_t g_idx_escrita = 0;
static volatile uint16_t g_idx_leitura = 0;
static uint8_t g_byte_rx;

void bt_inicializar(UART_HandleTypeDef *huart) {
    g_huart = huart;
    g_idx_escrita = 0;
    g_idx_leitura = 0;
    HAL_UART_Receive_IT(g_huart, &g_byte_rx, 1);
}

void bt_enviar(const uint8_t *dados, uint16_t tamanho) {
    if (!g_huart || tamanho == 0) return;
    // Timeout curto: se o módulo não estiver presente, não trava o robô.
    HAL_UART_Transmit(g_huart, (uint8_t *)dados, tamanho, BT_TIMEOUT_TX_MS + tamanho);
}

void bt_enviar_texto(const char *texto) {
    bt_enviar((const uint8_t *)texto, (uint16_t)strlen(texto));
}

uint16_t bt_bytes_disponiveis(void) {
    return (uint16_t)((g_idx_escrita + BT_TAMANHO_BUFFER_RX - g_idx_leitura) % BT_TAMANHO_BUFFER_RX);
}

bool bt_ler_byte(uint8_t *byte) {
    if (g_idx_leitura == g_idx_escrita) return false;
    *byte = g_buffer_rx[g_idx_leitura];
    g_idx_leitura = (uint16_t)((g_idx_leitura + 1) % BT_TAMANHO_BUFFER_RX);
    return true;
}

void bt_callback_rx(UART_HandleTypeDef *huart) {
    if (huart != g_huart) return;
    uint16_t proximo = (uint16_t)((g_idx_escrita + 1) % BT_TAMANHO_BUFFER_RX);
    if (proximo != g_idx_leitura) {          // Buffer cheio: descarta o byte
        g_buffer_rx[g_idx_escrita] = g_byte_rx;
        g_idx_escrita = proximo;
    }
    HAL_UART_Receive_IT(g_huart, &g_byte_rx, 1);
}

void bt_callback_erro(UART_HandleTypeDef *huart) {
    if (huart != g_huart) return;
    // Erro de ruído/overrun (ex.: módulo desconectado): volta a escutar.
    HAL_UART_Receive_IT(g_huart, &g_byte_rx, 1);
}
