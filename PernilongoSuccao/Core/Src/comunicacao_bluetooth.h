/*
 * comunicacao_bluetooth.h
 *
 * Módulo Bluetooth serial (HC-05 / HM-10 ou similar) no conector J5:
 *   J5.1 = PA9  = USART1_TX  -> ligar no RXD do módulo
 *   J5.2 = PA10 = USART1_RX  <- ligar no TXD do módulo
 *   J5.3 = GND
 * Velocidade definida no .ioc (USART1 = 9600 baud, padrão de fábrica do HC-05).
 *
 * Preparado para uso: transmissão bloqueante com timeout e recepção por
 * interrupção em buffer circular. Nenhum comando é interpretado ainda.
 */

#ifndef INC_COMUNICACAO_BLUETOOTH_H_
#define INC_COMUNICACAO_BLUETOOTH_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void bt_inicializar(UART_HandleTypeDef *huart);
void bt_enviar(const uint8_t *dados, uint16_t tamanho);
void bt_enviar_texto(const char *texto);
/** Quantos bytes recebidos aguardam leitura. */
uint16_t bt_bytes_disponiveis(void);
/** Lê um byte recebido. Retorna false se não houver dados. */
bool bt_ler_byte(uint8_t *byte);
/** Chamada pelo callback HAL_UART_RxCpltCallback (main.c). */
void bt_callback_rx(UART_HandleTypeDef *huart);
/** Chamada pelo callback HAL_UART_ErrorCallback (main.c): reinicia a recepção. */
void bt_callback_erro(UART_HandleTypeDef *huart);

#endif /* INC_COMUNICACAO_BLUETOOTH_H_ */
