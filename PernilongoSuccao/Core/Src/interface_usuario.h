/*
 * interface_usuario.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_INTERFACE_USUARIO_H_
#define INC_INTERFACE_USUARIO_H_

#include <stdint.h>
#include <stdbool.h>

void iu_inicializar(void);
void iu_aguardar_botao(void);
void iu_buzina_temporizada(uint32_t duracao_ms);
void iu_definir_buzina_persistente(bool ligar);
void iu_atualizar(void);
void iu_bip_bloqueante(uint32_t duracao_ms);
/** LEDs da placa: verde (D2, PC8) e vermelho (D3, PB9). Ativos em nível alto. */
void iu_led_verde(bool ligar);
void iu_led_vermelho(bool ligar);

#endif /* INC_INTERFACE_USUARIO_H_ */
