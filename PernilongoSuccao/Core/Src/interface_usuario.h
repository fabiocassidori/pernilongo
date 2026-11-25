/*
 * interface_usuario.h
 *
 * Botao (PB4, pull-up interno, 0 = apertado), buzina ativa (PB8) e LEDs
 * verde (PC8) e vermelho (PB9). Tudo sem bloquear: iu_atualizar() roda a
 * cada 1 ms e toca os padroes de bip e pisca os LEDs.
 */

#ifndef INC_INTERFACE_USUARIO_H_
#define INC_INTERFACE_USUARIO_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum { BOTAO_NADA = 0, BOTAO_CURTO, BOTAO_LONGO } evento_botao_t;
typedef enum { LED_APAGADO = 0, LED_ACESO, LED_PISCA_LENTO, LED_PISCA_RAPIDO } modo_led_t;

void           iu_inicializar(void);
void           iu_atualizar(uint32_t agora_ms);        /* a cada 1 ms */
evento_botao_t iu_botao_evento(void);                  /* consome o evento */
bool           iu_botao_apertado(void);

/* Bips (substituem o padrao em andamento) */
void iu_bip(uint16_t ms);
void iu_bip_duplo(uint16_t ms_on, uint16_t ms_off);    /* bip-bip */
void iu_bip_triplo(uint16_t ms_on, uint16_t ms_off);
void iu_bip_bloqueante(uint16_t ms);

/* LEDs: modo de fundo + "flash" temporario que tem prioridade (marcas) */
void iu_led_modo(modo_led_t verde, modo_led_t vermelho);
void iu_led_flash(bool verde, bool vermelho, uint16_t ms);

/* Alarme de bateria: LED vermelho aceso + bip periodico */
void iu_alarme_bateria(bool ativo, bool com_bip);

#endif /* INC_INTERFACE_USUARIO_H_ */
