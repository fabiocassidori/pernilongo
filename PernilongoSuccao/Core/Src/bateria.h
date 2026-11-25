/*
 * bateria.h
 *
 * Monitor da bateria 2S pelo BAT_REF (PC1 = ADC2_IN11, divisor R22/R23).
 *
 * Usa o ADC2, separado do ADC1 dos sensores: a leitura da bateria nao mexe
 * na sequencia nem no tempo da pulsagem da regua. A conversao e disparada
 * num tick de 10 ms e lida no tick seguinte, entao o laco nunca espera o ADC.
 */

#ifndef INC_BATERIA_H_
#define INC_BATERIA_H_

#include <stdbool.h>
#include <stdint.h>

void     bateria_inicializar(void);
/* Chamar a cada 10 ms (loop lento). em_corrida escolhe o limiar com carga. */
void     bateria_tick_10ms(bool em_corrida);
float    bateria_tensao(void);        /* filtro rapido (tau ~80 ms): compensacao */
float    bateria_tensao_lenta(void);  /* filtro lento (tau ~0,6 s): alarme       */
bool     bateria_presente(void);      /* false = alimentado so pelo ST-LINK     */
bool     bateria_baixa(void);         /* alarme com histerese (apaga se a tensao voltar) */
uint16_t bateria_adc(void);
void     bateria_imprimir_diagnostico(void);  /* printf: bruto, mV no pino, Vbat, fator */

#endif /* INC_BATERIA_H_ */
