/*
 * config_robo.h
 *
 * Created on: Jun 5, 2025
 * Author: Fabio Couto
 *
 * Adaptado para o hardware EVA-RT / Pernilongo (STM32F405 + 2x IFX9201SG).
 * Removidos: encoders, controle de velocidade em malha fechada, mapeamento
 * de pista e monitor de bateria (nao existem no circuito atual).
 */

#ifndef INC_CONFIG_ROBO_H_
#define INC_CONFIG_ROBO_H_

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURAÇÕES GERAIS
// =============================================================================
/*
 * A régua tem 12 sensores QRE1113 (J4 pinos 4..15 = QTR01..QTR12).
 * Os rótulos QTR13 (PC2) e QTR14 (PC1) existem no esquemático, mas NÃO chegam
 * ao conector J4 — por isso não são lidos.
 */
#define NUM_SENSORES_LINHA 12

/* Quantidade de leituras para calibrar o sensor de linha
  Para achar o tempo em milissegungos é só multiplicar por CICLOS_CALIBRACAO x DELAY_CALIBRACAO
 */
#define CICLOS_CALIBRACAO 100
#define DELAY_CALIBRACAO 50
#define INTERVALO_LOOP_CONTROLE_S 0.001f  // Intervalo em segundos (TIM6 = 1 ms)

// =============================================================================
// ORGANIZAÇÃO DO BUFFER DO ADC1 (DMA)
// =============================================================================
/*
 * Mesma convenção do código original: o sensor lateral DIREITO fica no índice 0,
 * os sensores de linha nos índices 1..NUM_SENSORES_LINHA e o lateral ESQUERDO
 * no último índice.
 *
 * Os sensores de linha entram na sequência do ADC do mais à DIREITA para o
 * mais à ESQUERDA (QTR12 ... QTR01), preservando a convenção original:
 *   SENSOR_POSICAO_MIN = extrema direita, SENSOR_POSICAO_MAX = extrema esquerda.
 * Assim o sinal do erro e do PID permanece idêntico ao código validado.
 *
 *  Rank | Canal | Pino | Sinal  | Índice
 *    1  | IN10  | PC0  | SLDIR  |   0   (lateral direito, RPR-220)
 *    2  | IN13  | PC3  | QTR12  |   1   (extrema direita)
 *    3  | IN0   | PA0  | QTR11  |   2
 *    4  | IN1   | PA1  | QTR10  |   3
 *    5  | IN2   | PA2  | QTR09  |   4
 *    6  | IN3   | PA3  | QTR08  |   5
 *    7  | IN4   | PA4  | QTR07  |   6
 *    8  | IN5   | PA5  | QTR06  |   7
 *    9  | IN6   | PA6  | QTR05  |   8
 *   10  | IN7   | PA7  | QTR04  |   9
 *   11  | IN14  | PC4  | QTR03  |  10
 *   12  | IN15  | PC5  | QTR02  |  11
 *   13  | IN8   | PB0  | QTR01  |  12   (extrema esquerda)
 *   14  | IN9   | PB1  | SLESQ  |  13   (lateral esquerdo, RPR-220)
 */
#define NUM_CANAIS_ADC            (NUM_SENSORES_LINHA + 2)
#define IDX_LATERAL_DIREITO       0
#define IDX_LATERAL_ESQUERDO      (NUM_SENSORES_LINHA + 1)

// =============================================================================
// CONFIGURAÇÕES DOS SENSORES
// =============================================================================
#define ADC_VALOR_MAXIMO 1023.0f
//TRATAMENTO DE VALORES
#define SENSOR_POSICAO_MIN   0 // EXTREMA DIREITA DO SENSOR DE LINHA
#define SENSOR_POSICAO_MAX   ((NUM_SENSORES_LINHA - 1) * 1000) // EXTREMA ESQUERDA (12 sensores = 11000)
#define SENSOR_POSICAO_CENTRO (SENSOR_POSICAO_MAX / 2)
#define POSICAO_ENTRADA_MIN  -255 //INTERPOLACAO - MAXIMA ESQUERDA
#define POSICAO_ENTRADA_MAX  255 //INTERPOLACAO - MAXIMA DIREITA

// --- Sensores Laterais (Marcadores) ---
 /* LIMIAR_SENSOR_LATERAL só é usado quando há falha na calibração.
 * calibrando os sensores laterais, usa-se valor mínimo + (valor maximo - valor minimo)/2
 */
#define LIMIAR_SENSOR_LATERAL 450    // Valor ADC para considerar uma marcação detectada

// Fim de corrida: marcação direita da largada + marcação direita da chegada
#define MARCADORES_DIREITA_FIM_CORRIDA 2

// =============================================================================
// CONFIGURAÇÕES DOS MOTORES (IFX9201SG)
// =============================================================================
/*
 * Sem encoders, a velocidade é controlada em MALHA ABERTA: potência base fixa
 * (0..POTENCIA_MAX_MOTOR) somada à correção do PID de direção.
 * Valor inicial conservador; ajuste em pista.
 */
#define POTENCIA_BASE_CORRIDA 80

#define POTENCIA_MAX_MOTOR 255
/*
 * O IFX9201SG aceita PWM de no máximo 20 kHz (datasheet, P_6.0.33).
 * O código antigo usava 50 kHz (period 3359), adequado ao TB6612 mas acima do
 * limite do IFX9201SG. TIM8 = 168 MHz / (8399+1) = 20 kHz.
 * Se os motores aquecerem ou a resposta em baixa potência for ruim, teste
 * 10 kHz (16799) — o IFX9201SG tem slew rate baixo e perde parte do pulso
 * em frequências altas. Mantenha este valor igual a TIM8.Period no .ioc.
 */
#define PERIODO_PWM_MOTOR 8399

/*
 * O IFX9201SG NÃO tem modo de freio ativo no controle PWM/DIR: com PWM=0 o
 * motor fica em roda livre. A frenagem é feita por CONTRA-CORRENTE (DIR
 * invertido) com esta potência, pelo tempo TEMPO_FRENAGEM_MS.
 */
#define POTENCIA_FRENAGEM 150
#define TEMPO_FRENAGEM_MS 50

/*
 * Qual driver aciona qual roda. Pela posição na placa (vista de cima, frente
 * para cima): U3/J3 (PWMA-PC6, DIRA-PB12) fica do lado DIREITO e U2/J6
 * (PWMB-PC7, DIRB-PB14) do lado ESQUERDO.
 * CONFIRME NA BANCADA: se a roda errada girar, mude para 1.
 */
#define MOTOR_TROCAR_LADOS 0

/*
 * Nível do pino DIR que faz cada roda girar PARA FRENTE.
 * Tabela-verdade: DIR=1 -> OUT1=H/OUT2=L; DIR=0 -> OUT1=L/OUT2=H.
 * Como os motores ficam espelhados, normalmente um lado precisa do nível
 * oposto ao outro. CONFIRME NA BANCADA: se uma roda girar para trás,
 * inverta o valor dela (0 <-> 1) — não é preciso inverter fios.
 */
#define MOTOR_ESQ_DIR_FRENTE 1
#define MOTOR_DIR_DIR_FRENTE 0

// --- PID DE DIREÇÃO ---
#define PID_SETPOINT 0
#define PID_KP 310 //270//310
#define PID_KD 2900 //2500 //2900
#define PID_KI 0

// ---  PID EM RETAS ---
// Mantido para uso futuro. Sem mapeamento (sem encoders) não há como
// identificar retas, então o loop de corrida usa apenas PID_KP/PID_KD.
#define PID_KP_RETA 190
#define PID_KD_RETA 4500

#define PID_DIVISOR 100
/*
 * Se PID_LIMITE_SAIDA = 150, CORREÇAO MÁXIMA É IGUAL A 150
 * LOGO SE V_CURVA É 80 -> 80 + 150 = POTENCIA MAXIMA É 230,
 * OU SEJA, NAO SATURA OS 8BITS.
 * EM VRETA NÃO TEM PROBLEMA SATURAR
 */
#define PID_LIMITE_SAIDA 150

// =============================================================================
// CONFIGURAÇÕES DE SEGURANÇA
// =============================================================================
#define TEMPO_MAX_LINHA_PERDIDA_MS 100
#define POS_LINHA_PERDIDA 250

// =============================================================================
// CONFIGURAÇÕES DA SUCÇÃO (Q3 - IRLML6344, PA15 = TIM2_CH1)
// =============================================================================
#define SUCCAO_HABILITADA 1          // 0 = nunca liga a sucção (testes na mão)
#define POTENCIA_SUCCAO_PCT 80       // Potência do ventilador na corrida (0..100 %)
#define SUCCAO_RAMPA_PASSO_PCT 5     // Rampa de partida: +5 % a cada 10 ms (0->80 % em 160 ms)
/* Espera entre o botão e a largada (valor original: 1000 ms). A sucção liga
 * no botão e sobe em rampa durante esta espera, formando o vácuo antes de largar. */
#define TEMPO_ESPERA_LARGADA_MS 1000
/* TIM2 = 84 MHz / (4199+1) = 20 kHz (inaudível). Igual a TIM2.Period no .ioc. */
#define PERIODO_PWM_SUCCAO 4199

// =============================================================================
// EMISSORES DA RÉGUA (Q5 - IRLML6344, PA8 = PWM_SENSOR)
// =============================================================================
/*
 * Na placa atual os LEDs IR da régua só acendem via Q5 (J4 pino 16).
 * Se PA8 ficar em nível baixo a régua não enxerga nada.
 *
 * PULSAGEM (rejeição de luz ambiente): técnica padrão em seguidores de linha
 * de alta performance (ex.: EVA-RT, vencedor All Chile 2025 — o mesmo Q5/PA8
 * desta placa já nasceu pensado para pulsar os emissores). O princípio:
 *   1. liga o LED, espera estabilizar, lê o sensor (valor = luz do LED + ambiente)
 *   2. desliga o LED, espera estabilizar, lê de novo (valor = só luz ambiente)
 *   3. usa a DIFERENÇA -> cancela a luz ambiente (sol, lâmpadas, etc.)
 * Isso é feito a cada leitura de posição (1 kHz), de forma transparente para
 * o resto do código — calibração, PID e o restante do firmware não mudam.
 *
 * Só os 12 QRE1113 da régua são pulsados (cátodo comum no MOSFET Q5). Os 2
 * sensores laterais são RPR-220 com LED sempre ligado por resistor próprio
 * (R14/R17), fisicamente independentes de PA8 — não pulsam e não precisam.
 */
#define SENSOR_PULSADO 1
/*
 * Tempo de espera após cada troca de estado do LED, antes de amostrar.
 * Soma dois fatores, ambos já levantados nesta conversa:
 *   - resposta do fototransistor QRE1113 (datasheet): ~10 us
 *   - tempo para o DMA do ADC1 completar 1 varredura nova dos 14 canais:
 *     ADC a 21 MHz (PCLK2/4), 15+11 ciclos por canal (10 bits) x 14 canais
 *     = ~17.3 us
 * 40 us cobre os dois com folga. Ajuste aqui se notar ruído na posição.
 */
#define PULSO_TEMPO_ESTAB_US 40

/*
 * Usado somente se SENSOR_PULSADO=0 (fallback ao comportamento antigo:
 * emissores sempre acesos, sem cancelamento de luz ambiente).
 */
#define EMISSORES_SEMPRE_ACESOS 1

// =============================================================================
// BLUETOOTH (J5 - USART1: PA9 = TX, PA10 = RX)
// =============================================================================
#define BT_TAMANHO_BUFFER_RX 64
#define BT_TIMEOUT_TX_MS 20
/*
 * 1 = tudo que sai pelo printf (SWO) também é enviado pelo Bluetooth.
 * Atenção: a 9600 baud cada caractere leva ~1 ms. O printf só é usado fora
 * da corrida, então o loop de 1 ms não é afetado.
 */
#define BT_ESPELHAR_PRINTF 1

#endif
