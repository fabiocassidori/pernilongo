/*
 * sensor_linha.c
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 *
 * Adaptado: 12 sensores QRE1113 (antes 6), buffer DMA de NUM_CANAIS_ADC
 * posições (antes 8 fixas) e acionamento dos emissores IR por PA8.
 * A lógica de calibração, normalização e média ponderada é a original.
 *
 * PULSAGEM (SENSOR_PULSADO, ver config_robo.h): a cada leitura, os emissores
 * da régua são ligados e desligados, e a posição é calculada sobre a
 * DIFERENÇA (aceso - apagado) em vez do valor bruto do ADC. Isso cancela a
 * contribuição da luz ambiente (sol, iluminação de teto), que soma igualmente
 * nas duas fases e desaparece na subtração. Técnica padrão em seguidores de
 * linha de alta performance.
 */

#include "sensor_linha.h"
#include "config_robo.h"
#include "main.h"
#include "interface_usuario.h"
#include "marcador_lateral.h"
#include <stdio.h>
#include <stdlib.h>

static ADC_HandleTypeDef *g_hadc = NULL;
// Índice 0 = lateral direito, 1..NUM_SENSORES_LINHA = régua, último = lateral esquerdo
static volatile uint16_t g_adc_buffer[NUM_CANAIS_ADC];

// Leitura já condicionada: com SENSOR_PULSADO=1, é (aceso - apagado) para os
// QTR e o valor direto para os laterais; com SENSOR_PULSADO=0, é uma cópia
// simples de g_adc_buffer (comportamento antigo).
static uint16_t g_valores_condicionados[NUM_CANAIS_ADC];
#if SENSOR_PULSADO
static uint16_t g_dbg_aceso[NUM_CANAIS_ADC];
static uint16_t g_dbg_apagado[NUM_CANAIS_ADC];
#endif

static uint16_t g_valores_minimos[NUM_SENSORES_LINHA + 1];
static uint16_t g_valores_maximos[NUM_SENSORES_LINHA + 1];

static bool g_esta_na_linha = false;
static int g_ultimo_erro = 0;
static volatile int g_contador_linha_perdida_ms = 0;

static uint16_t interpolar(uint16_t valor, uint16_t entrada_min, uint16_t entrada_max, uint16_t saida_min, uint16_t saida_max);
static void ler_e_normalizar(uint16_t* valores_normalizados);
static void atualizar_leitura_condicionada(void);
#if SENSOR_PULSADO
void dwt_delay_init(void);      // weak — ver definição no fim do arquivo
void delay_us(uint32_t us);     // weak — ver definição no fim do arquivo
#endif

void sensor_linha_definir_emissores(bool ligar) {
    HAL_GPIO_WritePin(PWM_SENSOR_GPIO_Port, PWM_SENSOR_Pin, ligar ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void sensor_linha_inicializar(ADC_HandleTypeDef *hadc) {
    g_hadc = hadc;
    for (int i = 0; i <= NUM_SENSORES_LINHA; i++) {
        g_valores_minimos[i] = (uint16_t)ADC_VALOR_MAXIMO;
        g_valores_maximos[i] = 0;
    }
    g_ultimo_erro = SENSOR_POSICAO_CENTRO;
#if SENSOR_PULSADO
    dwt_delay_init();
    // Estado inicial aceso: a primeira leitura já alterna a partir daqui.
    sensor_linha_definir_emissores(true);
#elif EMISSORES_SEMPRE_ACESOS
    // Sem isto a régua fica apagada: os LEDs IR só acendem via Q5 (PA8).
    sensor_linha_definir_emissores(true);
#endif
    HAL_ADC_Start_DMA(g_hadc, (uint32_t*)g_adc_buffer, NUM_CANAIS_ADC);
}

void sensor_linha_calibrar(void) {
	// Reseta os valores de calibração
	for (int i = 0; i <= NUM_SENSORES_LINHA; i++) {
	g_valores_minimos[i] = (uint16_t)ADC_VALOR_MAXIMO;
	g_valores_maximos[i] = 0;
	}

	printf("Iniciando calibracao. Mova o robo sobre a linha...\r\n");

	// Bip inicial para marcar o começo
	iu_bip_bloqueante(100);
	HAL_Delay(100);

	// O loop de calibração dura (CICLOS_CALIBRACAO * DELAY_CALIBRACAO) ms
	for (uint16_t i = 0; i < CICLOS_CALIBRACAO; i++) {
	// A cada ciclo, atualiza os valores mínimos e máximos para cada sensor
	// começa em 1 pois g_valores_condicionados 1 a NUM_SENSORES_LINHA são os sensores QTR.
	// Usa a leitura CONDICIONADA (pulsada), para calibrar no mesmo regime
	// em que a corrida vai ler (ver SENSOR_PULSADO em config_robo.h).
	atualizar_leitura_condicionada();
	for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
		uint16_t valor_atual = g_valores_condicionados[j];
		if (valor_atual < g_valores_minimos[j]) {
			g_valores_minimos[j] = valor_atual;
		}
		if (valor_atual > g_valores_maximos[j]) {
			g_valores_maximos[j] = valor_atual;
		}
	}
		marcador_lateral_calibrar_ciclo();

		HAL_Delay(DELAY_CALIBRACAO); // Pausa de DELAY_CALIBRACAO(ms) a cada ciclo para que seja possivel mover o robo sobre a linha
	}

	printf("Calibracao finalizada.\r\n");

	// Log para exibir os resultados da calibração para cada sensor.
	// Índice 1 = QTR12 (direita) ... índice 12 = QTR01 (esquerda).
	printf("--- Resultados da Calibracao do Sensor de Linha ---\r\n");
		for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
			// Imprime o sensor (numeração da placa), o mínimo (branco) e o máximo (preto).
			printf("QTR%02d | Min: %u | Max: %u\r\n", NUM_SENSORES_LINHA + 1 - j, g_valores_minimos[j], g_valores_maximos[j]);
		}
	printf("---------------------------------------------------\r\n");


	iu_bip_bloqueante(300);
}

int sensor_linha_ler_posicao(void) {
    atualizar_leitura_condicionada(); // pulsa os emissores e cancela luz ambiente (se SENSOR_PULSADO=1)
    uint16_t normalizados[NUM_SENSORES_LINHA + 1];
    ler_e_normalizar(normalizados); // Normaliza de 0 a 1000

    g_esta_na_linha = false;

    unsigned long avg = 0; // Acumulador para o numerador
    unsigned long sum = 0; // Acumulador para o denominador
    int last_value = 0;    // Posição calculada

    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        long value = normalizados[i + 1]; // Nossos sensores começam no índice 1

        if (value > 50) { // Um limiar mínimo para considerar que o sensor vê a linha
            g_esta_na_linha = true;
        }

        // Se o sensor está vendo a linha, ele contribui para a média ponderada
        if (value > 50) {
            // O peso é a própria posição do sensor (i * 1000)
            avg += (unsigned long)value * (i * 1000);
            sum += value;
        }
    }

    if (!g_esta_na_linha) {
        // Se nenhum sensor vê a linha, decide para que lado o robô "provavelmente" saiu
        // e retorna o valor mínimo ou máximo. O centro é SENSOR_POSICAO_CENTRO
        // ((NUM_SENSORES_LINHA-1)*1000/2 = 5500 com 12 sensores).
        if (g_ultimo_erro < SENSOR_POSICAO_CENTRO) {
            return SENSOR_POSICAO_MIN; // Perdeu pela direita
        } else {
            return SENSOR_POSICAO_MAX; // Perdeu pela esquerda
        }
    }

    last_value = avg / sum;
    g_ultimo_erro = last_value; // Atualiza a última posição válida conhecida

    return last_value; // Retorna o valor bruto (entre SENSOR_POSICAO_MIN e SENSOR_POSICAO_MAX)
}



static void ler_e_normalizar(uint16_t* valores_normalizados) {
	for (uint8_t i = 1; i <= NUM_SENSORES_LINHA; i++) {
		uint16_t valor_cru = g_valores_condicionados[i];

		// Mantém o "clamp" para garantir que os valores estejam dentro do intervalo calibrado
		if (valor_cru < g_valores_minimos[i]) valor_cru = g_valores_minimos[i];
		if (valor_cru > g_valores_maximos[i]) valor_cru = g_valores_maximos[i];


#if SENSOR_PULSADO
		// g_valores_condicionados é |aceso-apagado| (ver atualizar_leitura_condicionada):
		// magnitude da MUDANÇA causada pelo LED. Tem polaridade OPOSTA ao valor cru
		// contínuo: quanto mais a superfície reflete (branco/linha), MAIOR a queda de
		// tensão no fototransistor ao acender o LED, logo MAIOR a magnitude medida.
		// Ou seja, aqui MÁXIMO = branco (linha) e MÍNIMO = preto — invertido em
		// relação à leitura contínua abaixo. Mantém a mesma saída final (1000=linha).
		valores_normalizados[i] = interpolar(valor_cru, g_valores_minimos[i], g_valores_maximos[i], 0, 1000);
#else
		// Leitura contínua (sem pulsagem): o valor mínimo (branco) é 1000 e o máximo (preto) é 0.
		valores_normalizados[i] = interpolar(valor_cru, g_valores_minimos[i], g_valores_maximos[i], 1000, 0);
#endif
	}
}

static uint16_t interpolar(uint16_t valor, uint16_t entrada_min, uint16_t entrada_max, uint16_t saida_min, uint16_t saida_max) {
    if (entrada_min == entrada_max) return saida_min;
    return (uint16_t)(((int32_t)(valor - entrada_min) * (saida_max - saida_min)) / (entrada_max - entrada_min) + saida_min);
}

volatile uint16_t* sensor_linha_obter_buffer_dma(void) {
    return g_adc_buffer;
}

void sensor_linha_tick_1ms(void) {
    // Decrementa o contador a cada milissegundo.
    // Esta função será chamada pela interrupção do TIM6 em main.c
    if (g_contador_linha_perdida_ms > 0) {
        g_contador_linha_perdida_ms--;
    }
}

bool sensor_linha_is_robo_fora_da_pista(int pos_mapeada) {
    // Verifica se a posição atual está fora do limiar aceitável.
    if (abs(pos_mapeada) > POS_LINHA_PERDIDA) {
        // Se estiver fora, incrementa o contador. O incremento de 2, enquanto
        // o decremento é 1, faz com que o contador suba efetivamente a cada 1ms
        // que o robô permanece fora da linha.
        if (g_contador_linha_perdida_ms < TEMPO_MAX_LINHA_PERDIDA_MS) {
            g_contador_linha_perdida_ms += 2;
        }
    } else {
        // Se o robô voltou para a linha, reseta o contador imediatamente.
        g_contador_linha_perdida_ms = 0;
    }

    // Retorna verdadeiro se o contador ultrapassou o tempo máximo permitido.
    return (g_contador_linha_perdida_ms >= TEMPO_MAX_LINHA_PERDIDA_MS);
}

void sensor_linha_depuracao(void) {
    printf("Dir:%4u |", g_valores_condicionados[IDX_LATERAL_DIREITO]);
    for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
        printf(" %4u", g_valores_condicionados[j]);
    }
    printf(" | Esq:%4u\r\n", g_valores_condicionados[IDX_LATERAL_ESQUERDO]);
}

#if SENSOR_PULSADO
/*
 * DWT (Data Watchpoint and Trace): contador de ciclos de 32 bits do
 * Cortex-M4, usado aqui só como base de tempo para o delay_us() abaixo.
 * Recurso interno do núcleo, não usa nenhum periférico do MCU nem exige
 * mudança no .ioc.
 */
/*
 * Não-static e "weak" só para permitir substituir por um mock ao rodar os
 * testes deste módulo no host (PC), onde os endereços do DWT não existem.
 * No firmware real (STM32), sem nenhuma outra definição no link, é esta
 * implementação que roda — comportamento idêntico a uma função static.
 */
__attribute__((weak)) void dwt_delay_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__attribute__((weak)) void delay_us(uint32_t us) {
    uint32_t ciclos = us * (SystemCoreClock / 1000000U); // SystemCoreClock = 168 MHz
    uint32_t inicio = DWT->CYCCNT;
    while ((DWT->CYCCNT - inicio) < ciclos) {
        // busy-wait: ~40us por leitura, dentro do orçamento do loop de 1ms
    }
}
#endif

/*
 * Núcleo da pulsagem. Preenche g_valores_condicionados[1..NUM_SENSORES_LINHA]
 * com (leitura_aceso - leitura_apagado) de cada QTR, cancelando a parcela de
 * luz ambiente (constante nas duas fases). Os 2 laterais (RPR-220, LED fixo,
 * independente de PA8) só são copiados, sem subtração.
 *
 * Cada fase espera PULSO_TEMPO_ESTAB_US antes de amostrar, tempo suficiente
 * para o fototransistor responder E o DMA do ADC1 completar uma varredura
 * nova dos 14 canais (ver a conta em config_robo.h).
 */
static void atualizar_leitura_condicionada(void) {
#if SENSOR_PULSADO
    sensor_linha_definir_emissores(true);
    delay_us(PULSO_TEMPO_ESTAB_US);
    for (uint8_t i = 0; i < NUM_CANAIS_ADC; i++) {
        g_dbg_aceso[i] = g_adc_buffer[i];
    }

    sensor_linha_definir_emissores(false);
    delay_us(PULSO_TEMPO_ESTAB_US);
    for (uint8_t i = 0; i < NUM_CANAIS_ADC; i++) {
        g_dbg_apagado[i] = g_adc_buffer[i];
    }

    sensor_linha_definir_emissores(true); // deixa aceso entre leituras (menor duty desligado, LED "descansa" por ~2*PULSO_TEMPO_ESTAB_US a cada 1ms)

    for (uint8_t i = 1; i <= NUM_SENSORES_LINHA; i++) {
        // Módulo, não "aceso - apagado" cru: no QRE1113 em coletor-comum com
        // pull-up (como nesta placa), MAIS reflexo (linha branca) faz o
        // fototransistor conduzir mais e a tensão do coletor CAIR — ou seja,
        // o sinal útil pode aparecer tanto em aceso>apagado quanto o
        // contrário, dependendo da malha/posição do sensor na régua. Usar o
        // módulo extrai a magnitude da contribuição do LED (o que importa)
        // sem depender de acertar essa polaridade a priori; a calibração,
        // feita sobre este mesmo valor, adapta min/max normalmente.
        int32_t diferenca = (int32_t)g_dbg_aceso[i] - (int32_t)g_dbg_apagado[i];
        if (diferenca < 0) diferenca = -diferenca;
        g_valores_condicionados[i] = (uint16_t)diferenca;
    }
    // Laterais: RPR-220 com LED sempre aceso, não afetado por PA8. Copia direto.
    g_valores_condicionados[IDX_LATERAL_DIREITO] = g_dbg_aceso[IDX_LATERAL_DIREITO];
    g_valores_condicionados[IDX_LATERAL_ESQUERDO] = g_dbg_aceso[IDX_LATERAL_ESQUERDO];
#else
    for (uint8_t i = 0; i < NUM_CANAIS_ADC; i++) {
        g_valores_condicionados[i] = g_adc_buffer[i];
    }
#endif
}

void sensor_linha_depuracao_pulso(void) {
#if SENSOR_PULSADO
    atualizar_leitura_condicionada();
    printf("QTR  aceso apagado diff\r\n");
    for (uint8_t i = 1; i <= NUM_SENSORES_LINHA; i++) {
        printf("%02u   %4u   %4u   %4u\r\n",
               NUM_SENSORES_LINHA + 1 - i, g_dbg_aceso[i], g_dbg_apagado[i], g_valores_condicionados[i]);
    }
#else
    printf("SENSOR_PULSADO=0: pulsagem desligada (config_robo.h)\r\n");
#endif
}
