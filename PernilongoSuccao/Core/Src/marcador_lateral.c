/*
 * LÓGICA DE CALIBRAÇÃO E DETECÇÃO INTERSECÇÕES
 */

#include "marcador_lateral.h"
#include "config_robo.h"
#include <stdio.h> // Incluído para depuração, se necessário

// --- NOVA MÁQUINA DE ESTADOS E ACUMULADOR ---
typedef enum {
    ESTADO_MARCADOR_OCIOSO,  // Robô está fora de qualquer marcação
    ESTADO_MARCADOR_EM_EVENTO // Robô detectou o início de uma marcação
} EstadoMarcador;

static EstadoMarcador g_estado_marcador = ESTADO_MARCADOR_OCIOSO;

// Esta variável acumula o tipo de marcador visto durante um único evento
// 1 = viu só esquerda, 2 = viu só direita, 3 = viu ambos em algum momento
static int g_tipo_marcador_acumulado = 0;

// Ponteiro para o buffer ADC e variáveis de calibração permanecem
static volatile uint16_t* g_adc_buffer = NULL;
static uint16_t g_calib_min_dir = ADC_VALOR_MAXIMO, g_calib_max_dir = 0;
static uint16_t g_calib_min_esq = ADC_VALOR_MAXIMO, g_calib_max_esq = 0;
static uint16_t g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
static uint16_t g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;

// Funções de inicialização e reset não precisam de mudanças
void marcador_lateral_resetar_calibracao(void) {
    g_calib_min_dir = (uint16_t)ADC_VALOR_MAXIMO; g_calib_max_dir = 0;
    g_calib_min_esq = (uint16_t)ADC_VALOR_MAXIMO; g_calib_max_esq = 0;
    g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
}

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
	g_adc_buffer = adc_dma_buffer;
	marcador_lateral_resetar_calibracao();
    g_estado_marcador = ESTADO_MARCADOR_OCIOSO;
    g_tipo_marcador_acumulado = 0;
}


/**
 * @brief Executa uma única iteração de leitura para a calibração.
 * Esta função deve ser chamada repetidamente enquanto o robô se move
 * sobre as superfícies branca e preta da pista.
 */
void marcador_lateral_calibrar_ciclo(void) {
    if (!g_adc_buffer) {
        return;
    }

    // Lê os valores brutos atuais dos sensores laterais a partir do buffer DMA
    // O sensor direito está no índice 0 e o esquerdo no índice 7 do buffer ADC
    uint16_t valor_atual_dir = g_adc_buffer[0];
    uint16_t valor_atual_esq = g_adc_buffer[7];

    // Atualiza os valores mínimo e máximo para o sensor direito
    if (valor_atual_dir < g_calib_min_dir) {
        g_calib_min_dir = valor_atual_dir;
    }
    if (valor_atual_dir > g_calib_max_dir) {
        g_calib_max_dir = valor_atual_dir;
    }

    // Atualiza os valores mínimo e máximo para o sensor esquerdo
    if (valor_atual_esq < g_calib_min_esq) {
        g_calib_min_esq = valor_atual_esq;
    }
    if (valor_atual_esq > g_calib_max_esq) {
        g_calib_max_esq = valor_atual_esq;
    }
}

/**
 * @brief Finaliza o processo de calibração.
 * Calcula os limiares finais com base nos valores min/max coletados.
 */
void marcador_lateral_finalizar_calibracao(void) {
    // Calcula o limiar para o sensor direito como o ponto médio
    // entre a leitura mínima (branco) e máxima (preto).
    if (g_calib_max_dir > g_calib_min_dir) {
        g_limiar_calibrado_dir = g_calib_min_dir + (g_calib_max_dir - g_calib_min_dir) / 2;
    } else {
        // Se a calibração não for eficaz (min e max iguais), recorre ao valor padrão.
        g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    }

    // Faz o mesmo para o sensor esquerdo.
    if (g_calib_max_esq > g_calib_min_esq) {
        g_limiar_calibrado_esq = g_calib_min_esq + (g_calib_max_esq - g_calib_min_esq) / 2;
    } else {
        g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
    }

     //Log dos valores calibrados para depuração
     printf("Calib. Lateral Esq Min/Max/Limiar: %u/%u/%u\r\n", g_calib_min_esq, g_calib_max_esq, g_limiar_calibrado_esq);
     printf("Calib. Lateral Dir Min/Max/Limiar: %u/%u/%u\r\n", g_calib_min_dir, g_calib_max_dir, g_limiar_calibrado_dir);
}


TipoMarcador marcador_lateral_verificar(void) {

	if (!g_adc_buffer) {
        return MARCADOR_NENHUM;
    }
	 bool dir_agora = (g_adc_buffer[0] < g_limiar_calibrado_dir);
	 bool esq_agora = (g_adc_buffer[7] < g_limiar_calibrado_esq);

    int estado_leitura_atual = 0;
    if (esq_agora) estado_leitura_atual |= 1; // Bit 0 para a esquerda
    if (dir_agora) estado_leitura_atual |= 2; // Bit 1 para a direita
//  Resultado: 0=Nenhum, 1=Esquerda, 2=Direita, 3=Ambos

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    switch (g_estado_marcador) {
        case ESTADO_MARCADOR_OCIOSO:
            // Se qualquer marcador for detectado, inicia um novo evento
            if (estado_leitura_atual > 0) {
                // Inicia o acumulador com o primeiro estado visto
                g_tipo_marcador_acumulado = estado_leitura_atual;
                // Muda para o estado de evento
                g_estado_marcador = ESTADO_MARCADOR_EM_EVENTO;
            }
            break;

        case ESTADO_MARCADOR_EM_EVENTO:
            // Se ainda estamos vendo qualquer parte de uma marca...
            if (estado_leitura_atual > 0) {
                // ...acumulamos o tipo de marcador.
                // A operação OU (OR) garante que se virmos "ambos" (3) uma vez,
                // ou se virmos "esquerda" (1) e "direita" (2) em momentos diferentes,
                // o resultado final será 3.
                g_tipo_marcador_acumulado |= estado_leitura_atual;
            }
            // Se não vemos mais nenhuma marca, o evento terminou.
            else {
                // Decide qual foi o marcador com base no valor acumulado
                if (g_tipo_marcador_acumulado == 3) {
                    marcador_para_retorno = MARCADOR_AMBOS;
                } else if (g_tipo_marcador_acumulado == 2) {
                    marcador_para_retorno = MARCADOR_DIREITA;
                } else if (g_tipo_marcador_acumulado == 1) {
                    marcador_para_retorno = MARCADOR_ESQUERDA;
                }

                // Reseta tudo para o próximo ciclo
                g_tipo_marcador_acumulado = 0;
                g_estado_marcador = ESTADO_MARCADOR_OCIOSO;
            }
            break;
    }

    return marcador_para_retorno;
}

void depuracao_sensores_laterais(){
	printf("Sensores Laterais | Esq: %u, Dir: %u\r\n", g_adc_buffer[7], g_adc_buffer[0]);
	HAL_Delay(500);
}
