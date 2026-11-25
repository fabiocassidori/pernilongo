/*
 * config_robo.h
 *
 * Pernilongo 5 (STM32F405RG, 168 MHz) - seguidor de linha (robotrace) sem encoder.
 * Autor original: Fabio Couto. Versao unificada: EVA-RT + FujitoraBot + reRo.
 *
 * TODOS os parametros ajustaveis do robo ficam aqui. Cada bloco diz de onde
 * veio o valor (EVA-RT, teste validado do projeto ou escolha nova) e o que
 * acontece se ele for alterado. A wiki (pasta wiki/) explica cada um.
 */

#ifndef INC_CONFIG_ROBO_H_
#define INC_CONFIG_ROBO_H_

#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 * 1. SENSORES DE LINHA (12x QRE1113, pull-up de 22k, emissores via Q5/PA8)
 * ========================================================================== */
#define NUM_SENSORES_LINHA        12
#define NUM_CANAIS_ADC            14      /* 12 QTR + 2 laterais, ordem do .ioc */
#define ADC_VALOR_MAXIMO          1023    /* ADC1 em 10 bits */

/* Linha BRANCA sobre fundo PRETO (robotrace). 0 = linha preta em fundo branco. */
#define QTR_LINHA_BRANCA          1

/* Tempo de amostragem do ADC (ciclos de ADCCLK = 21 MHz).
 * QTR: saida de 22k -> 56 ciclos (validado em main_qtr_pulsado.c).
 * Laterais: pull-up de 10k, mais lentos -> 144 ciclos (validado em main_sl.c).
 * 1 varredura = 12 x (56+10) + 2 x (144+10) = 1100 ciclos = 52,4 us. */
#define QTR_ADC_SMP_CICLOS        56
#define LATERAL_ADC_SMP_CICLOS    144

/* Leitura pulsada: apagado -> aceso -> apagado (a luz ambiente se cancela).
 * Cada fase espera o fototransistor estabilizar e depois soma QTR_AMOSTRAS
 * varreduras. Uma leitura completa sai a cada ~1,1 ms. */
#define QTR_AMOSTRAS              4       /* varreduras somadas por fase */
#define QTR_ESPERA_AUTO           1       /* 1 = mede o tempo de resposta na calibracao */
#define QTR_ESPERA_PADRAO_US      300     /* usada se a medicao falhar ou AUTO = 0 */
#define QTR_ESPERA_MIN_US         20
#define QTR_ESPERA_MAX_US         1500
#define QTR_DEGRAU_MIN            80      /* degrau minimo (LSB) para medir um sensor */
#define QTR_TOLERANCIA_PCT        5       /* estabilizado = dentro de 5 % do valor final */

/* Filtro de mediana de 3 leituras (no tempo), por sensor. Remove picos
 * isolados (ruido do PWM dos motores) com atraso de so 1 leitura. */
#define QTR_FILTRO_MEDIANA        1

/* Calibracao (estilo Pololu calibrate()). */
#define CALIBRACAO_DURACAO_MS     5000    /* passe a regua sobre a linha e o fundo */
#define QTR_CONTRASTE_MIN         100     /* max - min abaixo disso = sensor reprovado */

/* Limiar do EVA-RT (SENSOR_THRESHOLD_PERCENT = 75): so os sensores acima de
 * 75 % do caminho entre fundo e linha entram na media ponderada. Valores
 * calibrados de 0 a 1000, entao 750. Use 50 para o comportamento Pololu. */
#define QTR_LIMIAR_LINHA          750

/* Cruzamento pela regua (EVA-RT CORRECT_CROSS_ERROR, CROSS_SENSOR_OFSET = 3):
 * se os sensores 2 e 9 veem linha ao mesmo tempo, o erro vira 0 (segue reto). */
#define QTR_CRUZ_SENSOR_A         2
#define QTR_CRUZ_SENSOR_B         9
#define QTR_CRUZ_MIN_SENSORES     8       /* ou: 8 ou mais sensores na linha */

/* Posicao: 0 = sob o QTR01 (extrema ESQUERDA) ... 11000 = sob o QTR12 (DIREITA) */
#define POSICAO_MAX               ((NUM_SENSORES_LINHA - 1) * 1000)
#define POSICAO_CENTRO            (POSICAO_MAX / 2)

/* =============================================================================
 * 2. CONTROLE PD (valores do EVA-RT v1.2.2, mesma placa, motores e reducao)
 * ========================================================================== */
/* u = KP*erro + KD*(erro - erro_anterior), erro = posicao - 5500 (-5500..5500)
 * EVA-RT 1.2.2 (ACRC 2025):  KP 0.06  KD 0.6  SPEED 40  IMP 180
 * EVA-RT LITE 1.0:           KP 0.04  KD 0.6  SPEED 60  IMP 215          */
#define PID_KP                    0.06f
#define PID_KD                    0.6f
#define PID_LIMITE_SAIDA          510     /* EVA-RT: constrain(-510, 510) */

/* =============================================================================
 * 3. VELOCIDADE (malha aberta, sem encoder: reta = curva)
 * ========================================================================== */
#define POTENCIA_MAX_MOTOR        255     /* escala 8 bits, igual ao analogWrite do EVA-RT */
#define VELOCIDADE_BASE           40      /* EVA-RT 1.2.2 SPEED = 40 */
#define VELOCIDADE_PASSO          5       /* EVA-RT: botao 2 soma +5 */
#define VELOCIDADE_MAXIMA         140     /* EVA-RT: _speed_ < 140 */
#define RAMPA_PARTIDA_MS          50      /* EVA-RT LITE SPEED_TIME_MS = 50 */

/* Fora da linha (EVA-RT "OUT METHOD"): gira forte para o ultimo lado visto. */
#define FORA_POTENCIA_EXTERNA     255     /* EVA-RT OUT_OUTTER_SPEED */
#define FORA_POTENCIA_INTERNA     (-255)  /* EVA-RT OUT_INNER_SPEED  */
#define TEMPO_MAX_FORA_LINHA_MS   30      /* EVA-RT usa 10 ms; 30 da mais folga */

/* Frenagem no fim (EVA-RT): -SPEED por STOP_TIME_MS, depois desliga. */
#define TEMPO_FRENAGEM_MS         75
#define TEMPO_SUCCAO_APOS_FIM_MS  300     /* EVA-RT: STOP_TIME_MS * 4 */

/* =============================================================================
 * 4. MOTORES DE TRACAO (2x IFX9201SG, TIM8 20 kHz) - validados em motores/main.c
 * ========================================================================== */
#define PERIODO_PWM_MOTOR         8399    /* = TIM8.Period: 168 MHz / 8400 = 20 kHz */
#define MOTOR_TROCAR_LADOS        0
#define MOTOR_ESQ_DIR_FRENTE      1
#define MOTOR_DIR_DIR_FRENTE      0

/* =============================================================================
 * 5. SUCCAO (Q3 IRLML6344, PA15 = TIM2_CH1, 20 kHz) - validada em vacuo/
 * ========================================================================== */
#define SUCCAO_HABILITADA         1       /* 0 = nunca liga (teste na mao) */
#define PERIODO_PWM_SUCCAO        4199    /* = TIM2.Period: 84 MHz / 4200 = 20 kHz */
#define SUCCAO_MAX_PCT            90      /* limite de seguranca do teste de 90 % */
/* Potencia segue a velocidade como no EVA-RT: IMP = 180 + 14 a cada +5 de SPEED
 * (PWM de 0 a 255). Em %: 180/255 = 70,6 %. Limitada a SUCCAO_MAX_PCT. */
#define SUCCAO_VEL_REF            40      /* velocidade em que IMP = SUCCAO_IMP_BASE */
#define SUCCAO_IMP_BASE           180
#define SUCCAO_IMP_PASSO          14
#define SUCCAO_IMP_MAX            245     /* EVA-RT MAX_IMP_SPEED */
#define SUCCAO_RAMPA_PASSO_PCT    5       /* +5 % a cada 10 ms (0 -> 70 % em 140 ms) */
#define TEMPO_MAOS_FORA_MS        1000    /* EVA-RT START_WAIT_MS: tirar a mao */
#define TEMPO_SUCCAO_ANTES_MS     1000    /* EVA-RT START_DELAY_MS: forma o vacuo */
/* Compensa a queda da bateria: duty = pct * V_REF / V_bat (forca de sucao constante) */
#define SUCCAO_COMPENSAR_BATERIA  1
#define SUCCAO_V_REFERENCIA       7.8f

/* =============================================================================
 * 6. SENSORES LATERAIS (marcas) - PC0 = SLDIR (rank 1), PB1 = SLESQ (rank 14)
 * ========================================================================== */
#define IDX_LATERAL_DIREITO       0
#define IDX_LATERAL_ESQUERDO      13
#define LIMIAR_SENSOR_LATERAL     600     /* < limiar = marca branca (ajuste empirico) */
#define HISTERESE_LATERAL         40      /* sai da marca so acima de limiar + 40 */
#define LATERAL_FILTRO_MS         2       /* estado novo precisa durar 2 ms (main_sl.c) */
#define MARCADORES_DIREITA_FIM    2       /* para na 2a marca direita (largada + chegada) */
#define TEMPO_MIN_ATE_CHEGADA_MS  1000    /* EVA-RT: so aceita a chegada apos 1 s */
#define JANELA_CRUZAMENTO_MS      60      /* EVA-RT CROSS_WARNING_TIME_MS */
/* Marca ignorada se a linha estiver muito fora do centro (o sensor lateral pode
 * estar vendo a propria linha numa curva fechada). 0 = desliga a protecao. */
#define MARCA_ERRO_MAX            4000

/* =============================================================================
 * 7. BATERIA (BAT_REF = PC1 = ADC2_IN11, divisor R22/R23)
 * ========================================================================== */
#define BAT_R_SUPERIOR_OHM        18000.0f /* R22 (na PCB original e 15k: satura em 8,25 V) */
#define BAT_R_INFERIOR_OHM        10000.0f /* R23 */
#define BAT_VDDA                  3.3f
/* Ajuste fino: (tensao no multimetro) / (bat_tensao_calc no Live Expressions).
 * Ex.: multimetro 8,40 V e firmware 8,25 V -> 8,40/8,25 = 1,018. Fica entre 0,9 e 1,1;
 * se der fora disso o problema e de hardware (resistor errado ou solda), nao de ajuste. */
#define BAT_FATOR_AJUSTE          1.000f
#define BAT_LIMIAR_BAIXA_V        7.0f    /* 2S: 3,5 V/celula em repouso */
#define BAT_LIMIAR_BAIXA_CORRIDA_V 6.4f   /* com motores e sucao a bateria cai mais */
#define BAT_HISTERESE_V           0.3f    /* alarme so apaga acima de limiar + 0,3 V */
#define BAT_TEMPO_CONFIRMA_MS     2000    /* abaixo do limiar por 2 s = alarme */
#define BAT_TEMPO_LIBERA_MS       3000    /* acima de limiar + histerese por 3 s = apaga */
#define BAT_TEMPO_PARTIDA_MS      1000    /* ignora o 1o segundo (filtro assentando) */
#define BAT_INTERVALO_ALARME_MS   1500    /* bip de alarme a cada 1,5 s */
#define BAT_ALARME_HABILITADO     0       /* 0 = so mede e mostra, nunca bipa (diagnostico) */
#define BAT_DIAGNOSTICO_PRINTF    1       /* 1 = imprime bruto/mV/V a cada 2 s fora da corrida */
#define BAT_ALARME_NA_CORRIDA     1       /* 1 = bipa tambem durante a corrida */

/* =============================================================================
 * 8. INTERFACE (botao PB4 com pull-up interno, LEDs PC8/PB9, buzina PB8)
 * ========================================================================== */
#define BOTAO_DEBOUNCE_MS         30
#define BOTAO_LONGO_MS            1000
#define BIP_MARCA_MS              60
#define LED_MARCA_MS              150
#define BIP_CRUZ_MS               30      /* intersecao: bip-bip (30 on, 40 off, 30 on) */
#define BIP_CRUZ_PAUSA_MS         40

/* =============================================================================
 * 9. BLUETOOTH (ignorado por enquanto; o syscalls.c usa estas definicoes)
 * ========================================================================== */
#define BT_TAMANHO_BUFFER_RX      64
#define BT_TIMEOUT_TX_MS          20
/* 0 = printf so no SWV/ITM. Com 1, cada caractere leva ~1 ms a 9600 baud. */
#define BT_ESPELHAR_PRINTF        0

#endif /* INC_CONFIG_ROBO_H_ */
