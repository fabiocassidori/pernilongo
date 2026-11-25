/*
 * robo.c
 *
 * Tres "lacos" separados por tempo, todos chamados de robo_executar():
 *   - leitura nova da regua (~1,1 ms, vem da interrupcao do DMA): controle PD
 *     e marcas. E o laco de controle: roda no ritmo do sensor, sem atraso.
 *   - 1 ms (TIM6): botao, buzina e LEDs.
 *   - 10 ms (TIM6 / 10): bateria, rampa da succao, vigia do ADC, transicoes.
 */

#include "robo.h"
#include "main.h"
#include "config_robo.h"
#include "sensor_linha.h"
#include "marcador_lateral.h"
#include "controlador_pid.h"
#include "controle_motor.h"
#include "controle_succao.h"
#include "interface_usuario.h"
#include "bateria.h"
#include "comunicacao_bluetooth.h"
#include <stdio.h>
#include <stdlib.h>

static volatile uint8_t  g_flag_1ms = 0;
static volatile uint8_t  g_cont_10ms = 0;
static volatile uint8_t  g_flag_10ms = 0;

static estado_robo_t g_estado = ROBO_AGUARDA_CALIB;
static uint32_t      g_t_estado = 0;           /* HAL_GetTick() da entrada no estado */
static int           g_velocidade = VELOCIDADE_BASE;
static uint8_t       g_succao_pct = 0;
static bool          g_calibrado = false;

/* Corrida */
static uint32_t g_t_largada = 0;
static uint32_t g_t_ultima_linha = 0;
static uint16_t g_marcas_dir = 0, g_marcas_esq = 0, g_cruzamentos = 0, g_ignoradas = 0;
static bool     g_fim_por_botao = false;
static uint32_t g_tempo_volta_ms = 0;
static float    g_bat_min_corrida = 99.0f;
static uint32_t g_leituras_corrida = 0;
static bool     g_alarme_avisado = false;
static uint32_t g_t_ultima_leitura = 0;

static qtr_leitura_t g_leitura;
static uint16_t      g_filtrado[NUM_SENSORES_LINHA];
static qtr_linha_t   g_linha;

/* Visiveis no Live Expressions */
volatile uint8_t  robo_estado_live = 0;
volatile int16_t  robo_erro_live = 0;
volatile int16_t  robo_u_live = 0;

extern volatile uint16_t sl_dir_live, sl_esq_live;   /* marcador_lateral.c */

static void entrar(estado_robo_t e);

void robo_tick_1ms_isr(void)
{
    g_flag_1ms = 1;
    if (++g_cont_10ms >= 10U) {
        g_cont_10ms = 0;
        g_flag_10ms = 1;
    }
}

estado_robo_t robo_estado(void) { return g_estado; }

void robo_desligar_tudo(void)
{
    motor_parar(true);
    succao_desligar();
}

void robo_inicializar(ADC_HandleTypeDef *hadc_linha, TIM_HandleTypeDef *htim_motor,
                      TIM_HandleTypeDef *htim_succao, UART_HandleTypeDef *huart_bt)
{
    iu_inicializar();
    motor_inicializar(htim_motor);
    succao_inicializar(htim_succao);
    bateria_inicializar();
    pid_inicializar();
    marcador_inicializar();
    bt_inicializar(huart_bt);          /* so para o syscalls.c compilar e o RX nao travar */
    qtr_inicializar(hadc_linha);
    robo_desligar_tudo();

    printf("\r\n=== Pernilongo 5 - seguidor de linha ===\r\n");
    printf("PD: KP=%.3f KD=%.3f | velocidade %d | succao %u%%\r\n",
           (double)PID_KP, (double)PID_KD, g_velocidade, succao_pct_para_velocidade(g_velocidade));
    printf("Ponha o robo SOBRE A LINHA e aperte o botao para calibrar.\r\n");
    entrar(ROBO_AGUARDA_CALIB);
}

/* ============================== Transicoes =============================== */
static void imprimir_resumo(void)
{
    printf("\r\n--- Fim da corrida (%s) ---\r\n", g_fim_por_botao ? "botao" : "chegada");
    printf("Tempo: %lu.%03lu s | leituras: %lu (%lu us cada)\r\n",
           (unsigned long)(g_tempo_volta_ms / 1000U), (unsigned long)(g_tempo_volta_ms % 1000U),
           (unsigned long)g_leituras_corrida,
           (unsigned long)(g_leituras_corrida ? (g_tempo_volta_ms * 1000U) / g_leituras_corrida : 0U));
    printf("Marcas: direita %u | esquerda %u | cruzamentos %u | ignoradas %u\r\n",
           g_marcas_dir, g_marcas_esq, g_cruzamentos, g_ignoradas);
    printf("Bateria: minima na corrida %.2f V | agora %.2f V\r\n",
           (double)g_bat_min_corrida, (double)bateria_tensao_lenta());
    printf("Aperte: curto = pronto para outra volta | longo = calibrar de novo\r\n");
}

static void entrar(estado_robo_t e)
{
    g_estado = e;
    g_t_estado = HAL_GetTick();
    robo_estado_live = (uint8_t)e;

    switch (e) {
    case ROBO_AGUARDA_CALIB:
        robo_desligar_tudo();
        iu_led_modo(LED_PISCA_LENTO, LED_APAGADO);
        break;

    case ROBO_CALIBRANDO:
        robo_desligar_tudo();
        iu_led_modo(LED_APAGADO, LED_PISCA_RAPIDO);
        iu_bip_bloqueante(100);
        /* 1. Mede o tempo de resposta (robo parado sobre a linha) */
        if (!qtr_medir_resposta()) {
            printf("Aviso: nenhum sensor deu degrau (regua sobre o preto?). Espera padrao.\r\n");
        }
        /* 2. Comeca a varredura min/max */
        qtr_calibracao_resetar();
        printf("Calibrando por %u s: passe a regua sobre a linha e o fundo.\r\n",
               (unsigned)(CALIBRACAO_DURACAO_MS / 1000U));
        break;

    case ROBO_CALIB_FALHOU:
        iu_led_modo(LED_APAGADO, LED_ACESO);
        iu_bip_triplo(80, 80);
        break;

    case ROBO_PRONTO:
        robo_desligar_tudo();
        iu_led_modo(LED_ACESO, LED_APAGADO);
        printf("PRONTO. Velocidade %d, succao %u%%. Curto = largar | longo = +%d de velocidade\r\n",
               g_velocidade, succao_pct_para_velocidade(g_velocidade), VELOCIDADE_PASSO);
        break;

    case ROBO_LARGADA:
        iu_led_modo(LED_PISCA_RAPIDO, LED_PISCA_RAPIDO);
        iu_bip(150);
        g_succao_pct = succao_pct_para_velocidade(g_velocidade);
        break;

    case ROBO_CORRIDA:
        g_t_largada = HAL_GetTick();
        g_t_ultima_linha = g_t_largada;
        g_marcas_dir = g_marcas_esq = g_cruzamentos = g_ignoradas = 0;
        g_fim_por_botao = false;
        g_bat_min_corrida = 99.0f;
        g_leituras_corrida = 0;
        marcador_inicializar();
        pid_resetar(g_linha.erro);
        iu_led_modo(LED_APAGADO, LED_APAGADO);
        motor_definir_potencia(0, 0);
        motor_parar(false);                       /* DIS = 0 */
        break;

    case ROBO_FRENANDO:
        g_tempo_volta_ms = HAL_GetTick() - g_t_largada;
        /* EVA-RT: contra-corrente com -SPEED por STOP_TIME_MS */
        motor_definir_potencia(-g_velocidade, -g_velocidade);
        iu_led_modo(LED_ACESO, LED_ACESO);
        break;

    case ROBO_FIM:
        robo_desligar_tudo();
        iu_led_modo(LED_PISCA_LENTO, LED_PISCA_LENTO);
        if (g_fim_por_botao) iu_bip_triplo(60, 60); else iu_bip(1000);
        imprimir_resumo();
        break;

    case ROBO_FORA_PISTA:
        robo_desligar_tudo();
        g_tempo_volta_ms = HAL_GetTick() - g_t_largada;
        iu_led_modo(LED_APAGADO, LED_PISCA_RAPIDO);
        iu_bip(400);
        printf("PERDEU A LINHA apos %lu ms (lado %d). Botao = pronto.\r\n",
               (unsigned long)g_tempo_volta_ms, g_linha.ultimo_lado);
        break;
    }
}

/* ========================= Controle (cada leitura) ======================= */
static void controle_corrida(uint32_t agora)
{
    uint32_t t = agora - g_t_largada;

    /* Rampa de partida (EVA-RT LITE: SPEED_TIME_MS = 50) */
    int base = g_velocidade;
    if (t < RAMPA_PARTIDA_MS) base = (int)((g_velocidade * (int32_t)t) / RAMPA_PARTIDA_MS);

    /* No cruzamento o erro vale 0 (segue reto): EVA-RT CORRECT_CROSS_ERROR */
    int16_t erro = g_linha.cruzamento ? 0 : g_linha.erro;
    int16_t u = pid_calcular(erro);
    int esq, dir;

    if (g_linha.valida || g_linha.cruzamento) {
        esq = base + u;
        dir = base - u;
        g_t_ultima_linha = agora;
    } else {
        /* Fora da linha (EVA-RT OUT METHOD): gira forte para o lado em que ela sumiu */
        int8_t lado = g_linha.ultimo_lado;
        if (lado == 0) lado = (g_linha.erro < 0) ? -1 : +1;
        if (lado < 0) { esq = FORA_POTENCIA_INTERNA; dir = FORA_POTENCIA_EXTERNA; }
        else          { esq = FORA_POTENCIA_EXTERNA; dir = FORA_POTENCIA_INTERNA; }
        if ((agora - g_t_ultima_linha) >= TEMPO_MAX_FORA_LINHA_MS) {
            entrar(ROBO_FORA_PISTA);
            return;
        }
    }
    motor_definir_potencia(esq, dir);
    robo_erro_live = erro;
    robo_u_live = u;
}

static void marcas_corrida(uint32_t agora)
{
    tipo_marca_t m = marcador_atualizar(g_leitura.lateral_dir, g_leitura.lateral_esq,
                                        g_linha.erro, g_linha.cruzamento, agora);
    switch (m) {
    case MARCA_ESQUERDA:
        g_marcas_esq++;
        iu_bip(BIP_MARCA_MS);
        iu_led_flash(true, false, LED_MARCA_MS);
        break;
    case MARCA_DIREITA:
        /* A 2a marca so vale depois de TEMPO_MIN_ATE_CHEGADA_MS (EVA-RT: 1 s) */
        if (g_marcas_dir == 0U || (agora - g_t_largada) >= TEMPO_MIN_ATE_CHEGADA_MS) {
            g_marcas_dir++;
        }
        iu_bip(BIP_MARCA_MS);
        iu_led_flash(false, true, LED_MARCA_MS);
        if (g_marcas_dir >= MARCADORES_DIREITA_FIM) entrar(ROBO_FRENANDO);
        break;
    case MARCA_INTERSECAO:
        g_cruzamentos++;
        iu_bip_duplo(BIP_CRUZ_MS, BIP_CRUZ_PAUSA_MS);
        iu_led_flash(true, true, LED_MARCA_MS);
        break;
    case MARCA_IGNORADA:
        g_ignoradas++;
        break;
    default:
        break;
    }
}

static void processar_leitura(void)
{
    uint32_t agora = HAL_GetTick();
    g_t_ultima_leitura = agora;
    sl_dir_live = g_leitura.lateral_dir;           /* laterais no Live Expressions */
    sl_esq_live = g_leitura.lateral_esq;           /* em qualquer estado           */
    qtr_filtrar(g_leitura.bruto, g_filtrado);

    if (g_estado == ROBO_CALIBRANDO) {
        qtr_calibracao_amostrar(g_filtrado);
        return;
    }
    if (!g_calibrado) return;

    qtr_processar(g_filtrado, &g_linha);

    if (g_estado == ROBO_CORRIDA) {
        g_leituras_corrida++;
        controle_corrida(agora);
        if (g_estado == ROBO_CORRIDA) marcas_corrida(agora);
    }
}

/* ======================= Laco lento (10 ms) ============================== */
static void laco_lento(void)
{
    uint32_t agora = HAL_GetTick();
    uint32_t no_estado = agora - g_t_estado;
    bool em_corrida = (g_estado == ROBO_CORRIDA) || (g_estado == ROBO_LARGADA) || (g_estado == ROBO_FRENANDO);

    /* Bateria: sempre monitorada */
    bateria_tick_10ms(em_corrida);
    if (bateria_presente()) {
        float v = bateria_tensao();
        if (g_estado == ROBO_CORRIDA && v < g_bat_min_corrida) g_bat_min_corrida = v;
    }
    if (bateria_baixa()) {
        iu_alarme_bateria(true, (g_estado != ROBO_CORRIDA) || BAT_ALARME_NA_CORRIDA);
        if (!g_alarme_avisado && !em_corrida) {
            g_alarme_avisado = true;
            printf("BATERIA BAIXA: %.2f V. Carregue a bateria!\r\n", (double)bateria_tensao_lenta());
            bateria_imprimir_diagnostico();
        }
    } else {
        iu_alarme_bateria(false, false);           /* tensao voltou: histerese liberou */
        if (g_alarme_avisado) {
            g_alarme_avisado = false;
            printf("Bateria OK de novo: %.2f V\r\n", (double)bateria_tensao_lenta());
        }
    }
#if BAT_DIAGNOSTICO_PRINTF
    {
        static uint32_t t_diag = 0;
        if (!em_corrida && (agora - t_diag) >= 2000U) {
            t_diag = agora;
            bateria_imprimir_diagnostico();
        }
    }
#endif

    succao_atualizar();
    qtr_vigiar_adc();

    evento_botao_t b = iu_botao_evento();

    switch (g_estado) {
    case ROBO_AGUARDA_CALIB:
    case ROBO_CALIB_FALHOU:
        if (b == BOTAO_CURTO) entrar(ROBO_CALIBRANDO);
        break;

    case ROBO_CALIBRANDO:
        if (no_estado >= CALIBRACAO_DURACAO_MS) {
            uint8_t ruins = qtr_calibracao_finalizar();
            qtr_calibracao_imprimir();
            qtr_reiniciar_estado();
            if (ruins == 0U) {
                g_calibrado = true;
                printf("Calibracao OK.\r\n");
                iu_bip(300);                       /* som de sucesso */
                entrar(ROBO_PRONTO);               /* LED verde aceso */
            } else {
                printf("Calibracao FALHOU: %u sensores com contraste < %u. Aperte para refazer.\r\n",
                       ruins, (unsigned)QTR_CONTRASTE_MIN);
                entrar(ROBO_CALIB_FALHOU);
            }
        }
        break;

    case ROBO_PRONTO:
        if (b == BOTAO_CURTO) {
            entrar(ROBO_LARGADA);
        } else if (b == BOTAO_LONGO) {
            g_velocidade += VELOCIDADE_PASSO;
            if (g_velocidade > VELOCIDADE_MAXIMA) {
                g_velocidade = VELOCIDADE_BASE;
                iu_bip(600);                       /* voltou ao inicio */
            } else {
                iu_bip_duplo(60, 60);
            }
            printf("Velocidade %d, succao %u%%\r\n", g_velocidade, succao_pct_para_velocidade(g_velocidade));
        }
        break;

    case ROBO_LARGADA:
        if (b != BOTAO_NADA) {                     /* cancela */
            succao_desligar();
            entrar(ROBO_PRONTO);
            break;
        }
        /* 1 s para tirar a mao, depois a succao sobe em rampa e forma o vacuo */
        if (no_estado >= TEMPO_MAOS_FORA_MS) succao_definir_potencia(g_succao_pct);
        if (no_estado >= TEMPO_MAOS_FORA_MS + TEMPO_SUCCAO_ANTES_MS) entrar(ROBO_CORRIDA);
        break;

    case ROBO_CORRIDA:
        if (b != BOTAO_NADA) {                     /* parada de emergencia (EVA-RT) */
            g_fim_por_botao = true;
            entrar(ROBO_FRENANDO);
        } else if ((agora - g_t_ultima_leitura) > 20U) {
            /* sem leitura nova ha 20 ms: o motor ficaria com o ultimo comando */
            printf("ERRO: a regua parou de mandar leituras.\r\n");
            entrar(ROBO_FORA_PISTA);
        }
        break;

    case ROBO_FRENANDO:
        if (no_estado >= TEMPO_FRENAGEM_MS) motor_parar(true);
        if (no_estado >= TEMPO_SUCCAO_APOS_FIM_MS) entrar(ROBO_FIM);
        break;

    case ROBO_FIM:
        if (b == BOTAO_CURTO) entrar(ROBO_PRONTO);
        else if (b == BOTAO_LONGO) entrar(ROBO_CALIBRANDO);
        break;

    case ROBO_FORA_PISTA:
        if (b == BOTAO_CURTO) entrar(ROBO_PRONTO);
        else if (b == BOTAO_LONGO) entrar(ROBO_CALIBRANDO);
        break;
    }
}

void robo_executar(void)
{
    if (g_flag_1ms) {
        g_flag_1ms = 0;
        iu_atualizar(HAL_GetTick());
    }
    if (qtr_nova_leitura(&g_leitura)) {
        processar_leitura();
    }
    if (g_flag_10ms) {
        g_flag_10ms = 0;
        laco_lento();
    }
}
