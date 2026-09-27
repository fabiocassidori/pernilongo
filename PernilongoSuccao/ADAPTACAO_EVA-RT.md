# Adaptação do firmware PernilongoSuccao para o hardware EVA-RT / Pernilongo

Base: código PernilongoSuccao (validado em pista). Hardware: esquemático/PCB
`Pernilongo_1` (STM32F405RGT6 + 2× IFX9201SG + régua de 12 QRE1113).

A lógica foi mantida. As mudanças são as que o hardware exige.

---

## ⚠️ Antes de compilar: regenerar pelo CubeMX (uma vez)

O projeto original nunca usou serial, então **o driver HAL de UART não está na pasta
`Drivers/`**. Para o Bluetooth compilar:

1. Abra `PernilongoSuccao.ioc` no STM32CubeIDE.
2. Clique em **Project → Generate Code**.
3. O CubeMX copia `stm32f4xx_hal_uart.c/.h` para `Drivers/` e regrava `main.c`,
   `stm32f4xx_hal_msp.c` e `stm32f4xx_it.c` a partir do `.ioc`.

Isso é seguro: toda a lógica está dentro de blocos `USER CODE`, e as funções `MX_*_Init`
que entreguei já são idênticas ao que o `.ioc` gera. As pastas `Debug/` e `Release/` foram removidas
de propósito — o STM32CubeIDE recria os makefiles no primeiro build (as antigas listavam
os módulos removidos e quebrariam o build).

---

## O que foi validado

Sem acesso a um compilador ARM nesta sessão, validei de três formas:

1. **Checagem sintática e de tipos** de todos os `.c` com GCC contra os headers HAL/CMSIS
   reais do projeto, com `-Wall -Wextra`: **0 erros e 0 avisos** no código do projeto.
   (O `sysmem.c`, não alterado, só compila com a newlib do toolchain ARM — igual ao original.)
2. **Execução real** de `sensor_linha.c`, `controlador_pid.c`, `controle_motor.c` e
   `marcador_lateral.c` no computador, com o HAL simulado: **22 verificações, 0 falhas**.
   Confirma o sentido da curva (linha à direita → roda esquerda acelera), a posição
   0 no centro da régua, a linha perdida, toda a lógica do IFX9201SG e os marcadores.
3. **Cruzamento com a PCB**: os 27 pinos do firmware batem com as nets reais do STM32 na
   PCB, e `.ioc` / `main.c` / `main.h` são consistentes entre si.

Isso não substitui o teste na bancada (veja o fim do documento), mas elimina erros de
pinagem, de tipo e de sinal.

---

## Pinagem final

| Função | Pino | Periférico | Observação |
|---|---|---|---|
| Lateral direito (SLDIR) | PC0 | ADC1_IN10 | índice 0 do buffer |
| QTR12 … QTR04 | PC3, PA0–PA7 | ADC1_IN13, IN0–IN7 | índices 1–9 |
| QTR03, QTR02 | PC4, PC5 | ADC1_IN14, IN15 | índices 10–11 |
| QTR01 | PB0 | ADC1_IN8 | índice 12 |
| Lateral esquerdo (SLESQ) | PB1 | ADC1_IN9 | índice 13 |
| PWM motor direito (U3) | PC6 | TIM8_CH1 | 20 kHz |
| PWM motor esquerdo (U2) | PC7 | TIM8_CH2 | 20 kHz |
| DIR direito / esquerdo | PB12 / PB14 | GPIO | |
| DIS (os dois drivers) | PB13 | GPIO | nasce em nível alto |
| Sucção (Q3) | PA15 | TIM2_CH1 | 20 kHz |
| Emissores da régua (Q5) | PA8 | GPIO | **precisa ficar alto** |
| Bluetooth TX / RX | PA9 / PA10 | USART1 | 9600 baud |
| Botão | PB4 | entrada | pull-up externo R18 |
| Buzina | PB8 | GPIO | |
| LED verde / vermelho | PC8 / PB9 | GPIO | ativos em alto |
| SWD + SWO | PA13, PA14, PB3 | debug | printf por SWO |

---

## Mudanças, e por quê

### Mantido sem alteração
- **Clock**: HSI → PLL → 168 MHz (`SystemClock_Config` idêntico byte a byte).
- **ADC**: 10 bits, PCLK2/4, 15 ciclos de amostragem, contínuo com DMA circular.
- **Loop de controle**: TIM6 a 1 ms, loop lento a 10 ms.
- **Calibração, normalização e média ponderada** dos sensores.
- **PID de direção** (`controlador_pid.c` intacto) e a regra `base − correção / base + correção`.
- **Marcadores laterais**, detecção de linha perdida, buzina e botão.
- **Fim de corrida**: 2ª marcação direita.

### Sensores: 6 → 12
- `NUM_SENSORES_LINHA = 12`. Buffer de 8 posições fixas → 14 (`NUM_CANAIS_ADC`).
- Números mágicos `2500`/`5000` → `SENSOR_POSICAO_CENTRO`/`MAX` (5500/11000), derivados do número de sensores.
- Índices fixos `[0]`/`[7]` dos laterais → `IDX_LATERAL_DIREITO`/`ESQUERDO` (0 e 13).
- **Ordem no ADC do mais à direita para o mais à esquerda** (QTR12 → QTR01), para manter a convenção original (posição mínima = extrema direita) e, com ela, o sinal do PID.
- **QTR13 (PC2) e QTR14 (PC1)** existem no esquemático mas **não chegam ao J4**: não são lidos.

### Emissores da régua
Na placa nova os LEDs IR da régua só acendem via **Q5, comandado por PA8**. O código antigo
não tocava nesse pino — só trocar a pinagem deixaria **a régua apagada**. Agora
`sensor_linha_inicializar()` liga os emissores (sempre acesos, como no robô validado).
A pulsagem para rejeitar luz ambiente fica como evolução futura (PA8 também é TIM1_CH1).

### Motores: TB6612 → IFX9201SG
Mesma interface (`motor_definir_potencia`, `motor_parar`, `motor_frear`), implementação nova,
conforme a tabela-verdade do datasheet (Rev. 1.1, Tabela 4-1):

| | TB6612 | IFX9201SG |
|---|---|---|
| Direção | IN1 + IN2 | **um pino DIR** |
| Habilitação | STBY, ativo em alto | **DIS, ativo em alto = desliga** (lógica inversa) |
| Freio ativo | sim (IN1=IN2=1) | **não existe** no modo PWM/DIR |
| PWM máximo | 100 kHz | **20 kHz** |

Consequências:
- **PWM de 50 kHz → 20 kHz** (`PERIODO_PWM_MOTOR` 3359 → 8399). É a única configuração de
  temporização alterada: 50 kHz é 2,5× o limite do IFX9201SG. A resolução do PWM subiu de
  3360 para 8400 passos.
- **Frenagem por contra-corrente** (motores em ré por `TEMPO_FRENAGEM_MS`), já que PWM=0
  deixa o motor em roda livre.
- **DIS nasce em nível alto** (e o IFX9201SG tem pull-up interno em DIS): as pontes não
  disparam no boot.

### Removido (não existe no circuito)
- **Encoders** (`leitor_encoder`), **controle de velocidade em malha fechada**
  (`controle_velocidade`), **mapeamento de pista** (`mapa_pista`) e **monitor de bateria**
  (`monitor_bateria`), com seus periféricos (TIM3, TIM4, ADC2).
- Consequência importante: **a velocidade passa a ser em malha aberta**
  (`POTENCIA_BASE_CORRIDA`, 0–255). Sem encoder não há como medir velocidade nem distância,
  então também saem o mapeamento e a aceleração em retas. `PID_KP_RETA`/`PID_KD_RETA` ficam
  no `config_robo.h` para uso futuro.
- Pinos do segundo TB6612 do ventilador (`*VENT`) e SPI3 — não existem na placa.

### Adicionado
- **Sucção** (`controle_succao`): o código antigo configurava o ventilador mas **nunca o
  ligava**. Agora liga no botão de largada, sobe em rampa (evita pico de corrente) durante
  a espera de 1 s já existente, e desliga ao parar, sair da pista ou em erro.
  `SUCCAO_HABILITADA 0` desliga tudo para testes na mão.
- **Bluetooth** (`comunicacao_bluetooth`): envio com timeout (nunca trava o robô) e
  recepção por interrupção em buffer circular, pronta para comandos. O `printf` é
  espelhado no Bluetooth (`BT_ESPELHAR_PRINTF`).
- **LEDs**: vermelho aceso na calibração e em erro/fora da pista; verde aceso pronto para largar e na corrida.

### Correções encontradas no caminho
- **`.ioc` fora de sincronia com o `main.c` validado**: o `.ioc` antigo tinha
  `TIM6.Period=49` (loop de **5 ms**), enquanto o `main.c` usa 9 (**1 ms**). Regenerar pelo
  `.ioc` antigo teria mudado o loop de controle sem aviso. Corrigido no `.ioc` novo.
- **`printf` travava sem depurador**: `ITM_SendChar` esperava o ITM mesmo desabilitado.
  Agora sai sem enviar quando não há SWV ativo (mesma checagem do CMSIS) — necessário para
  usar o Bluetooth sem o ST-LINK conectado.
- **Final de corrida**: no código antigo, o estado `DESACELERANDO_FINAL` nunca era
  processado pelo loop rápido, então o robô parava e não chegava a pedir o reinício. Agora
  a frenagem final termina e o robô volta a aguardar o botão.

---

## ⚠️ Três pontos de HARDWARE (o firmware não resolve)

### 1. O Bluetooth não tem alimentação adequada — **precisa correção na PCB**
O pino 4 do J5 (onde entraria o VCC do módulo) está ligado a um **divisor resistivo**:
R26 (10 kΩ) do VBAT e R25 (15 kΩ) para o GND. Isso gera ~0,6×VBAT (4,4–5,0 V) **sem
carga**, mas com impedância de saída de ~6 kΩ. Um HC-05 consome 8–40 mA; o divisor
entregaria menos de 1 mA. **O módulo não liga.**

Parece que o divisor de medição de bateria (que no projeto antigo ia para o PB1) acabou
ligado ao conector do Bluetooth. Sugestão: ligue J5.4 direto ao **3,3 V** e use um
módulo 3,3 V (HM-10 ou HC-05 sem placa adaptadora). Os níveis lógicos já são 3,3 V.

Ligação do cabo: J5.1 (PA9, TX do MCU) → **RXD** do módulo; J5.2 (PA10, RX do MCU) ← **TXD** do módulo.

### 2. Carga contínua no regulador de 3,3 V
Com os emissores sempre acesos, os 12 LEDs IR (100 Ω cada) consomem ~250 mA contínuos no
LT1764. Com VBAT de 7,4 V, o regulador dissipa ~1 W. Está dentro do componente, mas
confirme que ele tem cobre suficiente para dissipar. A pulsagem dos emissores reduziria
isso bastante.

### 3. QTR13 e QTR14 sem ligação
Os rótulos existem no MCU (PC2, PC1), mas não chegam ao J4. Se quiser 14 sensores no
futuro, falta a ligação física e dois pinos no FFC.

---

## Teste na bancada (antes da pista)

Faça com o robô **suspenso**, rodas sem tocar o chão, e `SUCCAO_HABILITADA 0`.

1. **Sensores**: descomente `sensor_linha_depuracao()` no laço principal. Passe a linha sob
   a régua da **direita para a esquerda**: os valores devem mudar da coluna da esquerda
   para a da direita na tela (índice 1 = QTR12 = extrema direita).
2. **Lado dos motores**: com a linha à direita da régua, a roda **esquerda** deve girar
   mais rápido. Se a roda errada responder, mude `MOTOR_TROCAR_LADOS` para 1.
3. **Sentido de rotação**: as duas rodas devem girar para a **frente**. Se uma girar para
   trás, inverta `MOTOR_ESQ_DIR_FRENTE` ou `MOTOR_DIR_DIR_FRENTE` — não é preciso mexer
   nos fios.
4. **Sucção**: volte `SUCCAO_HABILITADA 1` e confira se o ventilador liga no botão de
   largada e desliga ao parar.

Os valores padrão desses três parâmetros foram deduzidos da posição dos componentes na
PCB, mas só a bancada confirma.

## Ajuste em pista
- **PID**: `PID_KP`/`PID_KD` foram ajustados para a régua antiga. Com 12 sensores e a
  posição normalizada em ±255, a sensibilidade mudou — **reajuste**.
- **Velocidade**: `POTENCIA_BASE_CORRIDA` começa em 80 (valor citado no próprio
  comentário original do PID). Suba aos poucos.
- **Sucção**: `POTENCIA_SUCCAO_PCT` começa em 80 %.
