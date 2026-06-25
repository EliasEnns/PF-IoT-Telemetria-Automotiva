/**
 * @file    telemetria_esp32.ino
 * @brief   Telemetria automotiva multitarefa para ESP32 (inspirada no TCC Baja SAE).
 *
 * @details
 * Aquisição de sensores e comunicação bidirecional com um dashboard no notebook
 * via WebSocket assíncrono. O projeto é estruturado sobre o FreeRTOS (nativo no
 * ESP32) em cinco tarefas com responsabilidades isoladas, comunicando-se por
 * dois mutexes e uma fila. Nenhuma operação bloqueante existe no caminho
 * principal; o que exige espera (DHT11, multiplexação do display, transmissão)
 * é confinado em sua própria tarefa ou agendado por período.
 *
 * ### Mapeamento funcional
 * - Potenciômetro (GPIO34) ...... posição do acelerador -> velocidade (display)
 * - DHT11        (GPIO33) ........ umidade (e temperatura ambiente de apoio)
 * - LDR          (GPIO39) ........ luminosidade
 * - Botão 3      (GPIO2)  ........ EMERGÊNCIA (entrada digital)
 * - Botão 4      (GPIO15) ........ FREIO      (entrada digital)
 * - LED 1        (GPIO4)  ........ luz do PIT       (saída comandada pelo PC)
 * - LED RGB azul (GPIO27) ........ BANDEIRA (saída comandada pelo PC)
 * - Display 7seg (2 díg.) ........ velocidade 00..99 km/h
 *
 * A temperatura do "motor" é SIMULADA a partir da carga do acelerador (ver
 * @ref SIMULAR_TEMP_MOTOR), com inércia térmica, para reproduzir o
 * comportamento de um motor real em telemetria de bancada.
 *
 * ### Modelo de concorrência (resumo)
 * | Tarefa             | Core | Prio | Período  | Recurso protegido        |
 * |--------------------|------|------|----------|--------------------------|
 * | taskLeitura        |  1   |  3   |  5 ms    | escreve g_sensores       |
 * | taskDHT            |  1   |  1   |  2000 ms | escreve g_sensores       |
 * | taskProcessamento  |  1   |  2   |  20 ms   | lê g_sensores / escreve g_telemetria / lê fila |
 * | taskDisplay        |  1   |  4   |  ~4 ms   | lê g_telemetria          |
 * | taskComunicacao    |  0   |  2   |  ~100 ms | lê g_telemetria          |
 *
 * O acesso a @ref g_sensores é serializado por @ref mtxSensores e o acesso a
 * @ref g_telemetria por @ref mtxTelemetria. Comandos de LED produzidos no
 * contexto assíncrono do WebSocket (core 0) são entregues à @ref taskProcessamento
 * pela fila @ref xFilaComandos, evitando que o callback de rede toque
 * diretamente o estado de saída.
 *
 * @warning AVISOS DE HARDWARE desta devboard:
 *  -# O display usa GPIO1 (TX0) e GPIO3 (RX0): a Serial NÃO é utilizada.
 *  -# GPIO2 e GPIO15 (botões) são pinos de strapping; o pull-up de fábrica
 *     mantém o boot normal mesmo com os botões conectados.
 *  -# O relé (GPIO13) causa brownout e nunca é acionado.
 *  -# Sem capacitores de desacoplamento: ADC ruidoso, filtragem obrigatória.
 *
 * @note Bibliotecas (Arduino IDE): "DFRobot_DHT11", "ESP Async WebServer",
 *       "AsyncTCP". Placa: "ESP32 Dev Module". ArduinoJson não é necessário.
 *
 * @author  Elias Enns
 * @date    2025
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DFRobot_DHT11.h>

/* ===========================================================================
 *  1) PINOUT (definições do autor do shield, mantidas)
 * =========================================================================== */
/** @name Pinos do shield (definições originais do autor) */
///@{
#define LED1 4
#define LED2 0
#define LED3 2
#define LED4 15
#define BOTAO1 4
#define BOTAO2 0
#define BOTAO3 2
#define BOTAO4 15
#define RGB_RED 25
#define RGB_GREEN 26
#define RGB_BLUE 27
#define RELE 13
#define A 18
#define B 5
#define C 21
#define D 03
#define E 01
#define F 23
#define G 22
#define DP 19
#define DISPLAY1 16
#define DISPLAY2 17
#define DHT11_PIN 33   ///< Sensor DHT11 (rotulado VP na placa)
#define LDR 39         ///< Divisor com LDR (rotulado VN na placa)
#define POTENCIOMETRO 34
#define ULTRASSONIC_TRIG 32
#define ULTRASSONIC_ECHO 35
///@}

/**
 * @brief Mapa de segmentos por dígito decimal.
 * @details Cada linha corresponde a um dígito 0..9 na ordem de segmentos
 *          {A,B,C,D,E,F,G,DP}; o valor 1 indica segmento aceso. Tabela
 *          fornecida pelo autor do shield.
 */
byte segNum[][8] =
{
  {1, 1, 1, 1, 1, 1, 0, 0}, // 0
  {0, 1, 1, 0, 0, 0, 0, 0}, // 1
  {1, 1, 0, 1, 1, 0, 1, 0}, // 2
  {1, 1, 1, 1, 0, 0, 1, 0}, // 3
  {0, 1, 1, 0, 0, 1, 1, 0}, // 4
  {1, 0, 1, 1, 0, 1, 1, 0}, // 5
  {1, 0, 1, 1, 1, 1, 1, 0}, // 6
  {1, 1, 1, 0, 0, 0, 0, 0}, // 7
  {1, 1, 1, 1, 1, 1, 1, 0}, // 8
  {1, 1, 1, 1, 0, 1, 1, 0}, // 9
};

/** @brief Vetor com todos os pinos do display (7 segmentos + DP + 2 commons). */
byte display_pins[] = {A, B, C, D, E, F, G, DP, DISPLAY1, DISPLAY2};

/** @name Papéis efetivos dos pinos neste projeto (apelidos legíveis)
 *  @note O Botão 2 (GPIO0) original tinha mau contato, então as entradas
 *        passaram para os Botões 3/4 (GPIO2/GPIO15) e as saídas para o LED 1
 *        (PIT) e uma cor do LED RGB (bandeira). Isso também tira a entrada do
 *        GPIO0, que é sensível no boot. */
///@{
#define PIN_ACELERADOR   POTENCIOMETRO  ///< Acelerador (potenciômetro, ADC1)
#define PIN_LUM          LDR            ///< Luminosidade (LDR, ADC1)
#define PIN_EMERGENCIA   BOTAO3         ///< Botão de emergência (GPIO2)
#define PIN_FREIO        BOTAO4         ///< Botão de freio (GPIO15)
#define PIN_LED_PIT      LED1           ///< Saída: luz do PIT (GPIO4)
#define PIN_LED_BANDEIRA RGB_GREEN       ///< Saída: bandeira = LED RGB azul (GPIO27)
///@}

/** @brief Pinos dos 7 segmentos + ponto, na ordem da tabela @ref segNum. */
static const uint8_t SEG_PINS[8] = {A, B, C, D, E, F, G, DP};

/* ===========================================================================
 *  2) POLARIDADE DO DISPLAY
 * =========================================================================== */
/**
 * @name Polaridade do display de 7 segmentos
 * @brief Níveis lógicos de acionamento de segmentos e dígitos desta placa.
 */
///@{
#define SEG_ON   HIGH   ///< Nível que ACENDE um segmento
#define SEG_OFF  LOW    ///< Nível que APAGA um segmento
#define DIG_ON   HIGH   ///< Nível que ATIVA o common de um dígito
#define DIG_OFF  LOW    ///< Nível que DESATIVA o common de um dígito
///@}

/* ===========================================================================
 *  3) PARÂMETROS DE REDE / WIFI
 * =========================================================================== */
/**
 * @brief Seleciona o modo de rede do ESP32.
 * @details Em modo AP, o ESP cria a
 *          própria rede e o IP é sempre 192.168.4.1. Em modo STA (false) o
 *          ESP entra na rede existente e o IP é atribuído pelo roteador.
 */
#define USAR_AP   true

const char* AP_SSID  = "BAJA_TELEM"; ///< SSID do Access Point criado pelo ESP
const char* AP_PASS  = "baja12345";  ///< Senha do AP (mínimo 8 caracteres)
const char* STA_SSID = "";  ///< SSID da rede
const char* STA_PASS = "";  ///< Senha da rede

/* ===========================================================================
 *  4) PARÂMETROS DE CONTROLE / FILTROS / TEMPOS
 * =========================================================================== */
/** @name Parâmetros de filtragem e de escala */
///@{
#define JANELA_MEDIA             16   ///< Amostras da média móvel (analógicas)
#define AMOSTRAS_FILTRO_DIGITAL  5    ///< Confirmações do anti-bounce digital
#define ADC_MAX                  4095 ///< Fundo de escala do ADC (12 bits)
#define VEL_MAX_KMH              99   ///< Velocidade máxima exibível (2 dígitos)
#define FATOR_FREIO_PCT          50   ///< Freio reduz a velocidade exibida (%)
///@}

/**
 * @name Simulação da temperatura do motor
 * @brief Gera uma temperatura plausível de motor a partir da carga (acelerador).
 * @details Com @ref SIMULAR_TEMP_MOTOR ativo, a temperatura parte de
 *          @ref TEMP_MOTOR_IDLE em marcha lenta e tende a
 *          @ref TEMP_MOTOR_MAX sob aceleração plena, com inércia térmica
 *          (aproximação exponencial) para subir/descer de forma suave. Isso
 *          substitui a leitura crua do DHT11 (que reflete apenas a temperatura
 *          ambiente) por um sinal coerente com telemetria automotiva. Defina
 *          como 0 para transmitir a temperatura real do DHT11.
 */
///@{
#define SIMULAR_TEMP_MOTOR   0     ///< 1 = simula temperatura de motor; 0 = DHT11 cru
#define TEMP_MOTOR_IDLE      85    ///< Temperatura em marcha lenta (°C)
#define TEMP_MOTOR_MAX       110   ///< Temperatura sob carga máxima (°C)
#define TEMP_INERCIA_PCT     3     ///< Aproximação por ciclo rumo ao alvo (% — menor = mais lento)
///@}

/** @name Períodos das tarefas FreeRTOS (ms) */
///@{
#define PERIODO_LEITURA_MS   5    ///< taskLeitura: aquisição rápida (200 Hz)
#define PERIODO_PROC_MS      20   ///< taskProcessamento: lógica (50 Hz)
#define PERIODO_DHT_MS       2000 ///< taskDHT: leitura lenta do DHT11 (0,5 Hz)
#define PERIODO_DISPLAY_MS   4    ///< taskDisplay: ~4 ms por dígito
#define PERIODO_TX_MS        100  ///< taskComunicacao: telemetria (10 Hz)
///@}

/**
 * @brief Polaridade elétrica dos botões.
 * @details true: botão fecha para GND (ativo em LOW) e usa pull-up interno.
 *          false: botão fecha para VCC (ativo em HIGH) e usa pull-down interno.
 */
#define BOTAO_ATIVO_EM_LOW  true

/* ===========================================================================
 *  5) FILTRO ANALÓGICO — MÉDIA MÓVEL
 * =========================================================================== */
/**
 * @class MediaMovel
 * @brief Filtro de média móvel com janela circular e soma incremental.
 *
 * @details
 * Mantém uma soma corrente sobre uma janela fixa de @ref JANELA_MEDIA amostras.
 * A cada nova leitura subtrai a amostra mais antiga e soma a recém-adquirida,
 * resultando em custo O(1) por iteração (sem varredura do buffer). Indicado
 * para suavizar o ruído do ADC desta placa, que não possui capacitores de
 * desacoplamento.
 *
 * @note Não é reentrante: cada instância é manipulada por uma única tarefa
 *       (a @ref taskLeitura), portanto dispensa proteção por mutex.
 */
class MediaMovel {
public:
  /**
   * @brief Inicializa o filtro e pré-carrega a janela com a leitura atual.
   * @param[in] pino Pino analógico a ser amostrado.
   * @return void
   * @note O pré-carregamento evita a rampa inicial que ocorreria se a janela
   *       começasse zerada.
   */
  void begin(uint8_t pino) {
    _pino = pino;
    _idx  = 0;
    _soma = 0;
    uint16_t v = analogRead(_pino);
    for (uint8_t i = 0; i < JANELA_MEDIA; i++) {
      _buf[i] = v;
      _soma  += v;
    }
  }

  /**
   * @brief Adquire uma nova amostra e atualiza a média.
   * @return Média atual da janela (0 .. @ref ADC_MAX).
   */
  uint16_t read() {
    uint16_t nova = analogRead(_pino);
    _soma -= _buf[_idx];
    _buf[_idx] = nova;
    _soma += nova;
    _idx = (_idx + 1) % JANELA_MEDIA;
    return (uint16_t)(_soma / JANELA_MEDIA);
  }

  /**
   * @brief Retorna a média atual sem adquirir nova amostra.
   * @return Média atual da janela (0 .. @ref ADC_MAX).
   */
  uint16_t valor() const { return (uint16_t)(_soma / JANELA_MEDIA); }

private:
  uint8_t  _pino = 0;                 ///< Pino analógico amostrado
  uint16_t _buf[JANELA_MEDIA] = {0};  ///< Buffer circular de amostras
  uint8_t  _idx = 0;                  ///< Índice de escrita no buffer
  uint32_t _soma = 0;                 ///< Soma corrente das amostras da janela
};

/* ===========================================================================
 *  6) FILTRO DIGITAL — ANTI-BOUNCE DE 3 ESTÁGIOS (portado do TCC)
 * =========================================================================== */
/**
 * @class EntradaDigital
 * @brief Debounce de botão por confirmação de três estágios.
 *
 * @details
 * Algoritmo idêntico ao validado no TCC:
 *  -# Acionamento: confirma estado=1 após @ref AMOSTRAS_FILTRO_DIGITAL leituras
 *     ativas consecutivas (na prática, AMOSTRAS+1 ciclos).
 *  -# Desacionamento: confirma estado=0 após @ref AMOSTRAS_FILTRO_DIGITAL
 *     leituras inativas consecutivas.
 *  -# Indecisão: força estado=0 quando o sinal oscila por tempo excessivo
 *     (AMOSTRAS*4 ciclos), tratando contato instável como repouso seguro.
 *
 * @note Não é reentrante: cada instância pertence à @ref taskLeitura, que é a
 *       única produtora dos estados consumidos posteriormente.
 */
class EntradaDigital {
public:
  /**
   * @brief Configura o pino como entrada com pull interno e zera o filtro.
   * @param[in] pino Pino digital do botão.
   * @return void
   * @note O resistor interno (pull-up ou pull-down) é selecionado conforme
   *       @ref BOTAO_ATIVO_EM_LOW.
   */
  void begin(uint8_t pino) {
    _pino = pino;
    pinMode(_pino, BOTAO_ATIVO_EM_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    _estado = false;
    _f.a = AMOSTRAS_FILTRO_DIGITAL;
    _f.d = AMOSTRAS_FILTRO_DIGITAL;
    _f.i = AMOSTRAS_FILTRO_DIGITAL * 4;
  }

  /**
   * @brief Executa uma iteração do filtro anti-bounce.
   * @return void
   * @note Deve ser chamada periodicamente (a cada ciclo da @ref taskLeitura);
   *       o tempo de confirmação é função do período dessa tarefa.
   */
  void read() {
    const bool raw = readRaw();

    /* Estágio 1 — acionamento */
    if (raw) {
      _f.d = AMOSTRAS_FILTRO_DIGITAL;
      if (_f.a > 0) { _f.a--; }
      else          { _estado = true; }
    }

    /* Estágio 2 — desacionamento */
    if (!raw) {
      _f.a = AMOSTRAS_FILTRO_DIGITAL;
      if (_f.d > 0) { _f.d--; }
      else          { _estado = false; }
    }

    /* Estágio 3 — indecisão (sinal instável por AMOSTRAS*4 ciclos) */
    if (_f.a > 0 && _f.d > 0) {
      if (_f.i > 0) { _f.i--; }
      else          { _estado = false; }
    } else {
      _f.i = AMOSTRAS_FILTRO_DIGITAL * 4;
    }
  }

  /**
   * @brief Estado já filtrado (debounced) do botão.
   * @return true se pressionado (confirmado), false caso contrário.
   */
  bool estado() const { return _estado; }

private:
  /**
   * @brief Lê o nível bruto do pino e normaliza para "pressionado = true".
   * @return Estado instantâneo do botão, sem filtragem.
   */
  bool readRaw() {
    bool nivel = (digitalRead(_pino) == HIGH);
    return BOTAO_ATIVO_EM_LOW ? !nivel : nivel;
  }

  /** @brief Contadores do filtro de três estágios. */
  struct Filtro {
    int16_t a;  ///< Contador regressivo de acionamento
    int16_t d;  ///< Contador regressivo de desacionamento
    int16_t i;  ///< Contador regressivo de indecisão
  } _f;
  uint8_t _pino = 0;       ///< Pino digital monitorado
  bool    _estado = false; ///< Estado filtrado corrente
};

/* ===========================================================================
 *  7) ESTRUTURAS COMPARTILHADAS E PRIMITIVAS DE SINCRONIZAÇÃO
 * =========================================================================== */
/**
 * @struct Sensores
 * @brief  Estado bruto e filtrado das entradas físicas.
 *
 * @details Produzido por @ref taskLeitura (analógicas/digitais) e por
 *          @ref taskDHT (temperatura/umidade); consumido por
 *          @ref taskProcessamento.
 *
 * @note CONCORRÊNCIA: toda leitura/escrita desta instância global deve ocorrer
 *       sob @ref mtxSensores. Como há dois produtores (taskLeitura e taskDHT) em
 *       campos distintos e um consumidor, o mutex garante consistência do
 *       snapshot lido pelo processamento.
 */
struct Sensores {
  uint16_t acelRaw;   ///< Potenciômetro filtrado (0 .. @ref ADC_MAX)
  uint16_t lumRaw;    ///< LDR filtrado (0 .. @ref ADC_MAX)
  int16_t  tempC;     ///< Temperatura do DHT11 em °C (-127 = sem leitura)
  int16_t  umid;      ///< Umidade do DHT11 em % (-127 = sem leitura)
  bool     emergencia;///< Botão de emergência (debounced)
  bool     freio;     ///< Botão de freio (debounced)
};

/**
 * @struct Telemetria
 * @brief  Pacote de telemetria já processado, pronto para exibição/transmissão.
 *
 * @details Produzido por @ref taskProcessamento; consumido por
 *          @ref taskDisplay (velocidade) e @ref taskComunicacao (JSON).
 *
 * @note CONCORRÊNCIA: acesso serializado por @ref mtxTelemetria.
 * @note O número de sequência NÃO faz parte desta estrutura: ele conta pacotes
 *       efetivamente transmitidos e por isso é gerado dentro da
 *       @ref taskComunicacao (variável local @c tx_seq), e não a cada ciclo de
 *       processamento. Isso evita falso positivo de perda no dashboard, que
 *       interpretaria os saltos (50 Hz de processamento vs 10 Hz de TX) como
 *       pacotes perdidos.
 */
struct Telemetria {
  uint8_t  acelPct;   ///< Acelerador em % (0..100)
  uint8_t  velKmh;    ///< Velocidade mapeada em km/h (0..@ref VEL_MAX_KMH)
  uint8_t  lumPct;    ///< Luminosidade em % (0..100)
  int16_t  tempC;     ///< Temperatura do motor em °C
  int16_t  umid;      ///< Umidade em %
  bool     emergencia;///< Estado da emergência
  bool     freio;     ///< Estado do freio
  bool     pit;       ///< Estado atual do LED do PIT
  bool     bandeira;  ///< Estado atual do LED de bandeira
};

/**
 * @struct Comando
 * @brief  Comando de atuação de LED recebido do dashboard.
 *
 * @details Trafega do callback assíncrono do WebSocket (core 0) até a
 *          @ref taskProcessamento (core 1) através da fila @ref xFilaComandos.
 *
 * @note Campos com valor -1 significam "não alterar"; 0/1 definem o estado.
 */
struct Comando {
  int8_t pit;       ///< -1 = manter; 0/1 = define o LED do PIT
  int8_t bandeira;  ///< -1 = manter; 0/1 = define o LED de bandeira
};

/** @brief Instância global das entradas. @note Protegida por @ref mtxSensores. */
static Sensores    g_sensores   = { 0, 0, -127, -127, false, false };

/** @brief Instância global da telemetria. @note Protegida por @ref mtxTelemetria. */
static Telemetria  g_telemetria = { 0, 0, 0, -127, -127, false, false, false, false };

/**
 * @brief Mutex que serializa o acesso a @ref g_sensores.
 * @note  Tomado por taskLeitura, taskDHT (escrita) e taskProcessamento (leitura).
 */
static SemaphoreHandle_t mtxSensores;

/**
 * @brief Mutex que serializa o acesso a @ref g_telemetria.
 * @note  Tomado por taskProcessamento (escrita), taskDisplay e taskComunicacao
 *        (leitura).
 */
static SemaphoreHandle_t mtxTelemetria;

/**
 * @brief Fila de comandos de LED (produtor: WebSocket/core 0; consumidor:
 *        taskProcessamento/core 1).
 * @note  Desacopla o contexto assíncrono de rede do laço de controle, de modo
 *        que o callback de rede nunca atua diretamente sobre os GPIO de saída.
 */
static QueueHandle_t     xFilaComandos;

/**
 * @brief Estado autoritativo do LED do PIT.
 * @note  Escrito apenas pela @ref taskProcessamento; lido sob @ref mtxTelemetria
 *        após ser copiado para @ref g_telemetria.
 */
static bool g_pit      = false;

/**
 * @brief Estado autoritativo do LED de bandeira.
 * @note  Mesma disciplina de acesso de @ref g_pit.
 */
static bool g_bandeira = false;

/* ===========================================================================
 *  8) SERVIDOR WEB ASSÍNCRONO + WEBSOCKET
 * =========================================================================== */
/** @brief Servidor HTTP assíncrono na porta 80 (serve a página de diagnóstico). */
AsyncWebServer server(80);

/** @brief Endpoint WebSocket "/ws" para telemetria e comandos em tempo real. */
AsyncWebSocket ws("/ws");

/**
 * @brief Página HTML mínima de diagnóstico (sinal de vida no navegador).
 * @note  Armazenada em flash (PROGMEM). O dashboard oficial é o programa Python;
 *        esta página apenas confirma que o WebSocket está no ar.
 */
const char PAGINA_DIAG[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Telemetria ESP32</title>
<style>body{font-family:system-ui;background:#0e1116;color:#e6edf3;margin:0;padding:24px}
.b{background:#171c23;border:1px solid #2a323d;border-radius:8px;padding:16px;max-width:520px}
code{color:#4ea1ff}</style></head><body><div class="b">
<h2>Telemetria ESP32 — vivo</h2>
<p>WebSocket em <code>ws://192.168.4.1/ws</code></p>
<p id="s">conectando...</p>
<pre id="d" style="white-space:pre-wrap;color:#8b949e"></pre>
<script>
let ws=new WebSocket("ws://"+location.host+"/ws");
ws.onopen=()=>document.getElementById("s").textContent="WS conectado";
ws.onclose=()=>document.getElementById("s").textContent="WS fechado";
ws.onmessage=e=>document.getElementById("d").textContent=e.data;
</script></div></body></html>
)HTML";

/**
 * @brief Extrai o valor inteiro 0/1 associado a uma chave em um JSON simples.
 * @param[in] s     String JSON recebida (terminada em nulo).
 * @param[in] chave Chave a localizar, incluindo aspas (ex.: "\"pit\"").
 * @return 0 ou 1 se encontrado; -1 se a chave não existir ou o valor for inválido.
 * @note  Parser minimalista suficiente porque o formato do comando é controlado
 *        pelo próprio dashboard ({"pit":1,"bandeira":0}); dispensa ArduinoJson.
 */
static int8_t extraiComando(const char* s, const char* chave) {
  const char* p = strstr(s, chave);
  if (!p) return -1;
  p += strlen(chave);
  while (*p && *p != '0' && *p != '1') {
    if (*p == ',' || *p == '}') return -1;
    p++;
  }
  if (*p == '0') return 0;
  if (*p == '1') return 1;
  return -1;
}

/**
 * @brief Converte um payload de comando em @ref Comando e o enfileira.
 * @param[in] payload String JSON recebida do dashboard.
 * @return void
 * @note Executa no contexto da tarefa do AsyncTCP (core 0). Usa
 *       @c xQueueSend com timeout zero para nunca bloquear a pilha de rede.
 */
static void onWsData(const char* payload) {
  Comando cmd = { -1, -1 };
  cmd.pit      = extraiComando(payload, "\"pit\"");
  cmd.bandeira = extraiComando(payload, "\"bandeira\"");
  if (cmd.pit != -1 || cmd.bandeira != -1) {
    xQueueSend(xFilaComandos, &cmd, 0);
  }
}

/**
 * @brief Callback de eventos do WebSocket.
 * @param[in] server Ponteiro para o servidor WebSocket que emitiu o evento.
 * @param[in] client Cliente associado ao evento.
 * @param[in] type   Tipo do evento (conexão, desconexão, dados, etc.).
 * @param[in] arg    Metadados do frame (cast para @c AwsFrameInfo em WS_EVT_DATA).
 * @param[in] data   Buffer de dados recebidos.
 * @param[in] len    Comprimento dos dados recebidos.
 * @return void
 * @note  Executa no contexto assíncrono do AsyncTCP (core 0). Apenas frames de
 *        texto completos e não fragmentados são processados; o payload é
 *        copiado para um buffer local terminado em nulo antes do parsing.
 */
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        char buf[160];
        size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
        memcpy(buf, data, n);
        buf[n] = '\0';
        onWsData(buf);
      }
      break;
    }
    default:
      break;
  }
}

/* ===========================================================================
 *  9) DISPLAY 7 SEGMENTOS
 * =========================================================================== */
/**
 * @brief Desativa ambos os dígitos do display.
 * @return void
 * @note Usado entre dígitos para eliminar "ghosting" (rastro do dígito anterior).
 */
static inline void apagaDigitos() {
  digitalWrite(DISPLAY1, DIG_OFF);
  digitalWrite(DISPLAY2, DIG_OFF);
}

/**
 * @brief Escreve um dígito em um dos commons do display.
 * @param[in] valor     Dígito a exibir (0..9; valores fora da faixa viram 0).
 * @param[in] commonPin Common a ativar (@ref DISPLAY1 ou @ref DISPLAY2).
 * @return void
 * @note Sequência anti-ghosting: desativa os dígitos, ajusta os segmentos e só
 *       então ativa o common alvo.
 */
static void escreveDigito(uint8_t valor, uint8_t commonPin) {
  if (valor > 9) valor = 0;
  apagaDigitos();
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SEG_PINS[i], segNum[valor][i] ? SEG_ON : SEG_OFF);
  }
  digitalWrite(commonPin, DIG_ON);
}

/* ===========================================================================
 *  10) TAREFAS (FreeRTOS)
 * =========================================================================== */
/** @name Instâncias de filtro pertencentes à taskLeitura */
///@{
MediaMovel     filtroAcel;    ///< Média móvel do acelerador
MediaMovel     filtroLum;     ///< Média móvel da luminosidade
EntradaDigital botEmergencia; ///< Debounce do botão de emergência
EntradaDigital botFreio;      ///< Debounce do botão de freio
///@}

/**
 * @brief Tarefa de aquisição das entradas (analógicas e digitais).
 * @param[in] pv Parâmetro da tarefa (não utilizado).
 * @return Nunca retorna (laço infinito FreeRTOS).
 *
 * @details Amostra o acelerador e o LDR com média móvel e processa os botões
 *          com o filtro anti-bounce, publicando o resultado em @ref g_sensores.
 *
 * @note Core 1, prioridade 3, período @ref PERIODO_LEITURA_MS via
 *       @c vTaskDelayUntil. Única produtora dos campos analógicos e digitais de
 *       @ref g_sensores; a escrita ocorre sob @ref mtxSensores.
 */
void taskLeitura(void* pv) {
  TickType_t t0 = xTaskGetTickCount();
  for (;;) {
    uint16_t acel = filtroAcel.read();
    uint16_t lum  = filtroLum.read();
    botEmergencia.read();
    botFreio.read();

    if (xSemaphoreTake(mtxSensores, portMAX_DELAY) == pdTRUE) {
      g_sensores.acelRaw    = acel;
      g_sensores.lumRaw     = lum;
      g_sensores.emergencia = botEmergencia.estado();
      g_sensores.freio      = botFreio.estado();
      xSemaphoreGive(mtxSensores);
    }
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_LEITURA_MS));
  }
}

/** @brief Driver do sensor de temperatura/umidade. */
DFRobot_DHT11 DHT11;

/**
 * @brief Tarefa de leitura do DHT11 (isolada por ser bloqueante).
 * @param[in] pv Parâmetro da tarefa (não utilizado).
 * @return Nunca retorna (laço infinito FreeRTOS).
 *
 * @details A biblioteca do DHT11 bloqueia por dezenas de milissegundos durante
 *          a aquisição. Confinar essa leitura em uma tarefa de baixa prioridade
 *          e período longo (@ref PERIODO_DHT_MS) impede que ela afete a
 *          multiplexação do display ou a transmissão.
 *
 * @note Core 1, prioridade 1. Escreve @c tempC e @c umid de @ref g_sensores sob
 *       @ref mtxSensores; é a segunda produtora dessa estrutura (ver
 *       @ref taskLeitura).
 */
void taskDHT(void* pv) {
  TickType_t t0 = xTaskGetTickCount();
  for (;;) {
    DHT11.read(DHT11_PIN);
    int16_t t = (int16_t)DHT11.temperature;
    int16_t h = (int16_t)DHT11.humidity;

    if (xSemaphoreTake(mtxSensores, portMAX_DELAY) == pdTRUE) {
      g_sensores.tempC = t;
      g_sensores.umid  = h;
      xSemaphoreGive(mtxSensores);
    }
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_DHT_MS));
  }
}

/**
 * @brief Tarefa de processamento: regras de controle e atuação dos LEDs.
 * @param[in] pv Parâmetro da tarefa (não utilizado).
 * @return Nunca retorna (laço infinito FreeRTOS).
 *
 * @details Em cada ciclo:
 *  -# drena a fila @ref xFilaComandos e atualiza @ref g_pit / @ref g_bandeira,
 *     refletindo o comando nos GPIO de saída;
 *  -# obtém um snapshot consistente de @ref g_sensores;
 *  -# converte o acelerador em velocidade por mapa fixo, aplicando as regras de
 *     freio (reduz) e emergência (zera);
 *  -# publica o resultado em @ref g_telemetria.
 *
 * @note Core 1, prioridade 2, período @ref PERIODO_PROC_MS. Lê @ref g_sensores
 *       sob @ref mtxSensores e escreve @ref g_telemetria sob @ref mtxTelemetria.
 * @note O número de sequência foi REMOVIDO desta tarefa: contar pacotes aqui
 *       (50 Hz) produzia saltos na numeração recebida pelo dashboard (10 Hz de
 *       TX), interpretados como perda. A contagem agora vive na
 *       @ref taskComunicacao, atrelada à transmissão efetiva.
 */
void taskProcessamento(void* pv) {
  TickType_t t0 = xTaskGetTickCount();

  for (;;) {
    /* 1) Consome todos os comandos pendentes vindos do PC. */
    Comando cmd;
    while (xQueueReceive(xFilaComandos, &cmd, 0) == pdTRUE) {
      if (cmd.pit      != -1) g_pit      = (cmd.pit == 1);
      if (cmd.bandeira != -1) g_bandeira = (cmd.bandeira == 1);
    }
    digitalWrite(PIN_LED_PIT,      g_pit      ? HIGH : LOW);
    digitalWrite(PIN_LED_BANDEIRA, g_bandeira ? HIGH : LOW);

    /* 2) Snapshot dos sensores. */
    Sensores s;
    if (xSemaphoreTake(mtxSensores, portMAX_DELAY) == pdTRUE) {
      s = g_sensores;
      xSemaphoreGive(mtxSensores);
    }

    /* 3) Processamento: percentuais e mapa fixo acelerador -> velocidade. */
    uint8_t acelPct = (uint8_t)(((uint32_t)s.acelRaw * 100) / ADC_MAX);
    uint8_t lumPct  = (uint8_t)(((uint32_t)s.lumRaw  * 100) / ADC_MAX);
    uint16_t vel = ((uint16_t)acelPct * VEL_MAX_KMH) / 100;
    if (s.freio)      vel = (vel * FATOR_FREIO_PCT) / 100;
    if (s.emergencia) vel = 0;
    if (vel > VEL_MAX_KMH) vel = VEL_MAX_KMH;

    /* 3b) Temperatura do motor.
     *  - Simulada: alvo proporcional à carga (acelerador), perseguido com
     *    inércia térmica (filtro exponencial) para subir/descer suavemente.
     *  - Crua: temperatura do DHT11 (somente ambiente). */
    int16_t tempFinal;
#if SIMULAR_TEMP_MOTOR
    static float tempMotor = (float)TEMP_MOTOR_IDLE;  /* persiste entre ciclos */
    float alvo = (float)TEMP_MOTOR_IDLE +
                 ((float)acelPct / 100.0f) * (float)(TEMP_MOTOR_MAX - TEMP_MOTOR_IDLE);
    tempMotor += (alvo - tempMotor) * ((float)TEMP_INERCIA_PCT / 100.0f);
    tempFinal = (int16_t)(tempMotor + 0.5f);
#else
    tempFinal = s.tempC;
#endif

    /* 4) Publica telemetria (sem seq: ver @note da tarefa). */
    if (xSemaphoreTake(mtxTelemetria, portMAX_DELAY) == pdTRUE) {
      g_telemetria.acelPct    = acelPct;
      g_telemetria.velKmh     = (uint8_t)vel;
      g_telemetria.lumPct     = lumPct;
      g_telemetria.tempC      = tempFinal;
      g_telemetria.umid       = s.umid;
      g_telemetria.emergencia = s.emergencia;
      g_telemetria.freio      = s.freio;
      g_telemetria.pit        = g_pit;
      g_telemetria.bandeira   = g_bandeira;
      xSemaphoreGive(mtxTelemetria);
    }

    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_PROC_MS));
  }
}

/**
 * @brief Tarefa de display: multiplexação dos dois dígitos de 7 segmentos.
 * @param[in] pv Parâmetro da tarefa (não utilizado).
 * @return Nunca retorna (laço infinito FreeRTOS).
 *
 * @details Exibe a velocidade de @ref g_telemetria alternando, em round-robin,
 *          a dezena (DISPLAY1) e a unidade (DISPLAY2) a cada ciclo.
 *
 * @note Core 1, prioridade 4 (a mais alta entre as tarefas), período
 *       @ref PERIODO_DISPLAY_MS. A prioridade elevada e o período curto mantêm
 *       a varredura estável, evitando cintilação. Lê @ref g_telemetria sob
 *       @ref mtxTelemetria.
 */
void taskDisplay(void* pv) {
  TickType_t t0 = xTaskGetTickCount();
  uint8_t qualDigito = 0;

  for (;;) {
    uint8_t vel;
    if (xSemaphoreTake(mtxTelemetria, portMAX_DELAY) == pdTRUE) {
      vel = g_telemetria.velKmh;
      xSemaphoreGive(mtxTelemetria);
    }

    uint8_t dezena  = (vel / 10) % 10;
    uint8_t unidade = vel % 10;

    if (qualDigito == 0) escreveDigito(dezena,  DISPLAY1);
    else                 escreveDigito(unidade, DISPLAY2);
    qualDigito ^= 1;

    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_DISPLAY_MS));
  }
}

/**
 * @brief Tarefa de comunicação: serializa e transmite a telemetria por WebSocket.
 * @param[in] pv Parâmetro da tarefa (não utilizado).
 * @return Nunca retorna (laço infinito FreeRTOS).
 *
 * @details A cada ciclo libera clientes mortos e, havendo ao menos um cliente,
 *          obtém um snapshot de @ref g_telemetria, monta o pacote JSON e o
 *          difunde com @c ws.textAll(). O número de sequência @c tx_seq é
 *          incrementado exatamente uma vez por pacote transmitido, garantindo
 *          uma contagem monotônica e sem lacunas no dashboard.
 *
 * @note Core 0 (junto da pilha WiFi/AsyncTCP), prioridade 2, período
 *       @ref PERIODO_TX_MS. Lê @ref g_telemetria sob @ref mtxTelemetria. A
 *       variável @c tx_seq é local e tocada apenas por esta tarefa, dispensando
 *       proteção adicional.
 * @note CORREÇÃO DE PACKET LOSS: ao atrelar a sequência à transmissão (e não ao
 *       processamento a 50 Hz), elimina-se o falso positivo de perda observado
 *       quando a numeração saltava entre pacotes consecutivos.
 */
void taskComunicacao(void* pv) {
  TickType_t t0 = xTaskGetTickCount();
  char buf[200];
  uint32_t tx_seq = 0;   /**< Sequência de pacotes transmitidos. */

  for (;;) {
    ws.cleanupClients();

    if (ws.count() > 0) {
      Telemetria t;
      if (xSemaphoreTake(mtxTelemetria, portMAX_DELAY) == pdTRUE) {
        t = g_telemetria;
        xSemaphoreGive(mtxTelemetria);
      }

      /* Incrementa a sequência no exato escopo do snprintf + textAll. */
      int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"seq\":%lu,"
        "\"vel\":%u,\"acel\":%u,\"temp\":%d,\"umid\":%d,\"lum\":%u,"
        "\"emergencia\":%d,\"freio\":%d,\"pit\":%d,\"bandeira\":%d}",
        (unsigned long)millis(), (unsigned long)(++tx_seq),
        t.velKmh, t.acelPct, t.tempC, t.umid, t.lumPct,
        t.emergencia ? 1 : 0, t.freio ? 1 : 0,
        t.pit ? 1 : 0, t.bandeira ? 1 : 0);

      if (n > 0 && n < (int)sizeof(buf)) {
        ws.textAll(buf, n);
      }
    }

    vTaskDelayUntil(&t0, pdMS_TO_TICKS(PERIODO_TX_MS));
  }
}

/* ===========================================================================
 *  11) SETUP
 * =========================================================================== */
/**
 * @brief Inicialização do sistema (executada uma vez no boot).
 * @return void
 *
 * @details Sequência:
 *  -# configura GPIO de saída (LEDs e relé) em nível seguro;
 *  -# configura os pinos do display como saída;
 *  -# ajusta o ADC (12 bits, atenuação para 0..3,3 V);
 *  -# pré-carrega os filtros das entradas;
 *  -# cria os mutexes e a fila de comandos;
 *  -# sobe a rede (AP ou STA) e o servidor WebSocket;
 *  -# cria as cinco tarefas, distribuindo tempo real no core 1 e rede no core 0.
 *
 * @note A criação dos objetos de sincronização ocorre ANTES da criação das
 *       tarefas, garantindo que nenhum @c xSemaphoreTake / @c xQueueReceive seja
 *       executado sobre um handle ainda nulo.
 * @warning Não chamar @c Serial.begin(): GPIO1/GPIO3 são segmentos do display.
 */
void setup() {
  /* Saídas dos LEDs do piloto em nível seguro. */
  pinMode(PIN_LED_PIT, OUTPUT);
  pinMode(PIN_LED_BANDEIRA, OUTPUT);
  digitalWrite(PIN_LED_PIT, LOW);
  digitalWrite(PIN_LED_BANDEIRA, LOW);

  /* LED RGB: a bandeira usa apenas uma cor (azul). As outras duas cores
   * precisam ser saídas em LOW, senão o LED comum não acende a cor pura. */
  pinMode(RGB_RED, OUTPUT);   digitalWrite(RGB_RED, LOW);
  pinMode(RGB_GREEN, OUTPUT); digitalWrite(RGB_GREEN, LOW);

  /* Relé como saída em LOW e NUNCA acionado (evita brownout). */
  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, LOW);

  /* Pinos do display como saída. */
  for (uint8_t i = 0; i < 8; i++) pinMode(SEG_PINS[i], OUTPUT);
  pinMode(DISPLAY1, OUTPUT);
  pinMode(DISPLAY2, OUTPUT);
  apagaDigitos();

  /* ADC: faixa cheia 0..3,3 V, 12 bits. */
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  /* Filtros (amostram uma vez para pré-carregar a janela). */
  filtroAcel.begin(PIN_ACELERADOR);
  filtroLum.begin(PIN_LUM);
  botEmergencia.begin(PIN_EMERGENCIA);
  botFreio.begin(PIN_FREIO);

  /* Sincronização — criada antes das tarefas. */
  mtxSensores   = xSemaphoreCreateMutex();
  mtxTelemetria = xSemaphoreCreateMutex();
  xFilaComandos = xQueueCreate(8, sizeof(Comando));

  /* Rede. */
  if (USAR_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);       /* IP fixo: 192.168.4.1 */
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    uint32_t ini = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - ini < 10000) {
      delay(100);   /* Apenas no setup, antes das tarefas subirem. */
    }
  }

  /* Desliga o modem-sleep do WiFi: sem ele, o rádio "cochila" entre pacotes e
   * injeta latência periódica que faz a fila de TX do WebSocket encher e a
   * conexão cair em ciclos (sintoma de perda alta em bancada). Com sleep
   * desligado a conexão fica estável e a perda some. */
  WiFi.setSleep(false);

  /* WebSocket + servidor. */
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PAGINA_DIAG);
  });
  server.begin();

  /* Criação das tarefas: tempo real no core 1, rede no core 0.
   * Prioridades: Display(4) > Leitura(3) > Processamento/Comunicacao(2) > DHT(1). */
  xTaskCreatePinnedToCore(taskLeitura,       "leitura", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskDHT,           "dht",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskProcessamento, "proc",    4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskDisplay,       "display", 2048, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskComunicacao,   "comm",    8192, NULL, 2, NULL, 0);
}

/* ===========================================================================
 *  12) LOOP
 * =========================================================================== */
/**
 * @brief Laço ocioso do Arduino.
 * @return void
 * @note Todo o trabalho está nas tarefas FreeRTOS. A @c loopTask do core 1
 *       apenas cede a CPU dormindo, deixando os ciclos para as tarefas de
 *       maior prioridade.
 */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
