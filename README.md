# Telemetria automotiva em ESP32 — visão geral

Autor: **Elias Enns**

## 1. O que é

Um sistema de **telemetria de veículo** rodando em um microcontrolador **ESP32**,
que lê sensores de um carro (acelerador, freio, botão de emergência, temperatura
do motor) e envia esses dados, em tempo real e por Wi-Fi, para um **painel
(dashboard) no notebook**. O painel mostra gráficos ao vivo, grava os dados em
arquivo e ainda permite **acionar dois sinais luminosos** que o operador usa para
se comunicar com o piloto (luz do box e bandeira vermelha).

É a releitura de um trabalho anterior (telemetria de um carro Baja SAE): a mesma
ideia, com a comunicação feita por Wi-Fi em vez de rádio dedicado.

O que cada entrada/saída representa:

| Sinal | De onde vem | Para que serve |
|---|---|---|
| Velocidade | calculada a partir do acelerador | mostrada no display e no gráfico |
| Acelerador | potenciômetro | posição do "pedal" (0–100%) |
| Temperatura e Umidade / Luminosidade | sensor DHT11 / LDR | dados ambientais de apoio |
| Emergência | botão | corta a velocidade exibida |
| Freio | botão | reduz a velocidade exibida |
| Luz do box (PIT) | LED | comandado pelo operador, via painel |
| Bandeira vermelha | LED | comandado pelo operador, via painel |

> Configure `SIMULAR_TEMP_MOTOR` para 1, para temperatura
> **gerada por software** para imitar um motor
> real (parte de ~85 °C em marcha lenta e sobe até ~110 °C sob aceleração, com
> aquecimento/resfriamento graduais).

---

## 2. Por que foi feito assim (as decisões que importam)

**Por que Wi-Fi (e não cabo USB).** A placa didática usada compartilha os pinos
da porta serial (USB) com o display de 7 segmentos. Usar a serial apagaria
partes do display. A saída foi comunicar por Wi-Fi, o que de quebra deixa o
notebook livre de cabos.

**Por que o ESP32 cria a própria rede Wi-Fi.** Para uma apresentação não
depender de roteador, senha de convidado ou TI: o ESP32 sobe como um ponto de
acesso fixo (rede `BAJA_TELEM`, sempre no endereço `192.168.4.1`). Conecta e
funciona. Dá para usar a rede da empresa, mas o modo "rede própria" é o à prova
de imprevistos.

**Por que o trabalho é dividido em "tarefas".** O ESP32 tem dois núcleos e roda
um pequeno sistema operacional de tempo real (FreeRTOS). O projeto separa as
responsabilidades em tarefas independentes — ler sensores, processar, atualizar o
display e comunicar — cada uma no seu ritmo. Assim a leitura dos sensores nunca
trava por causa da rede, e o display nunca pisca por causa de um sensor lento.
As tarefas trocam dados por mecanismos de sincronização que evitam que um núcleo
leia um dado "pela metade" enquanto o outro escreve.

**Por que existem filtros nas leituras.** A placa não tem capacitores, então os
sinais analógicos chegam "sujos" (ruidosos) e os botões "trepidam" ao serem
apertados. Cada leitura passa por um filtro (média para os analógicos,
confirmação por repetição para os botões), o que estabiliza os valores e evita
leituras falsas.

---

## 3. Como usar

### Pré-requisitos
- **Firmware:** Arduino IDE com suporte a ESP32 ("ESP32 Dev Module") e as
  bibliotecas `DFRobot_DHT11`, `ESP Async WebServer` e `AsyncTCP` (instaláveis
  pelo Gerenciador de Bibliotecas).
- **Painel:** Python 3.10+ e as dependências em `requirements.txt`
  (`pip install -r requirements.txt`).

### Passo a passo
1. Abra `telemetria_esp32.ino` na Arduino IDE, selecione a placa ESP32 e **grave**.
2. No notebook, conecte-se à rede Wi-Fi **`BAJA_TELEM`** (senha `baja12345`).
3. Rode o painel: `python telemetria_esp32_dash.py` e abra
   **http://127.0.0.1:8050** no navegador.
4. A barra superior fica **verde / CONECTADO** quando o painel encontra o ESP32.
   O endereço já vem preenchido (`192.168.4.1`); se precisar, troque no campo
   "ESP32" e clique em Conectar.

### O que dá para fazer no painel
- **Ver ao vivo:** um gráfico por sinal, mais indicadores grandes de velocidade,
  acelerador, temperatura e os estados de emergência/freio.
- **Comandar os LEDs do piloto:** os botões "Luz do PIT" e "Bandeira" acendem os
  LEDs no carro. A cor do botão indica a situação — **verde** (ligado e
  confirmado pelo carro), **cinza** (desligado) e **amarelo** (comando em
  trânsito, aguardando confirmação). O comando é reenviado sozinho até o carro
  confirmar, então um tranco de rede não faz o botão "falhar".
- **Gravar e reproduzir:** "Iniciar" começa a gravar tudo em um arquivo
  (`.csv`, na pasta `logs/`); "Parar" encerra. Depois, na opção **Playback**, é
  possível reabrir esse arquivo e reproduzir a sessão inteira respeitando o tempo
  real — útil para revisar uma corrida ou demonstrar sem o carro ligado.

> A gravação é **contínua e independente da conexão**: se o ESP32 reiniciar ou a
> rede oscilar no meio de uma gravação, o arquivo **não é cortado** — ele segue o
> mesmo, e a interrupção fica registrada como uma lacuna que você consegue ver
> depois no playback. Você controla início e fim apenas pelos botões.

---

## 4. Notas técnicas e histórico de correções

**Estabilidade da conexão.** O Wi-Fi do ESP32 vinha com economia de energia
ligada, o que fazia o rádio "cochilar" entre envios e provocava quedas cíclicas
da conexão (e perda alta em bancada). A economia de energia foi desligada e a
taxa de envio reduzida para 10 pacotes/segundo (suficiente para os gráficos),
deixando o link estável.

**Contagem de pacotes correta.** A numeração dos pacotes agora é feita no momento
do envio (e não a cada ciclo interno de cálculo), de modo que o painel mede a
perda de pacotes corretamente, sem falsos positivos.

**Confiabilidade dos comandos de LED.** Os botões usam um esquema de
"alvo × confirmado": o painel insiste no comando até o carro confirmar o novo
estado, em vez de enviar uma única vez e desistir.

**Disposição dos botões/LEDs.** Para contornar um botão com mau contato no
hardware, a pinagem foi reorganizada (LEDs nas posições 1 e 2; botões nas
posições 3 e 4), o que também evita um pino sensível durante a energização.

### Em uma linha, para cada arquivo
- `telemetria_esp32.ino` — o programa que roda no ESP32 (documentado em padrão Doxygen).
- `telemetria_esp32_dash.py` — o painel que roda no notebook.
- `requirements.txt` — bibliotecas Python necessárias para o painel.
