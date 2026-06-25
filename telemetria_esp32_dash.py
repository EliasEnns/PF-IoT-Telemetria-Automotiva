#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dashboard de Telemetria — ESP32 (exercício IoT)
================================================
Versão enxuta e específica do dashboard do TCC, adaptada para o firmware
ESP32 deste exercício. A fonte de dados deixa de ser USB serial e passa a ser
um WebSocket (o ESP32 roda um servidor WS assíncrono).

O que mudou em relação ao dashboard original (telemetria_baja.py):
  - Aquisição via WebSocket (websocket-client) no lugar de pyserial.
  - Catálogo específico das entradas desta placa (sem calibração, sem 3D).
  - UM gráfico dedicado para cada entrada (velocidade, acelerador, temp,
    umidade, luminosidade) + um gráfico de estados digitais (emergência/freio).
  - Controle dos LEDs do piloto (luz do PIT e bandeira) enviado pelo MESMO
    WebSocket — comunicação bidirecional.
  - Mantém o que era legal para a apresentação: gravação CSV em tempo real,
    playback respeitando o tempo entre pacotes, filtro EMA e engine de alertas.

Protocolo (definido no firmware):
  Telemetria recebida (texto JSON), ~30 Hz:
    {"ts":<ms>,"seq":<n>,"vel":..,"acel":..,"temp":..,"umid":..,"lum":..,
     "emergencia":0|1,"freio":0|1,"pit":0|1,"bandeira":0|1}
  Comando enviado (ao clicar nos controles):
    {"pit":0|1,"bandeira":0|1}

Como rodar:
    pip install -r requirements.txt
    python telemetria_esp32_dash.py
    -> http://127.0.0.1:8050

Conexão com o ESP32:
    - Modo AP (usar p/ apresentação): conecte o notebook na rede
      "BAJA_TELEM" senha baja12345 e use o host 192.168.4.1 (padrão abaixo).
    - O host pode ser trocado na própria interface (campo "ESP32") sem
      reiniciar o programa.

requirements.txt:
    dash>=2.16
    plotly>=5.18
    websocket-client>=1.7
"""

from __future__ import annotations

import csv
import json
import logging
import threading
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import plotly.graph_objects as go
import websocket  # websocket-client
from dash import (
    Dash,
    Input,
    Output,
    State,
    ctx,
    dash_table,
    dcc,
    html,
    no_update,
)

# =============================================================================
# SECTION 1 — CONFIG & LOGGING
# =============================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logging.getLogger("werkzeug").setLevel(logging.WARNING)
log = logging.getLogger("telemetria")

# ---- Conexão com o ESP32 ----------------------------------------------------
ESP32_HOST_DEFAULT: str = "192.168.4.1"   # IP do AP do ESP32
ESP32_WS_PATH: str = "/ws"
ESP32_WS_PORT: int = 80
RECONNECT_DELAY_S: float = 2.0
WS_PING_INTERVAL_S: int = 20              # mantém a conexão viva / detecta queda
WS_PING_TIMEOUT_S: int = 12               # tolerância antes de declarar a conexão morta

# ---- Sequence tracking ------------------------------------------------------
SEQ_MODULO: int = 2 ** 32
SEQ_WRAPAROUND_WINDOW: int = 100

# ---- UI / Performance -------------------------------------------------------
UI_TICK_MS: int = 300
MAX_GRAPH_POINTS: int = 200
MAX_PACKETS_BUFFER: int = 4000
MAX_ALERT_LOG: int = 200

# ---- Filesystem -------------------------------------------------------------
BASE_DIR: Path = Path(__file__).resolve().parent
CSV_DIR: Path = BASE_DIR / "logs"
ALERTS_FILE: Path = BASE_DIR / "config_alertas.json"
FILTERS_FILE: Path = BASE_DIR / "config_filtros.json"

# ---- Catálogo de variáveis numéricas (cada uma ganha um gráfico) -----------
TELEMETRY_VARIABLES: dict[str, dict[str, Any]] = {
    "vel":  {"label": "Velocidade",   "unit": "km/h", "ymin": 0, "ymax": 100},
    "acel": {"label": "Acelerador",   "unit": "%",    "ymin": 0, "ymax": 100},
    "temp": {"label": "Temp. Motor",  "unit": "°C",   "ymin": 0, "ymax": 60},
    "umid": {"label": "Umidade",      "unit": "%",    "ymin": 0, "ymax": 100},
    "lum":  {"label": "Luminosidade", "unit": "%",    "ymin": 0, "ymax": 100},
}
VAR_KEYS: list[str] = list(TELEMETRY_VARIABLES.keys())

# Estados digitais reportados pelo ESP (entradas)
DIGITAL_KEYS: list[str] = ["emergencia", "freio"]
DIGITAL_LABELS: dict[str, str] = {"emergencia": "Emergência", "freio": "Freio"}

# Saídas controláveis (LEDs do piloto)
OUTPUT_KEYS: list[str] = ["pit", "bandeira"]
OUTPUT_LABELS: dict[str, str] = {"pit": "Luz do PIT", "bandeira": "Bandeira Vermelha"}

# Todas as chaves que aparecem no CSV (na ordem)
CSV_FIELDS: list[str] = ["pc_ts", "ts", "seq"] + VAR_KEYS + DIGITAL_KEYS + OUTPUT_KEYS

# ---- Paleta dark ------------------------------------------------------------
COL_BG     = "#0e1116"
COL_PANEL  = "#171c23"
COL_PANEL2 = "#1d242c"
COL_BORDER = "#2a323d"
COL_TEXT   = "#e6edf3"
COL_MUTED  = "#8b949e"
COL_ACCENT = "#4ea1ff"
COL_OK     = "#22d07a"
COL_WARN   = "#f5b942"
COL_ERR    = "#ff4d5e"
COL_PURPLE = "#b48cff"

PLOTLY_TEMPLATE = "plotly_dark"

# Cor de cada gráfico (uma por variável)
VAR_COLORS: dict[str, str] = {
    "vel": COL_ACCENT, "acel": COL_OK, "temp": COL_ERR,
    "umid": COL_PURPLE, "lum": COL_WARN,
}

ALERT_OPERATORS: list[str] = [">", "<", ">=", "<=", "=="]

# =============================================================================
# SECTION 2 — FUNÇÕES PURAS
# =============================================================================

def track_sequence(last_seq, new_seq, modulo=SEQ_MODULO, window=SEQ_WRAPAROUND_WINDOW):
    """Rastreamento de sequência tolerante a wraparound (igual ao do TCC).
    Retorna (novo_last_seq, perdidos_neste_passo, foi_reset)."""
    if last_seq is None:
        return new_seq, 0, False
    diff = (new_seq - last_seq) % modulo
    if diff == 0:
        return last_seq, 0, False
    if diff > modulo // 2:
        return new_seq, 0, True
    if diff > window and diff < modulo - window:
        return new_seq, 0, True
    return new_seq, diff - 1, False


def ema_step(prev, new, alpha):
    """Média Móvel Exponencial. alpha em (0,1]; 1.0 = passthrough."""
    if prev is None or alpha >= 0.999:
        return new
    return alpha * new + (1.0 - alpha) * prev


def evaluate_alert(value, operator, threshold):
    if operator == ">":  return value > threshold
    if operator == "<":  return value < threshold
    if operator == ">=": return value >= threshold
    if operator == "<=": return value <= threshold
    if operator == "==": return value == threshold
    return False


# =============================================================================
# SECTION 3 — EMA FILTER (suavização visual, hot-reload por variável)
# =============================================================================

class EMAFilter:
    """Filtro passa-baixa por variável (suavização do traço no gráfico).
    O firmware já filtra na origem; este EMA é um polimento opcional no PC."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._alpha: dict[str, float] = {k: 1.0 for k in VAR_KEYS}
        self._state: dict[str, Optional[float]] = {k: None for k in VAR_KEYS}
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            self._save_unlocked()
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            for k in VAR_KEYS:
                a = float(raw.get(k, 1.0))
                self._alpha[k] = max(0.001, min(1.0, a))
            log.info("Filtros EMA carregados de %s", self._path.name)
        except Exception as e:
            log.error("Falha ao carregar filtros EMA: %s", e)

    def _save_unlocked(self) -> None:
        try:
            self._path.write_text(json.dumps(self._alpha, indent=2), encoding="utf-8")
        except OSError as e:
            log.error("Falha ao gravar filtros EMA: %s", e)

    def reset_state(self) -> None:
        with self._lock:
            for k in VAR_KEYS:
                self._state[k] = None

    def apply(self, var: str, value: float) -> float:
        with self._lock:
            alpha = self._alpha.get(var, 1.0)
            new = ema_step(self._state.get(var), value, alpha)
            self._state[var] = new
            return new

    def all_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {"variavel": k, "label": TELEMETRY_VARIABLES[k]["label"],
                 "alpha": self._alpha.get(k, 1.0)}
                for k in VAR_KEYS
            ]

    def update_from_table(self, rows: list[dict[str, Any]]) -> None:
        with self._lock:
            for row in rows:
                var = row.get("variavel")
                if var not in TELEMETRY_VARIABLES:
                    continue
                try:
                    a = float(row.get("alpha", 1.0))
                    self._alpha[var] = max(0.001, min(1.0, a))
                except (TypeError, ValueError):
                    log.warning("Alpha inválido para %s: %s", var, row)
            self._save_unlocked()
        log.info("Filtros EMA atualizados e persistidos.")


# =============================================================================
# SECTION 4 — ALERT ENGINE (borda de subida, log persistido)
# =============================================================================

@dataclass
class AlertRule:
    id: str
    variable: str
    operator: str
    threshold: float
    message: str
    enabled: bool = True


@dataclass
class AlertEvent:
    timestamp: str
    rule_id: str
    variable: str
    value: float
    threshold: float
    operator: str
    message: str


class AlertEngine:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._rules: dict[str, AlertRule] = {}
        self._active: dict[str, bool] = {}
        self._events: deque[AlertEvent] = deque(maxlen=MAX_ALERT_LOG)
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            self._seed_defaults()
            self._save_unlocked()
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            self._rules = {}
            for r in raw.get("rules", []):
                rid = str(r.get("id", uuid.uuid4().hex[:8]))
                self._rules[rid] = AlertRule(
                    id=rid, variable=str(r["variable"]), operator=str(r["operator"]),
                    threshold=float(r["threshold"]), message=str(r.get("message", "")),
                    enabled=bool(r.get("enabled", True)),
                )
            log.info("Regras de alerta carregadas: %d", len(self._rules))
        except Exception as e:
            log.error("Falha ao carregar alertas: %s — usando defaults", e)
            self._seed_defaults()

    def _seed_defaults(self) -> None:
        defaults = [
            AlertRule(uuid.uuid4().hex[:8], "temp", ">", 45.0, "Motor quente"),
            AlertRule(uuid.uuid4().hex[:8], "vel",  ">", 90.0, "Velocidade alta"),
        ]
        self._rules = {r.id: r for r in defaults}

    def _save_unlocked(self) -> None:
        try:
            payload = {"rules": [asdict(r) for r in self._rules.values()]}
            self._path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except OSError as e:
            log.error("Falha ao gravar alertas: %s", e)

    def evaluate(self, packet: dict[str, Any]) -> list[AlertEvent]:
        new_events: list[AlertEvent] = []
        with self._lock:
            for rid, rule in self._rules.items():
                if not rule.enabled:
                    continue
                key = f"{rule.variable}_filt"
                if key not in packet:
                    key = rule.variable
                if key not in packet or packet[key] is None:
                    continue
                try:
                    val = float(packet[key])
                except (TypeError, ValueError):
                    continue
                triggered = evaluate_alert(val, rule.operator, rule.threshold)
                was_active = self._active.get(rid, False)
                if triggered and not was_active:
                    evt = AlertEvent(
                        timestamp=datetime.now().isoformat(timespec="milliseconds"),
                        rule_id=rid, variable=rule.variable, value=val,
                        threshold=rule.threshold, operator=rule.operator,
                        message=rule.message or f"{rule.variable} {rule.operator} {rule.threshold}",
                    )
                    self._events.append(evt)
                    new_events.append(evt)
                    log.warning("ALERTA: %s (%.2f %s %.2f)",
                                rule.message, val, rule.operator, rule.threshold)
                self._active[rid] = triggered
        return new_events

    def active_count(self) -> int:
        with self._lock:
            return sum(1 for v in self._active.values() if v)

    def active_rules(self) -> list[AlertRule]:
        with self._lock:
            return [self._rules[rid] for rid, a in self._active.items()
                    if a and rid in self._rules]

    def all_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {"id": r.id, "variable": r.variable, "operator": r.operator,
                 "threshold": r.threshold, "message": r.message,
                 "enabled": "sim" if r.enabled else "não"}
                for r in self._rules.values()
            ]

    def update_from_table(self, rows: list[dict[str, Any]]) -> None:
        new_rules: dict[str, AlertRule] = {}
        for row in rows:
            try:
                rid = str(row.get("id") or uuid.uuid4().hex[:8])
                var = str(row.get("variable", "")).strip()
                op = str(row.get("operator", ">")).strip()
                if var not in TELEMETRY_VARIABLES:
                    continue
                if op not in ALERT_OPERATORS:
                    continue
                thr = float(row.get("threshold", 0.0))
                msg = str(row.get("message", "")).strip()
                enabled_raw = str(row.get("enabled", "sim")).strip().lower()
                enabled = enabled_raw in ("sim", "true", "1", "yes", "y", "on")
                new_rules[rid] = AlertRule(rid, var, op, thr, msg, enabled)
            except Exception as e:
                log.warning("Linha de alerta inválida ignorada: %s (%s)", row, e)
        with self._lock:
            self._rules = new_rules
            self._active = {rid: a for rid, a in self._active.items() if rid in new_rules}
            self._save_unlocked()
        log.info("Regras de alerta atualizadas: %d total", len(new_rules))

    def events_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {"timestamp": e.timestamp, "variable": e.variable,
                 "condicao": f"{e.operator} {e.threshold:.2f}",
                 "valor": f"{e.value:.2f}", "mensagem": e.message}
                for e in reversed(self._events)
            ]

    def clear_log(self) -> None:
        with self._lock:
            self._events.clear()
        log.info("Log de alertas limpo.")


# =============================================================================
# SECTION 5 — DATA STORE (buffer thread-safe)
# =============================================================================

class DataStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._packets: deque[dict[str, Any]] = deque(maxlen=MAX_PACKETS_BUFFER)
        self._last_seq: Optional[int] = None
        self._lost_count: int = 0
        self._reset_count: int = 0
        self._total_rx: int = 0

    def reset_session(self) -> None:
        with self._lock:
            self._packets.clear()
            self._last_seq = None
            self._lost_count = 0
            self._reset_count = 0
            self._total_rx = 0

    def push(self, packet: dict[str, Any]) -> tuple[int, bool]:
        with self._lock:
            seq = packet.get("seq")
            lost = 0
            was_reset = False
            if isinstance(seq, int):
                self._last_seq, lost, was_reset = track_sequence(self._last_seq, seq)
                self._lost_count += lost
                if was_reset:
                    self._reset_count += 1
            self._total_rx += 1
            self._packets.append(packet)
        return lost, was_reset

    def snapshot_stats(self) -> dict[str, Any]:
        with self._lock:
            denom = self._lost_count + self._total_rx
            loss_pct = (self._lost_count / denom * 100.0) if denom > 0 else 0.0
            last = self._packets[-1] if self._packets else None
            return {
                "total_rx": self._total_rx, "lost": self._lost_count,
                "loss_pct": loss_pct, "resets": self._reset_count,
                "last_packet": last,
            }

    def get_recent_samples(self, var: str, n: int,
                           use_filtered: bool = True) -> list[tuple[float, float]]:
        key_pref = f"{var}_filt" if use_filtered else var
        out: list[tuple[float, float]] = []
        with self._lock:
            for pkt in self._packets:
                ts = pkt.get("ts")
                if ts is None:
                    continue
                val = pkt.get(key_pref, pkt.get(var))
                if val is None:
                    continue
                try:
                    out.append((float(ts), float(val)))
                except (TypeError, ValueError):
                    continue
        return out[-n:] if len(out) > n else out

    def get_recent_digital(self, n: int) -> dict[str, list[tuple[float, int]]]:
        """Séries 0/1 das entradas digitais."""
        out: dict[str, list[tuple[float, int]]] = {k: [] for k in DIGITAL_KEYS}
        with self._lock:
            for pkt in self._packets:
                ts = pkt.get("ts")
                if ts is None:
                    continue
                for k in DIGITAL_KEYS:
                    v = pkt.get(k)
                    if v is None:
                        continue
                    try:
                        out[k].append((float(ts), int(v)))
                    except (TypeError, ValueError):
                        continue
        for k in DIGITAL_KEYS:
            if len(out[k]) > n:
                out[k] = out[k][-n:]
        return out


# =============================================================================
# SECTION 6 — GRAPH STATE TRACKER (suporte ao dash.Patch)
# =============================================================================

class GraphStateTracker:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_ts: dict[str, Optional[float]] = {}

    def reset(self, graph_id: str) -> None:
        with self._lock:
            self._last_ts[graph_id] = None

    def reset_all(self) -> None:
        with self._lock:
            self._last_ts.clear()

    def get(self, graph_id: str) -> Optional[float]:
        with self._lock:
            return self._last_ts.get(graph_id)

    def set(self, graph_id: str, last_ts: float) -> None:
        with self._lock:
            self._last_ts[graph_id] = last_ts


# =============================================================================
# SECTION 7 — WEBSOCKET MANAGER (substitui o SerialManager)
# =============================================================================

class WebSocketManager:
    """
    Mantém conexão WebSocket com o ESP32 em thread daemon, com reconexão
    automática. Recebe telemetria JSON, aplica EMA, alimenta DataStore + CSV e
    avalia alertas. Envia comandos (LEDs) pela mesma conexão.

    Espelha o SerialManager do TCC: o transporte é WS em vez de USB CDC, mas o
    pipeline (parse -> filtro -> store -> csv -> alertas) é o mesmo.
    """

    def __init__(self, store: DataStore, ema: EMAFilter, alert_engine: AlertEngine) -> None:
        self._store = store
        self._ema = ema
        self._alerts = alert_engine

        self._host: str = ESP32_HOST_DEFAULT
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = threading.Event()

        self._ws_app: Optional[websocket.WebSocketApp] = None
        self._ws_lock = threading.Lock()

        self._csv_file: Optional[Any] = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._csv_path: Optional[Path] = None
        self._csv_lock = threading.Lock()
        self._logging_enabled: bool = False
        self._logging_paused: bool = False

    # ---- ciclo de vida -----------------------------------------------------

    def set_host(self, host: str) -> None:
        host = (host or "").strip()
        if host and host != self._host:
            self._host = host
            log.info("Host do ESP32 alterado para %s — reconectando.", host)
            self.restart()

    def host(self) -> str:
        return self._host

    def url(self) -> str:
        return f"ws://{self._host}:{ESP32_WS_PORT}{ESP32_WS_PATH}"

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ws-rx", daemon=True)
        self._thread.start()
        log.info("WebSocketManager iniciado (%s).", self.url())

    def stop(self) -> None:
        self._stop_event.set()
        with self._ws_lock:
            if self._ws_app:
                try:
                    self._ws_app.close()
                except Exception:
                    pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._connected.clear()
        self._logging_enabled = False
        self._logging_paused = False
        self._close_csv()
        log.info("WebSocketManager parado.")

    def restart(self) -> None:
        was_logging = self._logging_enabled
        self.stop()
        self.start()
        if was_logging:
            self.start_logging()

    def is_connected(self) -> bool:
        return self._connected.is_set()

    def csv_path(self) -> Optional[Path]:
        return self._csv_path

    # ---- envio de comandos -------------------------------------------------

    def send_command(self, pit: Optional[int] = None,
                     bandeira: Optional[int] = None) -> bool:
        """Envia {"pit":..,"bandeira":..} ao ESP. Campos None são omitidos."""
        payload: dict[str, int] = {}
        if pit is not None:
            payload["pit"] = int(bool(pit))
        if bandeira is not None:
            payload["bandeira"] = int(bool(bandeira))
        if not payload:
            return False
        msg = json.dumps(payload)
        with self._ws_lock:
            if self._ws_app and self._connected.is_set():
                try:
                    self._ws_app.send(msg)
                    return True
                except Exception as e:
                    log.warning("Falha ao enviar comando: %s", e)
        return False

    # ---- controle de logging ----------------------------------------------

    def start_logging(self) -> Optional[str]:
        self._logging_paused = False
        self._logging_enabled = True
        # Abre o arquivo no ato, mesmo sem conexão: a gravação é independente do
        # link. As linhas entram quando os dados chegam; o arquivo permanece o
        # mesmo através de quedas/reconexões/resets do ESP até o usuário parar.
        if not self._csv_file:
            self._open_csv()
        name = self._csv_path.name if self._csv_path else None
        log.info("Logging habilitado. Arquivo: %s", name or "(falha ao abrir)")
        return name

    def pause_logging(self) -> None:
        self._logging_paused = True

    def resume_logging(self) -> None:
        self._logging_paused = False

    def stop_logging(self) -> None:
        self._logging_enabled = False
        self._logging_paused = False
        self._close_csv()

    def is_logging_paused(self) -> bool:
        return self._logging_enabled and self._logging_paused

    # ---- CSV interno -------------------------------------------------------

    def _open_csv(self) -> None:
        CSV_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = CSV_DIR / f"telemetria_{ts}.csv"
        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._csv_writer.writeheader()
        self._csv_file.flush()
        log.info("CSV aberto: %s", self._csv_path.name)

    def _close_csv(self) -> None:
        with self._csv_lock:
            if self._csv_file:
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except OSError as e:
                    log.warning("Erro ao fechar CSV: %s", e)
            self._csv_file = None
            self._csv_writer = None

    def _write_csv(self, packet: dict[str, Any]) -> None:
        if not self._logging_enabled or self._logging_paused:
            return
        if not self._csv_writer or not self._csv_file:
            return
        try:
            row = {
                "pc_ts": packet.get("pc_ts", ""),
                "ts": packet.get("ts", ""),
                "seq": packet.get("seq", ""),
            }
            for k in VAR_KEYS:
                row[k] = packet.get(f"{k}_raw", packet.get(k, ""))
            for k in DIGITAL_KEYS + OUTPUT_KEYS:
                row[k] = packet.get(k, "")
            with self._csv_lock:
                self._csv_writer.writerow(row)
                self._csv_file.flush()
        except Exception as e:
            log.warning("Falha ao escrever linha CSV: %s", e)

    # ---- pipeline de processamento ----------------------------------------

    def _process_message(self, message: str) -> None:
        message = message.strip()
        if not message:
            return
        try:
            raw = json.loads(message)
            if not isinstance(raw, dict):
                return
        except json.JSONDecodeError:
            return  # frame truncado durante boot/reset: descarta

        packet: dict[str, Any] = {
            "pc_ts": datetime.now().isoformat(timespec="milliseconds"),
        }
        for k in ("ts", "seq"):
            if k in raw:
                try:
                    packet[k] = int(raw[k])
                except (TypeError, ValueError):
                    pass

        for var in VAR_KEYS:
            if var not in raw:
                continue
            try:
                raw_val = float(raw[var])
            except (TypeError, ValueError):
                continue
            filtered = self._ema.apply(var, raw_val)
            packet[f"{var}_raw"] = raw_val
            packet[var] = raw_val
            packet[f"{var}_filt"] = filtered

        for k in DIGITAL_KEYS + OUTPUT_KEYS:
            if k in raw:
                try:
                    packet[k] = int(raw[k])
                except (TypeError, ValueError):
                    pass

        self._write_csv(packet)
        self._store.push(packet)
        self._alerts.evaluate(packet)

    # ---- callbacks do websocket-client -------------------------------------

    def _on_open(self, _ws: Any) -> None:
        self._connected.set()
        log.info("WS conectado em %s", self.url())
        # Não abre/fecha CSV aqui: a gravação é governada pelos botões Iniciar/
        # Parar, não pelo estado do link. Assim uma reconexão continua no mesmo
        # arquivo, preservando o gap da instabilidade para o playback.

    def _on_message(self, _ws: Any, message: Any) -> None:
        try:
            if isinstance(message, bytes):
                message = message.decode("utf-8", errors="ignore")
            self._process_message(message)
        except Exception as e:
            log.error("Erro processando mensagem: %s", e)

    def _on_error(self, _ws: Any, error: Any) -> None:
        log.debug("WS erro: %s", error)

    def _on_close(self, _ws: Any, *_args: Any) -> None:
        self._connected.clear()
        # NÃO fecha o CSV: a queda do link não deve interromper a gravação. O
        # arquivo segue aberto e a reconexão volta a escrever nele — o intervalo
        # sem dados fica registrado como o gap da instabilidade.
        log.info("WS desconectado (gravação preservada).")

    def _run(self) -> None:
        while not self._stop_event.is_set():
            url = self.url()
            try:
                app = websocket.WebSocketApp(
                    url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                with self._ws_lock:
                    self._ws_app = app
                app.run_forever(ping_interval=WS_PING_INTERVAL_S,
                                ping_timeout=WS_PING_TIMEOUT_S)
            except Exception as e:
                log.debug("run_forever saiu com: %s", e)
            finally:
                self._connected.clear()
                with self._ws_lock:
                    self._ws_app = None
            if not self._stop_event.is_set():
                time.sleep(RECONNECT_DELAY_S)


# =============================================================================
# SECTION 8 — PLAYBACK MANAGER (reproduz CSV respeitando o tempo)
# =============================================================================

class PlaybackManager:
    def __init__(self, store: DataStore, ema: EMAFilter, alert_engine: AlertEngine) -> None:
        self._store = store
        self._ema = ema
        self._alerts = alert_engine
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._lock = threading.Lock()
        self._speed: float = 1.0
        self._csv_path: Optional[Path] = None
        self._running = False
        self._progress = 0.0

    def is_running(self) -> bool:
        return self._running

    def is_paused(self) -> bool:
        return self._pause_event.is_set() and self._running

    def progress(self) -> float:
        return self._progress

    def csv_path(self) -> Optional[Path]:
        return self._csv_path

    def set_speed(self, speed: float) -> None:
        with self._lock:
            self._speed = max(0.1, min(20.0, float(speed)))
        log.info("Playback speed = %.2fx", self._speed)

    def start(self, csv_path: Path, speed: float = 1.0) -> bool:
        if self._thread and self._thread.is_alive():
            self.stop()
        if not csv_path.exists():
            log.error("Arquivo de playback não encontrado: %s", csv_path)
            return False
        self._csv_path = csv_path
        self._speed = max(0.1, min(20.0, float(speed)))
        self._stop_event.clear()
        self._pause_event.clear()
        self._ema.reset_state()
        self._store.reset_session()
        self._running = True
        self._progress = 0.0
        self._thread = threading.Thread(target=self._run, name="playback", daemon=True)
        self._thread.start()
        log.info("Playback iniciado: %s (%.2fx)", csv_path.name, self._speed)
        return True

    def pause(self) -> None:
        self._pause_event.set()

    def resume(self) -> None:
        self._pause_event.clear()

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._running = False

    def _run(self) -> None:
        try:
            with self._csv_path.open("r", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            n = len(rows)
            if n == 0:
                log.warning("CSV de playback vazio: %s", self._csv_path)
                self._running = False
                return
            prev_ts: Optional[float] = None
            for idx, row in enumerate(rows):
                if self._stop_event.is_set():
                    break
                while self._pause_event.is_set() and not self._stop_event.is_set():
                    time.sleep(0.05)
                if self._stop_event.is_set():
                    break

                try:
                    ts = float(row.get("ts", 0))
                except (TypeError, ValueError):
                    ts = 0.0
                if prev_ts is not None and ts > prev_ts:
                    delta_ms = ts - prev_ts
                    with self._lock:
                        speed = self._speed
                    end = time.monotonic() + (delta_ms / 1000.0) / speed
                    while time.monotonic() < end:
                        if self._stop_event.is_set():
                            return
                        time.sleep(min(0.05, max(0.0, end - time.monotonic())))
                prev_ts = ts

                packet: dict[str, Any] = {
                    "pc_ts": datetime.now().isoformat(timespec="milliseconds"),
                }
                for k in ("ts", "seq"):
                    v = row.get(k)
                    if v not in (None, ""):
                        try:
                            packet[k] = int(float(v))
                        except (TypeError, ValueError):
                            pass
                for var in VAR_KEYS:
                    v = row.get(var)
                    if v in (None, ""):
                        continue
                    try:
                        raw_val = float(v)
                    except (TypeError, ValueError):
                        continue
                    filtered = self._ema.apply(var, raw_val)
                    packet[f"{var}_raw"] = raw_val
                    packet[var] = raw_val
                    packet[f"{var}_filt"] = filtered
                for k in DIGITAL_KEYS + OUTPUT_KEYS:
                    v = row.get(k)
                    if v not in (None, ""):
                        try:
                            packet[k] = int(float(v))
                        except (TypeError, ValueError):
                            pass

                self._store.push(packet)
                self._alerts.evaluate(packet)
                self._progress = (idx + 1) / n

            self._progress = 1.0
            log.info("Playback concluído.")
        except Exception as e:
            log.error("Erro no playback: %s", e)
        finally:
            self._running = False


# =============================================================================
# SECTION 9 — FIGURAS PLOTLY
# =============================================================================

# NOTA DE DESIGN: usamos go.Scatter (SVG), NÃO go.Scattergl (WebGL).
# Cada dcc.Graph com Scattergl abre um contexto WebGL no navegador; com 6
# gráficos isso estourava o limite de contextos do browser e os gráficos
# "sumiam". Em SVG, 200 pontos por gráfico a 10 Hz é leve e 100% estável.
# Também reconstruímos a figura inteira a cada tick (em vez de dash.Patch),
# o que é mais robusto entre versões de Dash/Plotly; o uirevision preserva
# zoom/pan do usuário mesmo recriando a figura.

def make_single_figure(var: str,
                       samples: Optional[list[tuple[float, float]]] = None) -> go.Figure:
    """Gráfico dedicado de uma variável. Se `samples` for dado, já o desenha."""
    meta = TELEMETRY_VARIABLES[var]
    color = VAR_COLORS.get(var, COL_ACCENT)
    xs = [s[0] for s in samples] if samples else []
    ys = [s[1] for s in samples] if samples else []
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=xs, y=ys, mode="lines", name=meta["label"],
        line=dict(color=color, width=2), fill="tozeroy",
        hovertemplate="%{x} ms<br>%{y:.1f} " + meta["unit"] + "<extra></extra>",
    ))
    fig.update_layout(
        template=PLOTLY_TEMPLATE, paper_bgcolor=COL_PANEL, plot_bgcolor=COL_PANEL,
        margin=dict(l=44, r=14, t=28, b=30),
        title=dict(text=f"{meta['label']} ({meta['unit']})",
                   font=dict(size=13, color=COL_TEXT), x=0.01, xanchor="left"),
        xaxis=dict(title=None, gridcolor=COL_BORDER, zerolinecolor=COL_BORDER,
                   tickfont=dict(size=9, color=COL_MUTED)),
        yaxis=dict(range=[meta["ymin"], meta["ymax"]], gridcolor=COL_BORDER,
                   zerolinecolor=COL_BORDER, tickfont=dict(size=9, color=COL_MUTED)),
        showlegend=False, uirevision=f"graph-{var}", height=200,
    )
    return fig


def make_digital_figure(
        series: Optional[dict[str, list[tuple[float, int]]]] = None) -> go.Figure:
    """Gráfico de estados digitais (emergência/freio) como degraus 0/1.
    Se `series` for dado, já o desenha (freio deslocado p/ não sobrepor)."""
    offsets = {"emergencia": 0.0, "freio": 1.3}
    colors = {"emergencia": COL_ERR, "freio": COL_WARN}
    fig = go.Figure()
    for k in DIGITAL_KEYS:
        pts = series.get(k, []) if series else []
        xs = [p[0] for p in pts]
        ys = [p[1] + offsets[k] for p in pts]
        fig.add_trace(go.Scatter(
            x=xs, y=ys, mode="lines", name=DIGITAL_LABELS[k],
            line=dict(color=colors[k], width=2, shape="hv"),
            hovertemplate=DIGITAL_LABELS[k] + ": %{y}<extra></extra>",
        ))
    fig.update_layout(
        template=PLOTLY_TEMPLATE, paper_bgcolor=COL_PANEL, plot_bgcolor=COL_PANEL,
        margin=dict(l=44, r=14, t=28, b=30),
        title=dict(text="Entradas digitais (0/1)",
                   font=dict(size=13, color=COL_TEXT), x=0.01, xanchor="left"),
        xaxis=dict(gridcolor=COL_BORDER, zerolinecolor=COL_BORDER,
                   tickfont=dict(size=9, color=COL_MUTED)),
        yaxis=dict(range=[-0.3, 2.6], tickvals=[0, 1, 1.3, 2.3],
                   ticktext=["0", "1", "0", "1"], gridcolor=COL_BORDER,
                   tickfont=dict(size=9, color=COL_MUTED)),
        legend=dict(orientation="h", y=1.18, x=0.0, bgcolor="rgba(0,0,0,0)",
                    font=dict(size=10, color=COL_MUTED)),
        uirevision="graph-digitais", height=200,
    )
    return fig


# =============================================================================
# SECTION 10 — UI HELPERS / STYLES
# =============================================================================

STYLE_PAGE = {
    "backgroundColor": COL_BG, "color": COL_TEXT,
    "fontFamily": "system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif",
    "minHeight": "100vh", "padding": "0 16px 24px 16px",
}
STYLE_PANEL = {
    "backgroundColor": COL_PANEL, "border": f"1px solid {COL_BORDER}",
    "borderRadius": "8px", "padding": "14px", "marginBottom": "14px",
}
STYLE_PANEL_TIGHT = {**STYLE_PANEL, "padding": "10px 14px"}
STYLE_LABEL = {"color": COL_MUTED, "fontSize": "12px", "marginBottom": "4px",
               "textTransform": "uppercase", "letterSpacing": "0.5px"}
STYLE_VALUE = {"color": COL_TEXT, "fontSize": "20px", "fontWeight": 600}
STYLE_PILL_BASE = {"padding": "4px 10px", "borderRadius": "999px",
                   "fontSize": "12px", "fontWeight": 600, "display": "inline-block"}
STYLE_BTN = {"border": "none", "padding": "8px 14px", "borderRadius": "6px",
             "cursor": "pointer", "fontWeight": 600}


def make_pill(text: str, bg: str, fg: str = COL_BG,
              extra_style: dict | None = None) -> html.Span:
    style = {**STYLE_PILL_BASE, "backgroundColor": bg, "color": fg}
    if extra_style:
        style.update(extra_style)
    return html.Span(text, style=style)


def loss_color(loss_pct: float) -> str:
    if loss_pct < 1.0:
        return COL_OK
    if loss_pct < 5.0:
        return COL_WARN
    return COL_ERR


# =============================================================================
# SECTION 11 — APP (instâncias globais)
# =============================================================================

CSV_DIR.mkdir(parents=True, exist_ok=True)

ema_filter = EMAFilter(FILTERS_FILE)
alert_engine = AlertEngine(ALERTS_FILE)
data_store = DataStore()
graph_tracker = GraphStateTracker()
ws_mgr = WebSocketManager(data_store, ema_filter, alert_engine)
playback_mgr = PlaybackManager(data_store, ema_filter, alert_engine)

ws_mgr.start()  # inicia em modo live (conecta no ESP)

app = Dash(__name__, title="Telemetria ESP32",
           suppress_callback_exceptions=True, update_title=None)

app.index_string = """
<!DOCTYPE html>
<html>
<head>
    {%metas%}
    <title>{%title%}</title>
    {%favicon%}
    {%css%}
    <style>
        body { background-color: """ + COL_BG + """; margin: 0; }
        @keyframes blink { 0%,100%{opacity:1.0;} 50%{opacity:0.35;} }
        .blink-alert { animation: blink 0.8s infinite; }
        .tab-content { padding-top: 12px; }
        .dash-table-container .dash-spreadsheet-container .dash-spreadsheet-inner table {
            background-color: """ + COL_PANEL + """ !important;
        }
        .dash-table-container .dash-header { background-color: """ + COL_PANEL2 + """ !important; }
    </style>
</head>
<body>{%app_entry%}<footer>{%config%}{%scripts%}{%renderer%}</footer></body>
</html>
"""


# =============================================================================
# SECTION 12 — LAYOUT
# =============================================================================

def build_led_controls() -> html.Div:
    """Botões de controle dos LEDs do piloto (comando bidirecional)."""
    return html.Div(style=STYLE_PANEL, children=[
        html.Div("Comandos do piloto (LEDs)", style={**STYLE_LABEL, "fontSize": "13px"}),
        html.Div([
            html.Button("🟦 Luz do PIT: —", id="btn-pit", n_clicks=0,
                        style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                               "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
            html.Button("🚩 Bandeira: —", id="btn-bandeira", n_clicks=0,
                        style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                               "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
            html.Span(id="led-feedback",
                      style={"marginLeft": "8px", "color": COL_MUTED, "fontSize": "12px"}),
        ], style={"display": "flex", "gap": "10px", "alignItems": "center",
                  "flexWrap": "wrap"}),
        # Estado-alvo (target) dos LEDs; None = sem pendência (convergido).
        dcc.Store(id="led-state", data={"pit": None, "bandeira": None}),
    ])


def build_indicators() -> html.Div:
    """Indicadores grandes: velocidade + acelerador + temp + digitais."""
    return html.Div(style={
        "display": "grid",
        "gridTemplateColumns": "repeat(auto-fit, minmax(150px, 1fr))",
        "gap": "10px", "marginBottom": "14px",
    }, children=[
        html.Div(style={**STYLE_PANEL_TIGHT, "textAlign": "center"}, children=[
            html.Div("Velocidade", style=STYLE_LABEL),
            html.Div([html.Span(id="ind-vel", children="—",
                                style={"fontSize": "34px", "fontWeight": 700,
                                       "color": COL_ACCENT}),
                      html.Span(" km/h", style={"color": COL_MUTED, "fontSize": "13px"})]),
        ]),
        html.Div(style={**STYLE_PANEL_TIGHT, "textAlign": "center"}, children=[
            html.Div("Acelerador", style=STYLE_LABEL),
            html.Div(id="ind-acel", children="—", style=STYLE_VALUE),
        ]),
        html.Div(style={**STYLE_PANEL_TIGHT, "textAlign": "center"}, children=[
            html.Div("Temp. Motor", style=STYLE_LABEL),
            html.Div(id="ind-temp", children="—", style=STYLE_VALUE),
        ]),
        html.Div(id="ind-emergencia-box", style={**STYLE_PANEL_TIGHT,
                                                  "textAlign": "center"}, children=[
            html.Div("Emergência", style=STYLE_LABEL),
            html.Div(id="ind-emergencia", children="—", style=STYLE_VALUE),
        ]),
        html.Div(id="ind-freio-box", style={**STYLE_PANEL_TIGHT,
                                            "textAlign": "center"}, children=[
            html.Div("Freio", style=STYLE_LABEL),
            html.Div(id="ind-freio", children="—", style=STYLE_VALUE),
        ]),
    ])


def build_graphs_grid() -> html.Div:
    """Grade com um gráfico por variável + gráfico de digitais."""
    graphs = []
    for var in VAR_KEYS:
        graphs.append(html.Div(style={**STYLE_PANEL, "marginBottom": 0}, children=[
            dcc.Graph(id=f"graph-{var}", figure=make_single_figure(var),
                      config={"displaylogo": False, "displayModeBar": False}),
        ]))
    graphs.append(html.Div(style={**STYLE_PANEL, "marginBottom": 0}, children=[
        dcc.Graph(id="graph-digitais", figure=make_digital_figure(),
                  config={"displaylogo": False, "displayModeBar": False}),
    ]))
    return html.Div(style={
        "display": "grid",
        "gridTemplateColumns": "repeat(auto-fit, minmax(330px, 1fr))",
        "gap": "12px",
    }, children=graphs)


def build_source_panel() -> html.Div:
    return html.Div(style=STYLE_PANEL, children=[
        html.Div([
            html.Div([
                html.Div("Fonte de dados", style=STYLE_LABEL),
                dcc.RadioItems(
                    id="src-radio",
                    options=[
                        {"label": " Live (WebSocket)", "value": "live"},
                        {"label": " Playback (CSV)", "value": "playback"},
                    ],
                    value="live", inline=True, style={"color": COL_TEXT},
                    inputStyle={"marginRight": "6px", "marginLeft": "12px"},
                ),
            ], style={"flex": "1", "minWidth": "240px"}),
            html.Div([
                html.Div("ESP32 (host/IP)", style=STYLE_LABEL),
                html.Div([
                    dcc.Input(id="esp-host", type="text", value=ESP32_HOST_DEFAULT,
                              debounce=True,
                              style={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                                     "border": f"1px solid {COL_BORDER}",
                                     "borderRadius": "6px", "padding": "6px 10px",
                                     "width": "150px"}),
                    html.Button("Conectar", id="esp-connect", n_clicks=0,
                                style={**STYLE_BTN, "backgroundColor": COL_ACCENT,
                                       "color": COL_BG, "marginLeft": "8px"}),
                ], style={"display": "flex", "alignItems": "center"}),
            ], style={"flex": "1", "minWidth": "240px"}),
        ], style={"display": "flex", "gap": "16px", "flexWrap": "wrap",
                  "alignItems": "flex-start"}),

        html.Div(id="live-logging-panel", style={"marginTop": "12px"}, children=[
            html.Div("Gravação CSV", style={**STYLE_LABEL, "fontSize": "13px"}),
            html.Div([
                html.Button("⏺ Iniciar", id="log-start", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_ERR, "color": COL_TEXT}),
                html.Button("⏸ Pausar", id="log-pause", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_WARN, "color": COL_BG}),
                html.Button("⏹ Parar", id="log-stop", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                                   "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
                html.Span(id="log-status", children="Log não iniciado.",
                          style={"marginLeft": "4px", "color": COL_MUTED, "fontSize": "13px"}),
            ], style={"display": "flex", "gap": "8px", "alignItems": "center",
                      "flexWrap": "wrap"}),
        ]),

        html.Div(id="playback-panel", style={"marginTop": "12px", "display": "none"},
                 children=[
            html.Div("Playback", style={**STYLE_LABEL, "fontSize": "13px"}),
            html.Div([
                dcc.Dropdown(id="pb-log-dropdown", options=[],
                             placeholder="Selecione um CSV da pasta logs/...",
                             style={"flex": "1", "color": "#000", "minWidth": "260px"}),
                html.Button("🔄", id="pb-refresh-logs", n_clicks=0, title="Atualizar lista",
                            style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                                   "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
            ], style={"display": "flex", "gap": "8px", "marginBottom": "8px"}),
            dcc.Input(id="playback-path", type="text",
                      placeholder="...ou caminho manual do CSV",
                      style={"width": "100%", "backgroundColor": COL_PANEL2,
                             "color": COL_TEXT, "border": f"1px solid {COL_BORDER}",
                             "borderRadius": "6px", "padding": "6px 10px",
                             "marginBottom": "8px"}),
            html.Div([
                html.Button("▶ Play", id="pb-play", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_OK, "color": COL_BG}),
                html.Button("⏸ Pause", id="pb-pause", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_WARN, "color": COL_BG}),
                html.Button("⏹ Stop", id="pb-stop", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_ERR, "color": COL_TEXT}),
                dcc.Dropdown(id="pb-speed",
                             options=[{"label": f"{s}x", "value": s}
                                      for s in (0.5, 1, 2, 5, 10)],
                             value=1, clearable=False,
                             style={"width": "100px", "color": "#000"}),
            ], style={"display": "flex", "gap": "8px", "alignItems": "center",
                      "flexWrap": "wrap"}),
            html.Div(id="playback-status", style={"marginTop": "8px",
                                                  "color": COL_MUTED, "fontSize": "13px"}),
        ]),
    ])


def build_dashboard_tab() -> html.Div:
    return html.Div([
        build_source_panel(),
        build_led_controls(),
        build_indicators(),
        build_graphs_grid(),
    ])


def build_config_tab() -> html.Div:
    return html.Div([
        html.Div(style=STYLE_PANEL, children=[
            html.H3("Filtro EMA (suavização do gráfico)",
                    style={"marginTop": 0, "color": COL_TEXT}),
            html.Div([
                "O firmware já filtra na origem (média móvel + anti-bounce). "
                "Este EMA é um polimento extra no PC: ",
                html.Code("y[n] = a*x[n] + (1-a)*y[n-1]", style={"color": COL_ACCENT}),
                ". ", html.B("a=1.0", style={"color": COL_TEXT}), " desliga (passthrough).",
            ], style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "10px"}),
            dash_table.DataTable(
                id="ema-table",
                columns=[
                    {"name": "Variável", "id": "variavel", "editable": False},
                    {"name": "Descrição", "id": "label", "editable": False},
                    {"name": "Alpha (0.001 .. 1.0)", "id": "alpha",
                     "type": "numeric", "editable": True},
                ],
                data=ema_filter.all_as_table(),
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
                style_data_conditional=[{"if": {"column_editable": True},
                                         "backgroundColor": COL_PANEL2}],
            ),
            html.Button("💾 Salvar filtros", id="ema-save", n_clicks=0,
                        style={**STYLE_BTN, "backgroundColor": COL_ACCENT,
                               "color": COL_BG, "marginTop": "10px"}),
            html.Span(id="ema-save-feedback",
                      style={"marginLeft": "12px", "color": COL_MUTED}),
        ]),
    ])


def build_alerts_tab() -> html.Div:
    return html.Div([
        html.Div(style=STYLE_PANEL, children=[
            html.H3("Regras de alerta", style={"marginTop": 0, "color": COL_TEXT}),
            html.Div([
                "Operadores: ", html.Code(" >  <  >=  <=  == ", style={"color": COL_ACCENT}),
                ". Avaliação sobre o valor filtrado, na borda de subida.",
            ], style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "10px"}),
            dash_table.DataTable(
                id="alerts-table",
                columns=[
                    {"name": "ID", "id": "id", "editable": False},
                    {"name": "Variável", "id": "variable",
                     "presentation": "dropdown", "editable": True},
                    {"name": "Operador", "id": "operator",
                     "presentation": "dropdown", "editable": True},
                    {"name": "Threshold", "id": "threshold", "type": "numeric",
                     "editable": True},
                    {"name": "Mensagem", "id": "message", "editable": True},
                    {"name": "Ativo (sim/não)", "id": "enabled", "editable": True},
                ],
                data=alert_engine.all_as_table(),
                dropdown={
                    "variable": {"options": [{"label": TELEMETRY_VARIABLES[k]["label"],
                                              "value": k} for k in VAR_KEYS]},
                    "operator": {"options": [{"label": o, "value": o}
                                             for o in ALERT_OPERATORS]},
                },
                editable=True, row_deletable=True,
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
                style_data_conditional=[{"if": {"column_editable": True},
                                         "backgroundColor": COL_PANEL2}],
            ),
            html.Div([
                html.Button("➕ Nova regra", id="alerts-add", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                                   "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
                html.Button("💾 Salvar", id="alerts-save", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_ACCENT, "color": COL_BG}),
                html.Button("🧹 Limpar log", id="alerts-clearlog", n_clicks=0,
                            style={**STYLE_BTN, "backgroundColor": COL_PANEL2,
                                   "color": COL_TEXT, "border": f"1px solid {COL_BORDER}"}),
                html.Span(id="alerts-save-feedback",
                          style={"marginLeft": "8px", "color": COL_MUTED}),
            ], style={"display": "flex", "gap": "8px", "marginTop": "10px",
                      "flexWrap": "wrap"}),
        ]),
        html.Div(style=STYLE_PANEL, children=[
            html.H3("Histórico de alertas", style={"marginTop": 0, "color": COL_TEXT}),
            dash_table.DataTable(
                id="alerts-log",
                columns=[
                    {"name": "Timestamp", "id": "timestamp"},
                    {"name": "Variável", "id": "variable"},
                    {"name": "Condição", "id": "condicao"},
                    {"name": "Valor", "id": "valor"},
                    {"name": "Mensagem", "id": "mensagem"},
                ],
                data=[], page_size=12,
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "6px 8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
            ),
        ]),
    ])


def build_status_bar() -> html.Div:
    return html.Div(id="status-bar", style={
        **STYLE_PANEL_TIGHT, "display": "flex", "gap": "12px",
        "alignItems": "center", "flexWrap": "wrap", "marginBottom": "10px",
    })


app.layout = html.Div(style=STYLE_PAGE, children=[
    html.Div([
        html.H2("🏎 Telemetria ESP32",
                style={"margin": "12px 0 4px 0", "color": COL_TEXT}),
        html.Div("ESP32 (WebSocket assíncrono) → Dashboard",
                 style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "8px"}),
    ]),
    build_status_bar(),

    dcc.Tabs(id="tabs", value="dashboard", colors={
        "border": COL_BORDER, "primary": COL_ACCENT, "background": COL_PANEL,
    }, children=[
        dcc.Tab(label="Dashboard", value="dashboard",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
        dcc.Tab(label="Configuração", value="config",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
        dcc.Tab(label="Alertas", value="alertas",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
    ]),

    html.Div(id="tab-dashboard", className="tab-content", children=build_dashboard_tab()),
    html.Div(id="tab-config", className="tab-content",
             style={"display": "none"}, children=build_config_tab()),
    html.Div(id="tab-alertas", className="tab-content",
             style={"display": "none"}, children=build_alerts_tab()),

    dcc.Interval(id="tick", interval=UI_TICK_MS, n_intervals=0),
    dcc.Store(id="graph-reset-signal", data=0),
])


# =============================================================================
# SECTION 13 — CALLBACKS
# =============================================================================

# ---- Navegação entre abas --------------------------------------------------

@app.callback(
    Output("tab-dashboard", "style"),
    Output("tab-config", "style"),
    Output("tab-alertas", "style"),
    Input("tabs", "value"),
)
def cb_switch_tabs(tab):
    show = {"display": "block"}
    hide = {"display": "none"}
    return (show if tab == "dashboard" else hide,
            show if tab == "config" else hide,
            show if tab == "alertas" else hide)


# ---- Visibilidade dos painéis por modo ------------------------------------

@app.callback(
    Output("playback-panel", "style"),
    Output("live-logging-panel", "style"),
    Input("src-radio", "value"),
)
def cb_panels_visibility(src):
    vis = {"marginTop": "12px"}
    hid = {"marginTop": "12px", "display": "none"}
    if src == "playback":
        return vis, hid
    return hid, vis


# ---- Conexão com o ESP (troca de host) ------------------------------------

@app.callback(
    Output("status-bar", "children", allow_duplicate=True),
    Input("esp-connect", "n_clicks"),
    Input("esp-host", "value"),
    prevent_initial_call=True,
)
def cb_connect_esp(_n, host):
    ws_mgr.set_host(host)
    return no_update


# ---- Source switch (live <-> playback) -------------------------------------

@app.callback(
    Output("playback-status", "children"),
    Output("graph-reset-signal", "data", allow_duplicate=True),
    Input("src-radio", "value"),
    State("graph-reset-signal", "data"),
    prevent_initial_call=True,
)
def cb_source_switch(src, reset_gen):
    gen = (reset_gen or 0) + 1
    if src == "live":
        if playback_mgr.is_running():
            playback_mgr.stop()
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset_all()
        ws_mgr.start()
        return "Modo Live ativo. Conectando ao ESP32...", gen
    else:
        ws_mgr.stop()
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset_all()
        return "Modo Playback ativo. Selecione um CSV e clique em Play.", gen


# ---- Controles de playback -------------------------------------------------

@app.callback(
    Output("playback-status", "children", allow_duplicate=True),
    Output("graph-reset-signal", "data", allow_duplicate=True),
    Input("pb-play", "n_clicks"),
    Input("pb-pause", "n_clicks"),
    Input("pb-stop", "n_clicks"),
    Input("pb-speed", "value"),
    State("playback-path", "value"),
    State("graph-reset-signal", "data"),
    prevent_initial_call=True,
)
def cb_playback_controls(_p, _pa, _s, speed, path, reset_gen):
    triggered = ctx.triggered_id
    gen = (reset_gen or 0) + 1
    if triggered == "pb-speed":
        playback_mgr.set_speed(float(speed or 1))
        return f"Velocidade ajustada para {speed}x.", no_update
    if triggered == "pb-play":
        if not path:
            return "⚠ Informe ou selecione o CSV antes de tocar.", no_update
        p = Path(path).expanduser()
        if not p.is_absolute():
            p = BASE_DIR / p
        if not p.exists():
            return f"⚠ Arquivo não encontrado: {p}", no_update
        if playback_mgr.is_paused():
            playback_mgr.resume()
            return f"▶ Retomando: {p.name}", no_update
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset_all()
        ok = playback_mgr.start(p, float(speed or 1))
        return (f"▶ Reproduzindo {p.name}" if ok else "⚠ Falha ao iniciar playback."), gen
    if triggered == "pb-pause":
        if playback_mgr.is_running() and not playback_mgr.is_paused():
            playback_mgr.pause()
            return "⏸ Pausado.", no_update
        return no_update, no_update
    if triggered == "pb-stop":
        playback_mgr.stop()
        return "⏹ Parado.", no_update
    return no_update, no_update


# ---- Lista de logs disponíveis (dropdown playback) -------------------------

@app.callback(
    Output("pb-log-dropdown", "options"),
    Input("pb-refresh-logs", "n_clicks"),
    Input("src-radio", "value"),
)
def cb_refresh_log_list(_n, _src):
    if not CSV_DIR.exists():
        return []
    files = sorted(CSV_DIR.glob("*.csv"), key=lambda f: f.stat().st_mtime, reverse=True)
    return [{"label": f.name, "value": str(f)} for f in files]


@app.callback(
    Output("playback-path", "value"),
    Input("pb-log-dropdown", "value"),
    prevent_initial_call=True,
)
def cb_log_dropdown_to_path(selected):
    return selected if selected else no_update


# ---- Atualização dos gráficos (um callback por gráfico) --------------------
#
# Dash 2.16+ computa um fingerprint hash para callbacks multi-output e o envia
# no request do browser; o servidor, ao registrar sem allow_duplicate, armazena
# a chave SEM esse hash → KeyError. A solução é um callback por gráfico: sem
# multi-output, sem fingerprint, sem mismatch.

def _make_graph_cb(var):
    @app.callback(
        Output(f"graph-{var}", "figure"),
        Input("tick", "n_intervals"),
        Input("graph-reset-signal", "data"),
    )
    def _cb(_n, _reset, _var=var):
        forcar = ctx.triggered_id == "graph-reset-signal"
        gid = f"graph-{_var}"
        samples = data_store.get_recent_samples(_var, MAX_GRAPH_POINTS)
        last_ts = samples[-1][0] if samples else None
        if not forcar and last_ts == graph_tracker.get(gid):
            return no_update
        graph_tracker.set(gid, last_ts)
        return make_single_figure(_var, samples)

for _var in VAR_KEYS:
    _make_graph_cb(_var)


@app.callback(
    Output("graph-digitais", "figure"),
    Input("tick", "n_intervals"),
    Input("graph-reset-signal", "data"),
)
def cb_update_graph_digitais(_n, _reset):
    forcar = ctx.triggered_id == "graph-reset-signal"
    gid = "graph-digitais"
    series = data_store.get_recent_digital(MAX_GRAPH_POINTS)
    last_ts = None
    for k in DIGITAL_KEYS:
        if series[k]:
            ts_k = series[k][-1][0]
            last_ts = ts_k if last_ts is None else max(last_ts, ts_k)
    if not forcar and last_ts == graph_tracker.get(gid):
        return no_update
    graph_tracker.set(gid, last_ts)
    return make_digital_figure(series)


# ---- Tick: indicadores numéricos / digitais --------------------------------

@app.callback(
    Output("ind-vel", "children"),
    Output("ind-acel", "children"),
    Output("ind-temp", "children"),
    Output("ind-emergencia", "children"),
    Output("ind-freio", "children"),
    Output("ind-emergencia-box", "style"),
    Output("ind-freio-box", "style"),
    Input("tick", "n_intervals"),
)
def cb_indicators(_n):
    stats = data_store.snapshot_stats()
    last = stats.get("last_packet") or {}

    def fmt(key, suffix="", filt=True):
        v = last.get(f"{key}_filt") if filt else None
        if v is None:
            v = last.get(key)
        if v is None:
            return "—"
        try:
            return f"{float(v):.0f}{suffix}"
        except (TypeError, ValueError):
            return "—"

    vel = fmt("vel")
    acel = fmt("acel", " %")
    temp = fmt("temp", " °C")

    emer_on = int(last.get("emergencia", 0) or 0) != 0
    freio_on = int(last.get("freio", 0) or 0) != 0

    box_base = {**STYLE_PANEL_TIGHT, "textAlign": "center"}
    emer_box = {**box_base, "backgroundColor": COL_ERR} if emer_on else box_base
    freio_box = {**box_base, "backgroundColor": COL_WARN} if freio_on else box_base

    emer_txt = "ATIVA" if emer_on else "ok"
    freio_txt = "ACIONADO" if freio_on else "solto"

    return vel, acel, temp, emer_txt, freio_txt, emer_box, freio_box


# ---- LEDs do piloto: padrão Target (alvo) vs Reported (telemetria) ---------
#
# O envio direto de comandos é frágil: se o socket oscila no instante do clique,
# o comando se perde e a interface "desiste". Em vez disso, separamos a INTENÇÃO
# (o que o operador quer) do FATO (o que o hardware reporta):
#
#   - cb_led_target: o clique apenas registra o estado-alvo desejado no Store
#     `led-state` (não envia nada).
#   - cb_led_reconcile: a cada tick compara alvo x reportado. Enquanto houver
#     divergência, reenvia o comando (martela); quando convergem, zera o alvo.
#
# Assim um comando perdido é reenviado no próximo ciclo até o ESP confirmar, e a
# cor do botão indica a fase: verde=ON confirmado, cinza=OFF confirmado,
# amarelo=pendente (alvo != reportado, comando em progresso).

def _reported_led(last, led):
    """Estado reportado (0/1) de um LED na última telemetria, ou None se ausente."""
    v = last.get(led)
    if v is None:
        return None
    try:
        return 1 if int(v) != 0 else 0
    except (TypeError, ValueError):
        return None


@app.callback(
    Output("led-state", "data"),
    Input("btn-pit", "n_clicks"),
    Input("btn-bandeira", "n_clicks"),
    State("led-state", "data"),
    prevent_initial_call=True,
)
def cb_led_target(_p, _b, target):
    """Atualização de Alvo: o clique inverte o estado desejado e o salva no Store.

    A inversão é relativa ao alvo pendente (se houver) ou, na ausência dele, ao
    estado atualmente reportado pelo hardware — assim cliques repetidos antes da
    confirmação alternam o alvo de forma previsível.
    """
    triggered = ctx.triggered_id
    target = dict(target or {})
    last = data_store.snapshot_stats().get("last_packet") or {}

    led = "pit" if triggered == "btn-pit" else \
          "bandeira" if triggered == "btn-bandeira" else None
    if led is None:
        return no_update

    base = target.get(led)
    if base is None:                      # sem alvo pendente: parte do reportado
        base = _reported_led(last, led)
        if base is None:                  # sem telemetria ainda: assume OFF
            base = 0
    target[led] = 1 - int(base)           # inverte e registra a intenção
    return target


@app.callback(
    Output("led-state", "data", allow_duplicate=True),
    Output("btn-pit", "children"),
    Output("btn-pit", "style"),
    Output("btn-bandeira", "children"),
    Output("btn-bandeira", "style"),
    Output("led-feedback", "children"),
    Input("tick", "n_intervals"),
    State("led-state", "data"),
    prevent_initial_call=True,
)
def cb_led_reconcile(_n, target):
    """Reconciliação: aproxima o estado reportado do alvo, martelando comandos.

    Para cada LED com alvo definido: se diverge do reportado, reenvia o comando;
    se converge, limpa o alvo (None). A aparência do botão reflete a fase atual.
    """
    target = dict(target or {})
    last = data_store.snapshot_stats().get("last_packet") or {}

    pendentes = []
    store_changed = False

    for led in OUTPUT_KEYS:
        alvo = target.get(led)
        if alvo is None:
            continue                       # nada a fazer para este LED
        alvo = int(alvo)
        reported = _reported_led(last, led)
        if reported == alvo:
            target[led] = None             # convergiu -> limpa o alvo
            store_changed = True
        else:
            # diverge (ou ainda sem telemetria): martela o comando
            ws_mgr.send_command(**{led: alvo})
            pendentes.append(OUTPUT_LABELS[led])

    # ---- aparência dos botões (verde/cinza confirmado, amarelo pendente) ----
    def render(led, emoji, label):
        alvo = target.get(led)
        reported = _reported_led(last, led)
        if alvo is not None:               # pendente: alvo ainda não confirmado
            destino = "ON" if int(alvo) else "OFF"
            style = {**STYLE_BTN, "backgroundColor": COL_WARN, "color": COL_BG}
            return f"{emoji} {label} → {destino}", style
        # convergido: cor pelo estado reportado
        on = (reported == 1)
        estado = "ON" if on else ("OFF" if reported == 0 else "—")
        if on:
            style = {**STYLE_BTN, "backgroundColor": COL_OK, "color": COL_BG}
        else:
            style = {**STYLE_BTN, "backgroundColor": COL_PANEL2, "color": COL_TEXT,
                     "border": f"1px solid {COL_BORDER}"}
        return f"{emoji} {label}: {estado}", style

    pit_txt, pit_style = render("pit", "🟦", "Luz do PIT")
    band_txt, band_style = render("bandeira", "🚩", "Bandeira")

    if pendentes:
        feedback = "↻ enviando: " + ", ".join(pendentes)
    elif any(_reported_led(last, k) is not None for k in OUTPUT_KEYS):
        feedback = "● sincronizado"
    else:
        feedback = ""

    store_out = target if store_changed else no_update
    return store_out, pit_txt, pit_style, band_txt, band_style, feedback


# ---- Barra de status -------------------------------------------------------

@app.callback(
    Output("status-bar", "children"),
    Input("tick", "n_intervals"),
    State("src-radio", "value"),
)
def cb_status_bar(_n, src):
    stats = data_store.snapshot_stats()

    if src == "live":
        if ws_mgr.is_connected():
            conn_pill = make_pill(f"● CONECTADO {ws_mgr.host()}", COL_OK)
        else:
            conn_pill = make_pill(f"○ CONECTANDO {ws_mgr.host()}", COL_WARN)
        csv_pill = make_pill(
            f"📄 {ws_mgr.csv_path().name}" if ws_mgr.csv_path() else "📄 —",
            COL_PANEL2, fg=COL_TEXT)
    else:
        running = playback_mgr.is_running()
        paused = playback_mgr.is_paused()
        if running and not paused:
            conn_pill = make_pill("▶ PLAYBACK", COL_ACCENT)
        elif paused:
            conn_pill = make_pill("⏸ PAUSADO", COL_WARN)
        else:
            conn_pill = make_pill("⏹ STOPPED", COL_PANEL2, fg=COL_TEXT,
                                  extra_style={"border": f"1px solid {COL_BORDER}"})
        prog = playback_mgr.progress() * 100.0
        csv_pill = make_pill(
            f"📄 {playback_mgr.csv_path().name} ({prog:.0f}%)"
            if playback_mgr.csv_path() else "📄 —",
            COL_PANEL2, fg=COL_TEXT)

    loss_pct = stats.get("loss_pct", 0.0)
    loss_pill = make_pill(f"Perda: {loss_pct:.2f}%", loss_color(loss_pct))
    rx_pill = make_pill(f"RX: {stats.get('total_rx', 0)}", COL_PANEL2, fg=COL_TEXT,
                        extra_style={"border": f"1px solid {COL_BORDER}"})
    rst_pill = make_pill(f"Resets: {stats.get('resets', 0)}", COL_PANEL2, fg=COL_TEXT,
                         extra_style={"border": f"1px solid {COL_BORDER}"})

    n_active = alert_engine.active_count()
    if n_active > 0:
        alert_pill = html.Span(f"🔔 {n_active} alerta(s)", className="blink-alert",
                               style={**STYLE_PILL_BASE, "backgroundColor": COL_ERR,
                                      "color": COL_TEXT})
    else:
        alert_pill = make_pill("Sem alertas", COL_OK)

    return [conn_pill, csv_pill, rx_pill, loss_pill, rst_pill, alert_pill]


# ---- Logging live ----------------------------------------------------------

@app.callback(
    Output("log-status", "children"),
    Input("log-start", "n_clicks"),
    Input("log-pause", "n_clicks"),
    Input("log-stop", "n_clicks"),
    prevent_initial_call=True,
)
def cb_logging_controls(_s, _p, _st):
    triggered = ctx.triggered_id
    if triggered == "log-start":
        if ws_mgr.is_logging_paused():
            ws_mgr.resume_logging()
            name = ws_mgr.csv_path().name if ws_mgr.csv_path() else "—"
            return f"⏺ Gravando: {name}"
        name = ws_mgr.start_logging()
        return f"⏺ Gravando: {name}" if name else "⏺ Aguardando conexão para gravar..."
    if triggered == "log-pause":
        ws_mgr.pause_logging()
        return "⏸ Log pausado."
    if triggered == "log-stop":
        ws_mgr.stop_logging()
        return "⏹ Log parado."
    return no_update


# ---- EMA: salvar -----------------------------------------------------------

@app.callback(
    Output("ema-save-feedback", "children"),
    Input("ema-save", "n_clicks"),
    State("ema-table", "data"),
    prevent_initial_call=True,
)
def cb_save_ema(_n, rows):
    ema_filter.update_from_table(rows or [])
    return f"✓ Salvo em {datetime.now().strftime('%H:%M:%S')}"


# ---- Alertas: adicionar/salvar/limpar log ----------------------------------

@app.callback(
    Output("alerts-table", "data"),
    Output("alerts-save-feedback", "children"),
    Input("alerts-add", "n_clicks"),
    Input("alerts-save", "n_clicks"),
    Input("alerts-clearlog", "n_clicks"),
    State("alerts-table", "data"),
    prevent_initial_call=True,
)
def cb_alerts_actions(_a, _s, _c, rows):
    triggered = ctx.triggered_id
    rows = rows or []
    if triggered == "alerts-add":
        rows = list(rows) + [{
            "id": uuid.uuid4().hex[:8], "variable": VAR_KEYS[0], "operator": ">",
            "threshold": 0.0, "message": "Nova regra", "enabled": "sim",
        }]
        return rows, "Nova linha — clique em Salvar para persistir."
    if triggered == "alerts-save":
        alert_engine.update_from_table(rows)
        return alert_engine.all_as_table(), f"✓ Salvo em {datetime.now().strftime('%H:%M:%S')}"
    if triggered == "alerts-clearlog":
        alert_engine.clear_log()
        return no_update, f"Log limpo em {datetime.now().strftime('%H:%M:%S')}"
    return no_update, no_update


@app.callback(
    Output("alerts-log", "data"),
    Input("tick", "n_intervals"),
)
def cb_alerts_log(_n):
    return alert_engine.events_as_table()


# =============================================================================
# SECTION 14 — ENTRY POINT
# =============================================================================

def main():
    log.info("=" * 60)
    log.info("Dashboard de Telemetria ESP32")
    log.info("=" * 60)
    log.info("Base dir:  %s", BASE_DIR)
    log.info("CSV dir:   %s", CSV_DIR)
    log.info("ESP32 WS:  %s", ws_mgr.url())
    log.info("Dashboard em http://127.0.0.1:8050")
    log.info("=" * 60)
    # debug=False: o reloader do Flask criaria 2 processos e duplicaria a
    # thread do WebSocket, gerando conexões concorrentes ao ESP.
    app.run(debug=False, host="127.0.0.1", port=8050)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Interrompido pelo usuário. Encerrando...")
    finally:
        ws_mgr.stop()
        playback_mgr.stop()
        log.info("Encerrado.")
