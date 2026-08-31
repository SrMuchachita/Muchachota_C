#!/usr/bin/env python3
"""
RD90 Console - Monitor BLE (GATT)
==================================================================

GUI para conectarse por BLE a la consola RD90 (WP_C_V9_UP, DEVICE_NAME=
"RD90Pro") y ver en vivo los datos que expone por su servicio GATT:
tilt (pitch/roll), bateria (consola + robot), temperatura, encoder, eventos
de "Captura" y version de firmware/protocolo.

Replica exactamente lo que main.c define en:
  - robotSvcUuid / tiltChrUuid / battChrUuid / tempChrUuid / encChrUuid /
    captureChrUuid / fwVerChrUuid (BLE_UUID128_INIT, main.c ~linea 397-430)
  - Los payloads de cada característica (structs empaquetados sin relleno,
    little-endian, tal cual los arma cada *ChrAccess() con os_mbuf_append):

    c001 tilt     : 2x int16 LE  -> [pitch_raw, roll_raw]      (appTilt)
    c002 battery  : 2x int16 LE  -> [consola_mV, robot_raw]    (appBattery)
    c003 temp     : 1x int16 LE  -> temp_raw                   (appTemp)
    c004 encoder  : 1x int32 LE  -> conteo                     (appEncoder)
    c005 capture  : 1x uint8     -> contador de pulsaciones    (appCaptureEvt)
    c006 fwver    : 4x uint8     -> [proto_ver, major, minor, patch] (solo lectura)

NOTA IMPORTANTE sobre pitch/roll/temp: el robot los manda por Modbus en
grados x100 (ver TABLAS_MODBUS.md), y hoy la consola los reenvia por BLE
SIN dividir por 100 (bug ya identificado, pendiente de corregir). Por eso
esta GUI muestra el valor crudo Y ademas una columna "/100" con la posible
lectura real, para no tener que adivinar mientras el bug siga sin resolver.

Requisitos:
    pip install bleak
"""

import asyncio
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    raise SystemExit(
        "Falta bleak. Instala con: pip install bleak"
    )

# ============================================================
# UUIDs (128-bit) - mismos que main.c, pero en formato string
# estandar (big-endian/textual). NimBLE's BLE_UUID128_INIT toma
# los bytes en orden "de aire" (little-endian); para el string
# textual convencional hay que invertir el arreglo completo.
# Por eso ademas de matchear por UUID completo, tambien se
# matchea por sufijo (...c00N) como red de seguridad.
# ============================================================
DEVICE_NAME_FILTER = "RD90"  # cualquier device cuyo nombre CONTENGA esto se resalta

SERVICE_UUID = "4cc885f6-f3d0-45fe-add5-2d29e1310e01"

CHAR_DEFS = {
    "tilt":    {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c001", "suffix": "c001", "label": "Tilt (pitch/roll)"},
    "battery": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c002", "suffix": "c002", "label": "Bateria"},
    "temp":    {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c003", "suffix": "c003", "label": "Temperatura"},
    "encoder": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c004", "suffix": "c004", "label": "Encoder"},
    "capture": {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c005", "suffix": "c005", "label": "Captura (evento)"},
    "fwver":   {"uuid": "0000bcea-5f78-2315-deef-1212a1f1c006", "suffix": "c006", "label": "Version FW/protocolo"},
}


def decode_payload(key: str, data: bytes) -> str:
    """Decodifica el payload crudo de cada característica al formato que
    arma main.c. Si el tamaño no coincide con lo esperado, se avisa en vez
    de fallar en silencio (asi se nota enseguida si el firmware cambio)."""
    try:
        if key == "tilt":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)}, se esperaban 4 bytes)"
            pitch_raw, roll_raw = struct.unpack("<hh", data)
            return (f"pitch_raw={pitch_raw:6d}  roll_raw={roll_raw:6d}   "
                    f"(/100 → pitch={pitch_raw/100:.2f}°  roll={roll_raw/100:.2f}°)")

        if key == "battery":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)}, se esperaban 4 bytes)"
            console_mv, robot_raw = struct.unpack("<hh", data)
            return f"consola={console_mv} mV   robot={robot_raw} (raw)"

        if key == "temp":
            if len(data) != 2:
                return f"(tamaño inesperado {len(data)}, se esperaban 2 bytes)"
            (temp_raw,) = struct.unpack("<h", data)
            return f"temp_raw={temp_raw}   (/100 → {temp_raw/100:.2f} °C)"

        if key == "encoder":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)}, se esperaban 4 bytes)"
            (count,) = struct.unpack("<i", data)
            return f"count={count}"

        if key == "capture":
            if len(data) != 1:
                return f"(tamaño inesperado {len(data)}, se esperaba 1 byte)"
            (evt,) = struct.unpack("<B", data)
            return f"contador de pulsaciones={evt}"

        if key == "fwver":
            if len(data) != 4:
                return f"(tamaño inesperado {len(data)}, se esperaban 4 bytes)"
            proto, major, minor, patch = struct.unpack("<BBBB", data)
            return f"protocolo v{proto}   firmware v{major}.{minor}.{patch}"

    except struct.error as e:
        return f"(error decodificando: {e})"

    return "(clave desconocida)"


# ============================================================
# ESTADO COMPARTIDO
# ============================================================
class BleState:
    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.device_name = ""
        self.device_addr = ""
        # valores decodificados por clave
        self.values = {k: "(sin datos aun)" for k in CHAR_DEFS}
        self.raw_hex = {k: "" for k in CHAR_DEFS}
        self.last_update = {k: 0.0 for k in CHAR_DEFS}
        # log de eventos (timestamp, texto)
        self.log_lines = []

    def log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self.lock:
            self.log_lines.append(f"[{ts}] {text}")
            if len(self.log_lines) > 500:
                self.log_lines = self.log_lines[-500:]

    def update_value(self, key: str, data: bytes, quiet: bool = False):
        """quiet=True: actualiza el valor pero solo escribe en el log si cambio
        respecto al anterior (para el polling por READ, que si no inunda el log)."""
        text = decode_payload(key, data)
        with self.lock:
            changed = (self.values[key] != text)
            self.values[key] = text
            self.raw_hex[key] = data.hex(" ")
            self.last_update[key] = time.time()
        if not quiet or changed:
            self.log(f"{CHAR_DEFS[key]['label']}: {text}  |  raw={data.hex(' ')}")


# ============================================================
# HILO ASYNCIO (bleak es asincrono; corre en su propio hilo,
# la GUI de Tkinter se actualiza leyendo el estado compartido)
# ============================================================
class BleWorker(threading.Thread):
    def __init__(self, state: BleState, on_status):
        super().__init__(daemon=True)
        self.state = state
        self.on_status = on_status
        self.loop = None
        self.client = None
        self._stop_flag = threading.Event()
        self._target_address = None
        self._connect_requested = threading.Event()
        self._disconnect_requested = threading.Event()

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main())

    def stop(self):
        self._stop_flag.set()

    def request_connect(self, address: str):
        self._target_address = address
        self._connect_requested.set()

    def request_disconnect(self):
        self._disconnect_requested.set()

    async def _main(self):
        while not self._stop_flag.is_set():
            if self._connect_requested.is_set():
                self._connect_requested.clear()
                await self._connect_and_run(self._target_address)
            await asyncio.sleep(0.1)

    async def _connect_and_run(self, address: str):
        self.on_status(f"🔄 Conectando a {address}...")
        try:
            async with BleakClient(address) as client:
                self.client = client
                self.state.connected = True
                self.on_status(f"✅ Conectado a {address}")
                self.state.log(f"Conectado a {address}")

                # --- Descubrir servicios/caracteristicas ---
                matched = {}  # key -> characteristic uuid string real
                for svc in client.services:
                    for ch in svc.characteristics:
                        ch_uuid = str(ch.uuid).lower()
                        for key, meta in CHAR_DEFS.items():
                            if ch_uuid == meta["uuid"] or ch_uuid.endswith(meta["suffix"]):
                                matched[key] = ch.uuid
                        self.state.log(
                            f"Descubierto: {ch_uuid}  props={list(ch.properties)}  handle={ch.handle}"
                        )

                if not matched:
                    self.state.log("⚠️ No se encontro ninguna caracteristica conocida del servicio RD90 "
                                    "(revisa el log de arriba: puede que los UUIDs cambiaron en el firmware)")

                # --- Leer fwver una vez (read-only, no notify) ---
                if "fwver" in matched:
                    try:
                        data = await client.read_gatt_char(matched["fwver"])
                        self.state.update_value("fwver", bytes(data))
                    except Exception as e:
                        self.state.log(f"Error leyendo fwver: {e}")

                # --- Leer c005 al conectar: valor base del contador de pulsaciones.
                #     Despues los notify van actualizando; el delta contra este
                #     valor base = pulsaciones ocurridas mientras estuve conectado. ---
                if "capture" in matched:
                    try:
                        data = await client.read_gatt_char(matched["capture"])
                        base = bytes(data)[0] if data else 0
                        self.state.update_value("capture", bytes(data))
                        self.state.log(f"Captura: contador base al conectar = {base}")
                    except Exception as e:
                        self.state.log(f"Error leyendo capture base: {e}")

                # --- Suscribirse a notify en el resto ---
                def make_handler(key):
                    def handler(_sender, data: bytearray):
                        self.state.update_value(key, bytes(data))
                    return handler

                subscribed = []
                for key, ch_uuid in matched.items():
                    if key == "fwver":
                        continue
                    try:
                        await client.start_notify(ch_uuid, make_handler(key))
                        subscribed.append(key)
                    except Exception as e:
                        self.state.log(f"No se pudo suscribir a {key}: {e}")

                if subscribed:
                    self.state.log(f"Suscrito a notify: {', '.join(subscribed)}")

                # --- Esperar hasta desconexion pedida o perdida de conexion ---
                #     Ademas del notify, se hace polling por READ de c005 (el
                #     contador de pulsaciones) cada 250 ms. Asi la cuenta avanza
                #     aunque las notificaciones no lleguen (p.ej. cache GATT de
                #     Windows, o el stack no entrega el callback).
                poll_capture = "capture" in matched
                while client.is_connected and not self._disconnect_requested.is_set() and not self._stop_flag.is_set():
                    if poll_capture:
                        try:
                            data = await client.read_gatt_char(matched["capture"])
                            self.state.update_value("capture", bytes(data), quiet=True)
                        except Exception as e:
                            self.state.log(f"Error en polling de capture: {e}")
                            poll_capture = False
                    await asyncio.sleep(0.25)

                self._disconnect_requested.clear()
                self.state.log("Desconectando...")

        except Exception as e:
            self.state.log(f"❌ Error de conexion: {e}")
            self.on_status(f"❌ Error: {e}")
        finally:
            self.state.connected = False
            self.client = None
            self.on_status("⏹️ Desconectado")


# ============================================================
# INTERFAZ GRAFICA (Tkinter)
# ============================================================
class RD90BleApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RD90 Console — Monitor BLE")
        self.geometry("760x680")
        self.configure(bg="#1e1e1e")

        self.state = BleState()
        self.worker = BleWorker(self.state, self._set_status)
        self.worker.start()

        self.devices = []  # lista de bleak.BLEDevice encontrados en el ultimo scan

        self._build_ui()
        self.after(150, self._refresh_loop)

    # ---------------------------------------------------------
    def _build_ui(self):
        FG, BG = "#e6e6e6", "#1e1e1e"
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TLabel", background=BG, foreground=FG, font=("Segoe UI", 10))
        style.configure("Title.TLabel", background=BG, foreground=FG, font=("Segoe UI", 14, "bold"))
        style.configure("TFrame", background=BG)
        style.configure("TButton", font=("Segoe UI", 10))
        style.configure("Treeview", font=("Segoe UI", 10), rowheight=24)
        style.configure("Treeview.Heading", font=("Segoe UI", 10, "bold"))

        title = ttk.Label(self, text="RD90 Console — Monitor BLE (GATT)", style="Title.TLabel")
        title.pack(pady=(10, 4))

        # --- Escaneo / conexion ---
        conn_frame = ttk.Frame(self)
        conn_frame.pack(pady=4, fill="x", padx=14)

        ttk.Button(conn_frame, text="Escanear", command=self._start_scan).pack(side="left")
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(conn_frame, textvariable=self.device_var, width=45, state="readonly")
        self.device_combo.pack(side="left", padx=6)

        self.connect_btn = ttk.Button(conn_frame, text="Conectar", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=4)

        self.status_label = ttk.Label(self, text="Desconectado", foreground="#e06666")
        self.status_label.pack(pady=(2, 8))

        # --- Tabla de valores en vivo ---
        table_frame = ttk.Frame(self)
        table_frame.pack(fill="x", padx=14, pady=4)
        ttk.Label(table_frame, text="Valores en vivo", style="Title.TLabel").pack(anchor="w")

        columns = ("valor", "hace")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="tree headings", height=6)
        self.tree.heading("#0", text="Característica")
        self.tree.heading("valor", text="Valor decodificado")
        self.tree.heading("hace", text="Última act.")
        self.tree.column("#0", width=170)
        self.tree.column("valor", width=430)
        self.tree.column("hace", width=90, anchor="center")
        self.tree.pack(fill="x", pady=4)

        self.row_ids = {}
        for key, meta in CHAR_DEFS.items():
            self.row_ids[key] = self.tree.insert("", "end", text=meta["label"], values=("(sin datos aun)", "-"))

        # --- Log crudo ---
        log_frame = ttk.Frame(self)
        log_frame.pack(fill="both", expand=True, padx=14, pady=(8, 10))
        ttk.Label(log_frame, text="Log (raw hex + eventos)", style="Title.TLabel").pack(anchor="w")

        self.log_text = tk.Text(log_frame, bg="#111111", fg="#9fd3a8", font=("Consolas", 9),
                                 insertbackground="#ffffff")
        self.log_text.pack(fill="both", expand=True, pady=4)
        self.log_text.configure(state="disabled")

        clear_btn = ttk.Button(log_frame, text="Limpiar log", command=self._clear_log)
        clear_btn.pack(anchor="e")

        self._last_log_len = 0

    # ---------------------------------------------------------
    # ESCANEO
    # ---------------------------------------------------------
    def _start_scan(self):
        self.device_combo["values"] = ["(escaneando...)"]
        threading.Thread(target=self._scan_thread, daemon=True).start()

    def _scan_thread(self):
        async def scan():
            return await BleakScanner.discover(timeout=5.0)

        try:
            devices = asyncio.run(scan())
        except Exception as e:
            self.after(0, lambda: messagebox.showerror("Error de escaneo", str(e)))
            return

        self.devices = devices
        labels = []
        for d in devices:
            name = d.name or "(sin nombre)"
            marker = " ⭐" if DEVICE_NAME_FILTER.lower() in name.lower() else ""
            labels.append(f"{name}{marker}  [{d.address}]")

        def apply():
            self.device_combo["values"] = labels
            if labels:
                # Preseleccionar el primero que matchea el filtro, si hay
                for i, d in enumerate(devices):
                    if DEVICE_NAME_FILTER.lower() in (d.name or "").lower():
                        self.device_combo.current(i)
                        break
                else:
                    self.device_combo.current(0)
            self._set_status(f"Escaneo terminado: {len(devices)} dispositivo(s) encontrado(s)")

        self.after(0, apply)

    # ---------------------------------------------------------
    # CONEXION
    # ---------------------------------------------------------
    def _toggle_connection(self):
        if self.state.connected:
            self.worker.request_disconnect()
            self.connect_btn.config(text="Conectar")
            return

        idx = self.device_combo.current()
        if idx < 0 or idx >= len(self.devices):
            messagebox.showwarning("Sin dispositivo", "Escaneá y seleccioná un dispositivo primero.")
            return

        address = self.devices[idx].address
        self.worker.request_connect(address)
        self.connect_btn.config(text="Desconectar")

    def _set_status(self, text):
        self.after(0, lambda: self.status_label.config(
            text=text,
            foreground="#6aab73" if text.startswith("✅") else
                       ("#e0a526" if text.startswith("🔄") else "#e06666"),
        ))

    # ---------------------------------------------------------
    # REFRESCO PERIODICO DE LA UI
    # ---------------------------------------------------------
    def _refresh_loop(self):
        with self.state.lock:
            values = dict(self.state.values)
            last_update = dict(self.state.last_update)
            log_lines = list(self.state.log_lines)

        now = time.time()
        for key, row_id in self.row_ids.items():
            ago = now - last_update[key]
            ago_txt = "-" if last_update[key] == 0 else (f"{ago:.1f}s" if ago < 60 else "hace rato")
            self.tree.item(row_id, values=(values[key], ago_txt))

        if len(log_lines) != self._last_log_len:
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.insert("end", "\n".join(log_lines))
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
            self._last_log_len = len(log_lines)

        self.after(150, self._refresh_loop)

    def _clear_log(self):
        with self.state.lock:
            self.state.log_lines = []
        self._last_log_len = -1  # fuerza redibujado

    def on_close(self):
        self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    app = RD90BleApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
