"""
GUI para capturar el log serie de un ESP32 desde el instante en que el
puerto aparece (pensado para USB-CDC nativo, que se desconecta/reconecta
al resetear el chip).

Uso:
    python serial_catch_boot.py

Elegi el puerto COM, apreto "Iniciar", y dejalo corriendo. Apretá el
boton de reset del robot las veces que haga falta: la app detecta cuando
el puerto desaparece (reset) y se reconecta sola apenas Windows lo vuelve
a enumerar, sin perderse las primeras lineas del boot.
"""

import datetime
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

import serial
import serial.tools.list_ports

POLL_INTERVAL_S = 0.02
MAX_LOG_LINES = 5000


class SerialReaderThread(threading.Thread):
    def __init__(self, port, baud, line_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.line_queue = line_queue
        self.stop_event = stop_event

    def emit(self, text):
        stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.line_queue.put(f"[{stamp}] {text}")

    def wait_for_port(self):
        while not self.stop_event.is_set():
            ports = {p.device for p in serial.tools.list_ports.comports()}
            if self.port in ports:
                return True
            time.sleep(POLL_INTERVAL_S)
        return False

    def run(self):
        self.emit(f"--- Esperando el puerto {self.port} ---")
        while not self.stop_event.is_set():
            if not self.wait_for_port():
                break

            self.emit(f"--- {self.port} detectado, abriendo ---")
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
            except serial.SerialException as e:
                self.emit(f"--- No se pudo abrir todavia ({e}), reintentando ---")
                time.sleep(0.1)
                continue

            self.emit(f"--- {self.port} abierto @ {self.baud} ---")
            buf = b""
            try:
                while not self.stop_event.is_set():
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.emit(line.decode(errors="replace").rstrip("\r"))
            except (serial.SerialException, OSError):
                self.emit(f"--- {self.port} se desconecto (reset o desenchufado) ---")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

        self.emit("--- Detenido ---")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Captura de boot serie - RD90")
        self.geometry("900x600")

        self.reader_thread = None
        self.stop_event = None
        self.line_queue = queue.Queue()
        self.out_file = None

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_queue)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Puerto:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))

        ttk.Button(top, text="Refrescar", command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(12, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=(4, 8))

        self.start_btn = ttk.Button(top, text="Iniciar", command=self._start)
        self.start_btn.pack(side=tk.LEFT, padx=(12, 4))
        self.stop_btn = ttk.Button(top, text="Detener", command=self._stop, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="Guardar log...", command=self._save_log).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Button(top, text="Limpiar", command=self._clear_log).pack(side=tk.LEFT, padx=4)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Auto-scroll", variable=self.autoscroll_var).pack(side=tk.LEFT, padx=(12, 4))

        self.status_var = tk.StringVar(value="Detenido")
        ttk.Label(self, textvariable=self.status_var, padding=(8, 2), anchor="w").pack(side=tk.TOP, fill=tk.X)

        text_frame = ttk.Frame(self)
        text_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.text = tk.Text(text_frame, wrap="none", bg="#111", fg="#ddd", insertbackground="#ddd")
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        yscroll = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.text.configure(yscrollcommand=yscroll.set)

        # Resaltar lineas clave del boot que nos interesan
        self.text.tag_configure("highlight", foreground="#7CFC00")
        self.text.tag_configure("marker", foreground="#5DADE2")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _start(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("Elegi un puerto primero")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self.status_var.set("Baud invalido")
            return

        self.stop_event = threading.Event()
        self.reader_thread = SerialReaderThread(port, baud, self.line_queue, self.stop_event)
        self.reader_thread.start()

        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"Corriendo en {port} @ {baud}")

    def _stop(self):
        if self.stop_event:
            self.stop_event.set()
        self.start_btn.configure(state=tk.NORMAL)
        self.stop_btn.configure(state=tk.DISABLED)
        self.status_var.set("Detenido")

    def _save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Texto", "*.txt")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.text.get("1.0", tk.END))
        self.status_var.set(f"Log guardado en {path}")

    def _clear_log(self):
        self.text.delete("1.0", tk.END)

    def _poll_queue(self):
        try:
            while True:
                line = self.line_queue.get_nowait()
                self._append_line(line)
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    def _append_line(self, line):
        tag = None
        if line.startswith("---"):
            tag = "marker"
        elif any(key in line for key in ("Serial:", "Modelo:", "Modbus Slave", "Slave Modbus")):
            tag = "highlight"

        self.text.insert(tk.END, line + "\n", tag)

        # Recortar si crece demasiado
        line_count = int(self.text.index("end-1c").split(".")[0])
        if line_count > MAX_LOG_LINES:
            self.text.delete("1.0", f"{line_count - MAX_LOG_LINES}.0")

        if self.autoscroll_var.get():
            self.text.see(tk.END)

    def on_close(self):
        if self.stop_event:
            self.stop_event.set()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
