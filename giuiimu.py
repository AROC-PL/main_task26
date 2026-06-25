#!/usr/bin/env python3
"""
imu_yaw_gui.py  –  GUI (TIDAK membutuhkan ROS2)
────────────────────────────────────────────────
• Terima  : Raw yaw (radian) dari imu_yaw_node.py via TCP socket
• Tampilkan: Grafik real-time raw & filtered yaw
• Kontrol : Parameter Kalman Filter (Q, R, X₀, P₀), Reset, Pause

Jalankan SETELAH imu_yaw_node.py berjalan:
    python3 imu_yaw_gui.py

Dependency:
    pip install matplotlib
"""

import math
import socket
import threading
import collections
import time
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── Koneksi ke ROS2 node ──────────────────────────────────────────────────────
HOST      = "127.0.0.1"
PORT      = 5005
MAXPOINTS = 300       # jumlah titik di grafik

# ── Warna tema ────────────────────────────────────────────────────────────────
DARK_BG  = "#1e1e2e"
PANEL_BG = "#2a2a3e"
ENTRY_BG = "#313244"
BORDER   = "#45475a"
TEXT_FG  = "#cdd6f4"
ACCENT   = "#89b4fa"   # biru
GREEN    = "#a6e3a1"
YELLOW   = "#f9e2af"
RED      = "#f38ba8"
PURPLE   = "#cba6f7"


# ═════════════════════════════════════════════════════════════════════════════
# Kalman Filter
# ═════════════════════════════════════════════════════════════════════════════
class KalmanFilter:
    def __init__(self, q=0.001, r=0.05, x=0.0, p=1.0):
        self.q = q
        self.r = r
        self.x = x
        self.p = p

    def update(self, z: float) -> float:
        self.p += self.q                        # predict
        k       = self.p / (self.p + self.r)   # gain
        self.x += k * (z - self.x)             # update
        self.p  = (1 - k) * self.p             # error
        return self.x

    @property
    def gain(self) -> float:
        return self.p / (self.p + self.r)


# ═════════════════════════════════════════════════════════════════════════════
# Socket client – berjalan di background thread
# ═════════════════════════════════════════════════════════════════════════════
class SocketClient(threading.Thread):
    """
    Terus mencoba konek ke node ROS2.
    Setiap baris yang diterima ("1.2345\n") diparsing dan dikirim ke callback.
    """

    def __init__(self, host, port, on_data, on_status):
        super().__init__(daemon=True)
        self._host      = host
        self._port      = port
        self._on_data   = on_data    # callback(yaw_rad: float)
        self._on_status = on_status  # callback(status: str, ok: bool)
        self._running   = True
        self._sock      = None

    def run(self):
        buf = ""
        while self._running:
            # ── Coba konek ───────────────────────────────────────────────────
            try:
                self._on_status("Menghubungkan ke node…", False)
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(3.0)
                self._sock.connect((self._host, self._port))
                self._sock.settimeout(None)
                self._on_status(f"Terhubung  {self._host}:{self._port}", True)
                buf = ""
            except (ConnectionRefusedError, OSError, socket.timeout):
                self._on_status("Node tidak ditemukan – coba lagi 3 s…", False)
                time.sleep(3)
                continue

            # ── Terima data ───────────────────────────────────────────────────
            try:
                while self._running:
                    chunk = self._sock.recv(256).decode(errors="ignore")
                    if not chunk:
                        break
                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            try:
                                self._on_data(float(line))
                            except ValueError:
                                pass
            except (OSError, ConnectionResetError):
                pass

            self._on_status("Koneksi terputus – reconnecting…", False)
            time.sleep(1)

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


# ═════════════════════════════════════════════════════════════════════════════
# Demo data (aktif saat tidak ada koneksi, untuk preview GUI)
# ═════════════════════════════════════════════════════════════════════════════
class DemoThread(threading.Thread):
    def __init__(self, on_data):
        super().__init__(daemon=True)
        self._on_data = on_data
        self._running = True
        self._t = 0.0

    def run(self):
        while self._running:
            yaw = math.sin(self._t * 0.3) * 1.5 + math.sin(self._t * 2.1) * 0.12
            self._on_data(yaw)
            self._t += 0.05
            time.sleep(0.05)

    def stop(self):
        self._running = False


# ═════════════════════════════════════════════════════════════════════════════
# Aplikasi GUI
# ═════════════════════════════════════════════════════════════════════════════
class IMUYawGUI:
    def __init__(self, root: tk.Tk, demo_mode=False):
        self.root = root
        self.root.title("IMU Yaw Monitor — GUI")
        self.root.configure(bg=DARK_BG)
        self.root.minsize(1050, 680)

        # ── State ─────────────────────────────────────────────────────────────
        self._lock    = threading.Lock()
        self._paused  = False
        self._demo_mode = demo_mode
        self.kalman   = KalmanFilter()
        self._t0      = time.time()

        self.t_buf  = collections.deque(maxlen=MAXPOINTS)
        self.r_buf  = collections.deque(maxlen=MAXPOINTS)   # raw  (deg)
        self.f_buf  = collections.deque(maxlen=MAXPOINTS)   # filt (deg)

        # ── Tkinter vars ──────────────────────────────────────────────────────
        self.sv_raw    = tk.StringVar(value="—")
        self.sv_filt   = tk.StringVar(value="—")
        self.sv_gain   = tk.StringVar(value="—")
        self.sv_status = tk.StringVar(value="⬤  Memulai…")
        self.sv_status_color = YELLOW

        # ── Build ─────────────────────────────────────────────────────────────
        self._build_ui()

        # ── Source data ───────────────────────────────────────────────────────
        if demo_mode:
            self._src = DemoThread(self._on_yaw)
            self._src.start()
            self._set_status("DEMO MODE (tanpa node ROS2)", False)
        else:
            self._src = SocketClient(
                HOST, PORT,
                on_data=self._on_yaw,
                on_status=self._set_status,
            )
            self._src.start()

        # ── Animasi ───────────────────────────────────────────────────────────
        self._ani = animation.FuncAnimation(
            self.fig, self._animate, interval=50,
            blit=False, cache_frame_data=False,
        )

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════════════
    # UI
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        # Title
        hdr = tk.Frame(self.root, bg=DARK_BG)
        hdr.pack(fill="x", padx=16, pady=(12, 0))

        tk.Label(hdr, text="📡  IMU Yaw Monitor",
                 font=("Segoe UI", 17, "bold"),
                 bg=DARK_BG, fg=ACCENT).pack(side="left")

        self._status_lbl = tk.Label(
            hdr, textvariable=self.sv_status,
            font=("Segoe UI", 9), bg=DARK_BG, fg=YELLOW,
        )
        self._status_lbl.pack(side="right")

        # Body
        body = tk.Frame(self.root, bg=DARK_BG)
        body.pack(fill="both", expand=True, padx=16, pady=10)

        self._build_panel(body)
        self._build_chart(body)

    # ── Panel kiri ────────────────────────────────────────────────────────────
    def _build_panel(self, parent):
        panel = tk.Frame(parent, bg=PANEL_BG,
                         highlightbackground=BORDER, highlightthickness=1)
        panel.pack(side="left", fill="y", padx=(0, 12), ipadx=12, ipady=10)

        def sep(title):
            tk.Label(panel, text=title,
                     font=("Segoe UI", 10, "bold"), bg=PANEL_BG, fg=ACCENT
                     ).pack(anchor="w", pady=(14, 2))
            ttk.Separator(panel).pack(fill="x", pady=(0, 6))

        # ── Live values ───────────────────────────────────────────────────────
        sep("📊  Nilai Live")
        self._lrow(panel, "Raw Yaw",      self.sv_raw,  YELLOW)
        self._lrow(panel, "Filtered Yaw", self.sv_filt, GREEN)
        self._lrow(panel, "Kalman Gain",  self.sv_gain, ACCENT)

        # ── Parameter Kalman ──────────────────────────────────────────────────
        sep("⚙️  Parameter Kalman Filter")

        self.e_q = self._prow(panel, "Q  –  Process Noise",    "0.001",
            "Kecil → lebih smooth, lambat adaptasi")
        self.e_r = self._prow(panel, "R  –  Measurement Noise","0.05",
            "Besar → lebih percaya model (filter lebih kuat)")
        self.e_x = self._prow(panel, "X₀ – Initial State",     "0.0",
            "Nilai awal estimasi yaw (radian)")
        self.e_p = self._prow(panel, "P₀ – Initial Error Cov", "1.0",
            "Ketidakpastian estimasi awal")

        # ── Tombol ────────────────────────────────────────────────────────────
        sep("🎛️  Kontrol")
        g = tk.Frame(panel, bg=PANEL_BG)
        g.pack(fill="x")

        self._btn(g, "✅  Apply Params",  self._apply,        ACCENT,  0, 0)
        self._btn(g, "🔄  Reset Filter",  self._reset_filter, YELLOW,  0, 1)
        self._btn(g, "🗑️  Clear Graph",   self._clear,        RED,     1, 0)
        self._pbtn = self._btn(g, "⏸  Pause", self._toggle_pause, GREEN, 1, 1)

        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)

        # ── Info koneksi ──────────────────────────────────────────────────────
        sep("ℹ️  Info")
        info = [
            f"Node → GUI  :  {HOST}:{PORT}",
            f"Buffer      :  {MAXPOINTS} titik",
            "Refresh     :  50 ms",
        ]
        for s in info:
            tk.Label(panel, text=s, font=("Consolas", 8),
                     bg=PANEL_BG, fg=TEXT_FG).pack(anchor="w")

    def _lrow(self, p, label, var, color):
        f = tk.Frame(p, bg=PANEL_BG)
        f.pack(fill="x", pady=2)
        tk.Label(f, text=label + ":", width=14, anchor="w",
                 font=("Segoe UI", 9), bg=PANEL_BG, fg=TEXT_FG).pack(side="left")
        tk.Label(f, textvariable=var,
                 font=("Consolas", 11, "bold"), bg=PANEL_BG, fg=color
                 ).pack(side="left")

    def _prow(self, p, label, default, tip):
        tk.Label(p, text=label, font=("Segoe UI", 9, "bold"),
                 bg=PANEL_BG, fg=TEXT_FG).pack(anchor="w", pady=(5, 0))
        e = tk.Entry(p, font=("Consolas", 10), width=20,
                     bg=ENTRY_BG, fg=TEXT_FG, insertbackground=TEXT_FG,
                     relief="flat", highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=ACCENT)
        e.insert(0, default)
        e.pack(anchor="w", pady=(2, 0))
        tk.Label(p, text=tip, font=("Segoe UI", 7, "italic"),
                 bg=PANEL_BG, fg="#6c7086", wraplength=215, justify="left"
                 ).pack(anchor="w")
        return e

    def _btn(self, p, text, cmd, color, row, col):
        b = tk.Button(p, text=text, command=cmd,
                      font=("Segoe UI", 9, "bold"),
                      bg=PANEL_BG, fg=color,
                      activebackground=ENTRY_BG, activeforeground=color,
                      relief="flat", bd=0, cursor="hand2",
                      highlightthickness=1, highlightbackground=color,
                      padx=6, pady=6)
        b.grid(row=row, column=col, padx=4, pady=4, sticky="ew")
        return b

    # ── Grafik ────────────────────────────────────────────────────────────────
    def _build_chart(self, parent):
        frame = tk.Frame(parent, bg=DARK_BG)
        frame.pack(side="left", fill="both", expand=True)

        self.fig = Figure(figsize=(8, 6), facecolor=PANEL_BG)
        self.fig.subplots_adjust(hspace=0.45, left=0.09,
                                 right=0.97, top=0.93, bottom=0.08)

        def style_ax(ax, title):
            ax.set_facecolor(DARK_BG)
            ax.set_title(title, color=ACCENT, fontsize=11,
                         fontweight="bold", pad=8)
            ax.tick_params(colors=TEXT_FG)
            for s in ax.spines.values():
                s.set_edgecolor(BORDER)
            ax.grid(True, color=BORDER, lw=0.5, ls="--")

        # Subplot 1 – time series
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        style_ax(self.ax1, "Yaw  vs  Waktu")
        self.ax1.set_ylabel("Yaw (°)", color=TEXT_FG)
        self.ax1.set_xlabel("Waktu (s)", color=TEXT_FG)
        self.ln_raw,  = self.ax1.plot([], [], color=YELLOW, lw=1.2,
                                      alpha=0.7, label="Raw")
        self.ln_filt, = self.ax1.plot([], [], color=GREEN,  lw=2.0,
                                      label="Filtered")
        self.ax1.legend(facecolor=ENTRY_BG, labelcolor=TEXT_FG,
                        edgecolor=BORDER, fontsize=8)

        # Subplot 2 – residual
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        style_ax(self.ax2, "Noise Residual  (Raw − Filtered)")
        self.ax2.set_ylabel("Δ Yaw (°)", color=TEXT_FG)
        self.ax2.set_xlabel("Waktu (s)", color=TEXT_FG)
        self.ln_diff, = self.ax2.plot([], [], color=RED, lw=1.0, alpha=0.85)
        self.ax2.axhline(0, color=BORDER, lw=0.8, ls=":")

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ══════════════════════════════════════════════════════════════════════════
    # Data & animasi
    # ══════════════════════════════════════════════════════════════════════════
    def _on_yaw(self, yaw_rad: float):
        if self._paused:
            return
        with self._lock:
            yaw_f = self.kalman.update(yaw_rad)
            t     = time.time() - self._t0
            self.t_buf.append(t)
            self.r_buf.append(math.degrees(yaw_rad))
            self.f_buf.append(math.degrees(yaw_f))

        # update label (thread-safe via tkinter .set)
        self.sv_raw.set(f"{math.degrees(yaw_rad):+.2f}°")
        self.sv_filt.set(f"{math.degrees(yaw_f):+.2f}°")
        self.sv_gain.set(f"{self.kalman.gain:.4f}")

    def _animate(self, _):
        with self._lock:
            t = list(self.t_buf)
            r = list(self.r_buf)
            f = list(self.f_buf)

        if len(t) < 2:
            return

        diff = [a - b for a, b in zip(r, f)]

        self.ln_raw.set_data(t, r)
        self.ln_filt.set_data(t, f)
        self.ln_diff.set_data(t, diff)

        for ax, vals in [(self.ax1, r + f), (self.ax2, diff)]:
            mn, mx = min(vals), max(vals)
            pad = max((mx - mn) * 0.15, 2.0)
            ax.set_xlim(t[0], max(t[-1], t[0] + 1))
            ax.set_ylim(mn - pad, mx + pad)

        self.canvas.draw_idle()

    # ══════════════════════════════════════════════════════════════════════════
    # Kontrol
    # ══════════════════════════════════════════════════════════════════════════
    def _parse_params(self):
        try:
            return (
                float(self.e_q.get()),
                float(self.e_r.get()),
                float(self.e_x.get()),
                float(self.e_p.get()),
            )
        except ValueError:
            messagebox.showerror("Input Error",
                                 "Semua nilai parameter harus berupa angka desimal.")
            return None

    def _apply(self):
        v = self._parse_params()
        if v is None:
            return
        q, r, x, p = v
        with self._lock:
            self.kalman.q = q
            self.kalman.r = r
            self.kalman.x = x
            self.kalman.p = p
        self._flash_status("✅  Parameter diterapkan", GREEN)

    def _reset_filter(self):
        v = self._parse_params()
        if v is None:
            return
        q, r, x, p = v
        with self._lock:
            self.kalman = KalmanFilter(q=q, r=r, x=x, p=p)
        self._flash_status("🔄  Filter direset", YELLOW)

    def _clear(self):
        with self._lock:
            self.t_buf.clear()
            self.r_buf.clear()
            self.f_buf.clear()
            self._t0 = time.time()
        self._flash_status("🗑️  Grafik dibersihkan", RED)

    def _toggle_pause(self):
        self._paused = not self._paused
        if self._paused:
            self._pbtn.config(text="▶  Resume", fg=YELLOW,
                              highlightbackground=YELLOW)
        else:
            self._pbtn.config(text="⏸  Pause",  fg=GREEN,
                              highlightbackground=GREEN)

    # ── Status label ─────────────────────────────────────────────────────────
    def _set_status(self, msg: str, ok: bool):
        color = GREEN if ok else YELLOW
        symbol = "⬤" if ok else "◌"
        self.sv_status.set(f"{symbol}  {msg}")
        self._status_lbl.config(fg=color)

    def _flash_status(self, msg: str, color: str):
        self.sv_status.set(msg)
        self._status_lbl.config(fg=color)
        prev_msg   = self.sv_status.get()
        self.root.after(2500, lambda: self.sv_status.set(
            "⬤  Terhubung" if not self._demo_mode else "⬤  DEMO MODE"
        ))

    # ── Tutup ─────────────────────────────────────────────────────────────────
    def _on_close(self):
        self._src.stop()
        self.root.destroy()


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════
def main():
    import sys
    demo = "--demo" in sys.argv   # jalankan: python3 imu_yaw_gui.py --demo

    root = tk.Tk()
    IMUYawGUI(root, demo_mode=demo)
    root.mainloop()


if __name__ == "__main__":
    main()