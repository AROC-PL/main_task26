#!/usr/bin/env python3

import json
import os
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import time
import matplotlib.pyplot as plt

# ─── FILE SAVE ───────────────────────────────────────
SAVE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pid_config.json")

DEFAULTS = {
    "pan":  {"kp": 1.0, "ki": 0.0, "kd": 0.07},
    "tilt": {"kp": 1.0, "ki": 0.0, "kd": 0.20},
}

def load_config():
    """Baca nilai terakhir dari file. Kalau belum ada, pakai DEFAULT."""
    if os.path.exists(SAVE_FILE):
        try:
            with open(SAVE_FILE, "r") as f:
                data = json.load(f)
            # validasi struktur
            for axis in ("pan", "tilt"):
                for param in ("kp", "ki", "kd"):
                    float(data[axis][param])
            print(f"[LOAD] Konfigurasi dibaca dari {SAVE_FILE}")
            return data
        except Exception as e:
            print(f"[LOAD] Gagal baca config, pakai default: {e}")
    return {
        "pan":  dict(DEFAULTS["pan"]),
        "tilt": dict(DEFAULTS["tilt"]),
    }

def save_config(data: dict):
    """Tulis nilai ke file JSON."""
    with open(SAVE_FILE, "w") as f:
        json.dump(data, f, indent=2)


# ─── ROS NODE ────────────────────────────────────────
class PIDPublisher(Node):
    def __init__(self):
        super().__init__("pid_tuner_gui_node")
        self.pub = self.create_publisher(String, "/pid_params", 10)
        self.start_time  = time.time()
        self.time_data   = []
        self.actual_pan  = []
        self.actual_tilt = []
        self.latest_pan  = 0.0
        self.latest_tilt = 0.0
        self.create_subscription(
            JointState, "/robotis/present_joint_states", self.joint_callback, 10
        )

    def joint_callback(self, msg):
        now = time.time() - self.start_time
        try:
            pi = msg.name.index("head_pan")
            ti = msg.name.index("head_tilt")
            self.latest_pan  = msg.position[pi]
            self.latest_tilt = msg.position[ti]
            self.time_data.append(now)
            self.actual_pan.append(self.latest_pan)
            self.actual_tilt.append(self.latest_tilt)
        except ValueError:
            pass

    def send(self, pan: dict, tilt: dict):
        payload = json.dumps({"pan": pan, "tilt": tilt})
        msg = String()
        msg.data = payload
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {payload}")


# ─── GUI ─────────────────────────────────────────────
class PIDTunerApp(tk.Tk):

    SLIDER_MIN = 0.0
    SLIDER_MAX = 5.0
    SAVE_DELAY = 500   # ms debounce sebelum tulis file

    def __init__(self, ros_node: PIDPublisher):
        super().__init__()
        self.ros_node = ros_node
        self.title("PID Tuner — Head Control")
        self.resizable(False, False)
        self.configure(bg="#F5F5F3")

        # Load dari file (atau default kalau belum ada)
        cfg = load_config()

        self._vars = {
            "pan":  {k: tk.DoubleVar(value=cfg["pan"][k])  for k in ("kp","ki","kd")},
            "tilt": {k: tk.DoubleVar(value=cfg["tilt"][k]) for k in ("kp","ki","kd")},
        }
        self._summary_vars = {}

        self.pan_val  = tk.StringVar(value="0.000 rad")
        self.tilt_val = tk.StringVar(value="0.000 rad")
        self._status  = tk.StringVar(value="● Siap")
        self._save_indicator = tk.StringVar(value="")

        self._save_job = None   # debounce handle

        self._build_ui()
        self._update_display()

    # ─── UI BUILD ────────────────────────────────────
    def _build_ui(self):
        BG = "#F5F5F3"
        tk.Label(self, text="PID Tuner — Head Control",
                 font=("Arial", 13, "bold"), bg=BG).pack(padx=16, pady=(12,2), anchor="w")

        # Baris subtitle + indikator save
        sub = tk.Frame(self, bg=BG)
        sub.pack(padx=16, fill="x")
        tk.Label(sub, text="Atur Kp / Ki / Kd tanpa colcon build",
                 font=("Arial", 9), fg="#888", bg=BG).pack(side="left")
        tk.Label(sub, textvariable=self._save_indicator,
                 font=("Arial", 9), fg="#3B6D11", bg=BG).pack(side="right", padx=4)

        self._build_axis_panel("HEAD PAN",  "pan",  "#185FA5", "#D6E8FA")
        self._build_axis_panel("HEAD TILT", "tilt", "#3B6D11", "#D8EDBE")
        self._build_actual_display()
        self._build_file_info()
        self._build_buttons()

        sb = tk.Frame(self, bg="#E8E7E2")
        sb.pack(fill="x", side="bottom")
        tk.Label(sb, textvariable=self._status,
                 font=("Arial", 9), bg="#E8E7E2", fg="#555").pack(side="left", padx=10, pady=3)

    def _build_axis_panel(self, title, axis, accent, header_bg):
        outer = tk.Frame(self, bg="#F5F5F3")
        outer.pack(padx=14, pady=(10,0), fill="x")

        card = tk.Frame(outer, bg="white", highlightthickness=1,
                        highlightbackground="#D0CFC9")
        card.pack(fill="x")

        hdr = tk.Frame(card, bg=header_bg)
        hdr.pack(fill="x")
        tk.Label(hdr, text=f"  {title}", font=("Arial", 10, "bold"),
                 bg=header_bg, fg=accent, pady=6).pack(side="left")

        self._summary_vars[axis] = {}
        summary_frame = tk.Frame(hdr, bg=header_bg)
        summary_frame.pack(side="right", padx=10)
        for param in ("kp", "ki", "kd"):
            sv = tk.StringVar(value=f"{self._vars[axis][param].get():.3f}")
            self._summary_vars[axis][param] = sv
            f = tk.Frame(summary_frame, bg=header_bg)
            f.pack(side="left", padx=5)
            tk.Label(f, text=f"{param.upper()}=", font=("Arial", 9),
                     bg=header_bg, fg=accent).pack(side="left")
            tk.Label(f, textvariable=sv, font=("Arial", 9, "bold"),
                     bg=header_bg, fg=accent, width=5).pack(side="left")

        body = tk.Frame(card, bg="white")
        body.pack(fill="x", padx=12, pady=8)
        for param in ("kp", "ki", "kd"):
            self._build_slider_row(body, axis, param, accent)

    def _build_slider_row(self, parent, axis, param, accent):
        var = self._vars[axis][param]
        row = tk.Frame(parent, bg="white")
        row.pack(fill="x", pady=4)

        tk.Label(row, text=param.upper(), width=4, anchor="w",
                 font=("Arial", 10, "bold"), bg="white", fg="#555").pack(side="left")

        ttk.Scale(row, from_=self.SLIDER_MIN, to=self.SLIDER_MAX,
                  orient="horizontal", variable=var
                  ).pack(side="left", fill="x", expand=True, padx=(6,8))

        entry = tk.Entry(row, textvariable=var, width=7,
                         font=("Arial", 10), justify="center",
                         bg="#EEF2F8", fg="#1A1A1A",
                         relief="flat", highlightthickness=1,
                         highlightbackground="#B0C8E8")
        entry.pack(side="left")

        var.trace_add("write", lambda *_, a=axis, p=param, v=var: self._on_var_change(a, p, v))
        entry.bind("<Return>",   lambda e, v=var, en=entry: self._sync_entry(v, en))
        entry.bind("<FocusOut>", lambda e, v=var, en=entry: self._sync_entry(v, en))

    def _build_actual_display(self):
        BG = "#F5F5F3"
        frame = tk.LabelFrame(self, text="  Posisi servo aktual  ",
                              font=("Arial", 9), bg=BG, fg="#555")
        frame.pack(padx=14, pady=(10,0), fill="x")
        inner = tk.Frame(frame, bg=BG)
        inner.pack(padx=10, pady=6)
        for col, (label, var) in enumerate([("HEAD PAN",  self.pan_val),
                                            ("HEAD TILT", self.tilt_val)]):
            tk.Label(inner, text=label, font=("Arial", 9),
                     bg=BG, fg="#666").grid(row=0, column=col*2, padx=(0,4), sticky="e")
            tk.Label(inner, textvariable=var,
                     font=("Arial", 11, "bold"), bg=BG, fg="#1A1A1A", width=10
                     ).grid(row=0, column=col*2+1, padx=(0,20), sticky="w")

    def _build_file_info(self):
        BG = "#F5F5F3"
        frame = tk.Frame(self, bg=BG)
        frame.pack(padx=14, pady=(6,0), fill="x")
        tk.Label(frame, text="File config:", font=("Arial", 8), fg="#999", bg=BG
                 ).pack(side="left")
        tk.Label(frame, text=SAVE_FILE, font=("Arial", 8), fg="#555", bg=BG
                 ).pack(side="left", padx=4)

    def _build_buttons(self):
        BG = "#F5F5F3"
        frame = tk.Frame(self, bg=BG)
        frame.pack(padx=14, pady=12)
        for text, cmd, bg, fg in [
            ("Reset Default",  self._reset,  "#E0DDD8", "#333"),
            ("Plot Grafik",    self._plot,   "#E0DDD8", "#333"),
            ("Kirim ke Robot", self._send,   "#185FA5", "white"),
        ]:
            tk.Button(frame, text=text, command=cmd,
                      font=("Arial", 10, "bold"), bg=bg, fg=fg,
                      relief="flat", padx=12, pady=6, cursor="hand2"
                      ).pack(side="left", padx=4)

    # ─── AUTO SAVE (debounce) ─────────────────────────
    def _on_var_change(self, axis, param, var):
        try:
            val = round(var.get(), 3)
            self._summary_vars[axis][param].set(f"{val:.3f}")

            # Print ke console
            pan_kp  = round(self._vars["pan"]["kp"].get(),  3)
            pan_ki  = round(self._vars["pan"]["ki"].get(),  3)
            pan_kd  = round(self._vars["pan"]["kd"].get(),  3)
            tilt_kp = round(self._vars["tilt"]["kp"].get(), 3)
            tilt_ki = round(self._vars["tilt"]["ki"].get(), 3)
            tilt_kd = round(self._vars["tilt"]["kd"].get(), 3)
            print(
                f"[HEAD PAN]  Kp={pan_kp:.3f}  Ki={pan_ki:.3f}  Kd={pan_kd:.3f}  |  "
                f"[HEAD TILT] Kp={tilt_kp:.3f}  Ki={tilt_ki:.3f}  Kd={tilt_kd:.3f}",
                end="\r"
            )

            # Debounce: tunda save 500ms agar tidak spam tulis saat slider gerak
            if self._save_job:
                self.after_cancel(self._save_job)
            self._save_indicator.set("💾 menyimpan...")
            self._save_job = self.after(self.SAVE_DELAY, self._do_save)

        except tk.TclError:
            pass

    def _do_save(self):
        """Tulis file JSON — dipanggil setelah debounce selesai."""
        data = {
            axis: {p: round(self._vars[axis][p].get(), 4) for p in ("kp","ki","kd")}
            for axis in ("pan", "tilt")
        }
        try:
            save_config(data)
            self._save_indicator.set(f"✔ tersimpan  {time.strftime('%H:%M:%S')}")
            print(f"\n[SAVE] {data} → {SAVE_FILE}")
        except Exception as e:
            self._save_indicator.set("✗ gagal simpan")
            print(f"\n[SAVE ERROR] {e}")

    # ─── HELPERS ─────────────────────────────────────
    def _sync_entry(self, var: tk.DoubleVar, entry: tk.Entry):
        try:
            val = float(entry.get())
            val = max(self.SLIDER_MIN, min(self.SLIDER_MAX, round(val, 4)))
            var.set(val)
        except ValueError:
            entry.delete(0, "end")
            entry.insert(0, f"{var.get():.3f}")

    def _get_values(self):
        return {
            axis: {p: round(self._vars[axis][p].get(), 4) for p in ("kp","ki","kd")}
            for axis in ("pan", "tilt")
        }

    # ─── REALTIME UPDATE ─────────────────────────────
    def _update_display(self):
        self.pan_val.set(f"{self.ros_node.latest_pan:.3f} rad")
        self.tilt_val.set(f"{self.ros_node.latest_tilt:.3f} rad")
        self.after(100, self._update_display)

    # ─── ACTIONS ─────────────────────────────────────
    def _send(self):
        vals = self._get_values()
        try:
            self.ros_node.send(vals["pan"], vals["tilt"])
            p, t = vals["pan"], vals["tilt"]
            status = (
                f"● Terkirim — "
                f"PAN: Kp={p['kp']} Ki={p['ki']} Kd={p['kd']}  |  "
                f"TILT: Kp={t['kp']} Ki={t['ki']} Kd={t['kd']}"
            )
            self._status.set(status)
            print(f"\n{status}")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            self._status.set(f"✗ Gagal: {e}")

    def _reset(self):
        for axis in ("pan", "tilt"):
            for param in ("kp", "ki", "kd"):
                self._vars[axis][param].set(DEFAULTS[axis][param])
        self._status.set("● Reset ke nilai default")
        print("\n[RESET] Nilai dikembalikan ke default.")

    def _plot(self):
        t_data    = self.ros_node.time_data
        pan_data  = self.ros_node.actual_pan
        tilt_data = self.ros_node.actual_tilt

        if len(t_data) < 5:
            messagebox.showinfo("Info", "Data belum cukup, tunggu beberapa detik.")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), tight_layout=True)
        ax1.plot(t_data, pan_data,  color="#185FA5", label="PAN actual")
        ax1.set_ylabel("rad"); ax1.legend(); ax1.grid(True)
        ax2.plot(t_data, tilt_data, color="#3B6D11", label="TILT actual")
        ax2.set_ylabel("rad"); ax2.set_xlabel("Time (s)"); ax2.legend(); ax2.grid(True)
        fig.suptitle("Posisi Servo Head — Real-time Log")
        plt.show()


# ─── MAIN ────────────────────────────────────────────
def main():
    rclpy.init()
    node = PIDPublisher()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    app = PIDTunerApp(node)
    app.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()