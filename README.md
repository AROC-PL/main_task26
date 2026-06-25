# `main_task26` — Python Source Package

Direktori source Python dari ROS2 package `main_task26` untuk robot humanoid **ROBOTIS OP3** bermain sepak bola otonom. Berisi 17 file `.py` + 1 `pid_config.json` — 7 node ROS2 utama, 4 node utility, 2 GUI, 2 library, dan 2 file tak terpakai.

## Daftar File

### 7 Node Utama (dijalankan via `main_launch.py`)

| File | Class / Node | Deskripsi |
|---|---|---|
| `main_action.py` | `FallRecoveryFull` (`fall_recovery_full`) | Deteksi jatuh via IMU (akselerasi sumbu X), recovery via action page (142/143), kick decision kiri/kanan saat jarak bola < threshold |
| `crab_walk.py` | `GaitController` (`gait_controller`) | Base class walking parameter: period 0.95s, DSP 0.2, step FB 0.28, X/Y/Z amplitude, arm swing gain, balance enable. Publishes ke `/robotis/walking/set_params` |
| `main_buttonhandler.py` | `ButtonSoccerNode(GaitController)` (`button_soccer_node`) | Turunan GaitController. Handle tombol Open-CR: user (jalan diagonal), start (jalan lurus), mode (toggle soccer/walking module), double-click detection |
| `main_head_control.py` | `HeadControl` (`button_stand_scan_node`) | PID tracking kepala via `/obj_detect`, auto-scan saat bola hilang, subscribe `/pid_params` live retuning, state machine NORMAL/KICK/RECOVER |
| `main_vision.py` | `VisionYolo` (`usb_cam_yolo`) | YOLO OpenVINO inference via USB camera, bounding box + FPS, publish `/obj_detect` (string koordinat bola) dan `/vision/jarak_kamera` (Float32) |
| `main_task.py` | `MainTask` (`main_task_node`) | Koordinator utama (legacy), subscribe `/vision/jarak_kamera`, MultiThreadedExecutor. Sebagian node di-comment |
| `jarak.py` | `JarakCalculation` (`jarak_calculation`) | Hitung jarak bola dari head tilt: `jarak = (tinggi_robot * tan(sudut_kamera - sudut_mat)) - kalibrasi_jarak` |

### 4 Node Utility

| File | Class / Node | Deskripsi |
|---|---|---|
| `cek.py` | `HeadListener` (`head_listener`) | Debug: print `head_pan` & `head_tilt` dari `/robotis/goal_joint_states` |
| `hadap.py` | `ImuYawNode` + `KalmanFilter` 1D (`imu_yaw_node`) | Kalkulasi yaw dari quaternion IMU, Kalman filter (Q=0.001, R=0.05), log ke console |
| `imucek.py` | `YawSocketServer` + ROS2 node | Yaw streaming server via TCP port 5005, multi-client support, kirim raw yaw radian ke GUI |
| `imucsv.py` | `ImuSubscriber` (`imu_subscriber`) | Log quaternion IMU (x, y, z, w) ke `imu_data_belakang.csv` |

### 2 GUI (Tkinter)

| File | Deskripsi |
|---|---|
| `tunning_PID.py` | PID Tuner GUI — slider Kp/Ki/Kd untuk pan & tilt, realtime plot (matplotlib), auto-save/load ke `pid_config.json`, publish ke `/pid_params`. Berjalan sebagai ROS2 node |
| `giuiimu.py` | IMU Yaw GUI via TCP — grafik real-time raw & filtered yaw, adjustable Kalman (Q, R, initial state), dark theme. Bisa jalan tanpa ROS2 (demo mode `--demo`) |

### 2 Library

| File | Deskripsi |
|---|---|
| `main_PID.py` | 3 class PID: `PID` (basic sample time), `AdaptivePID` (gain scheduling), `PIDControl` (anti-windup + output clamping). Total ~255 baris |
| `kalman_filter.py` | `KalmanFilter2D` — 4-state (cx, cy, vx, vy), predict + update, Joseph form stabil numerik, dt clamp. **Saat ini tidak aktif** (di-comment di main_vision.py) |

### File Tak Terpakai

| File | Keterangan |
|---|---|
| `kick_useless.py` | `KickDecision` — implementasi kick terpisah memanggil `FallRecoveryFull`. Seluruh `main()` di-comment. Logika kick sudah pindah ke `main_action.py` |
| `motion_PID.py` | **Bukan Python source** — ini PyInstaller archive (binary zip), jangan diedit |

### Konfigurasi

| File | Isi |
|---|---|
| `pid_config.json` | Default PID: `pan` (Kp=0, Ki=0.02, Kd=0.15), `tilt` (Kp=0, Ki=0, Kd=0) |
| `__init__.py` | Package marker (kosong) |

## Ringkasan Alur Data

```
USB Camera
  └─► main_vision.py (YOLO)
        ├─► /obj_detect ──────────► main_head_control.py (PID tracking + auto scan)
        └─► /vision/jarak_kamera ──► main_action.py (kick decision)
                                    └─► main_task.py (coordinator)

IMU Open-CR
  ├─► main_action.py (fall detection via X acceleration)
  ├─► hadap.py (yaw calculation) ──► imucek.py (TCP server :5005) ──► giuiimu.py (GUI)
  └─► imucsv.py (CSV logger)

Tombol Open-CR
  └─► main_buttonhandler.py (mode: jalan lurus / diagonal / toggle)

Joint States
  ├─► crab_walk.py (head pan feedback for direction)
  ├─► cek.py (debug monitor)
  └─► tunning_PID.py (PID tuner GUI feedback)
```

## Catatan untuk Developer

- **Kalman Filter Tidak Aktif** — `kalman_filter.py` sudah implementasi lengkap (predict, update, Joseph form), tapi semua panggilannya di `main_vision.py` di-comment. Juga 1D Kalman di `hadap.py` di-comment.
- **Path Model Absolute** — `main_vision.py:32-35` hardcoded ke `/home/aroc/main_task26/main_task26/model/best_openvino_model`. Sebaiknya diubah jadi parameter ROS2 atau path relatif.
- **Typo Entry Point** — `setup.py` line 31 menulis `buttonhandler` bukan `main_buttonhandler`. File asli adalah `main_buttonhandler.py`. Di `entry_points.txt` hasil build sudah diperbaiki.
- **`motion_PID.py`** — Bukan source Python, melainkan PyInstaller archive. Jangan dibuka sebagai kode.
- **`kick_useless.py`** — Seluruh `main()` di-comment. Logika kick sudah terintegrasi di `main_action.py`.
