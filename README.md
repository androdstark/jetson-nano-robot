# 🤖 Jetson Nano Robot — Asistente con IA y Locomoción Adaptativa

> **Edge AI robot** basado en Jetson Nano. Procesamiento de visión local, LLM/TTS en la nube.
> Fase 1: navegación con ruedas | Fase 2: locomoción bípeda con piernas tipo delta (3 actuadores/pierna).

[![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue)](http://wiki.ros.org/melodic)
[![JetPack 4.6.4](https://img.shields.io/badge/JetPack-4.6.4-green)](https://developer.nvidia.com/embedded/jetpack)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## 🏗️ Arquitectura del Proyecto

```
┌─────────────────────────────────────────────────────┐
│                  JETSON NANO (edge)                  │
│  ┌────────────┐  ┌──────────┐  ┌─────────────────┐  │
│  │ Orbbec     │  │ YOLOv8n  │  │ openWakeWord    │  │
│  │ Astra Pro+ │→ │ 15-20fps │  │ "Hey Anderson"  │  │
│  │ 30Hz depth │  └──────────┘  └─────────────────┘  │
│  └────────────┘                                      │
│         ↓ ROS topics              ↓ audio            │
│  ┌────────────────────────────────────────────────┐  │
│  │ ROS Melodic — Navigation Stack (move_base)    │  │
│  │ gmapping / RTAB-Map — AMCL — costmap          │  │
│  └────────────────────────────────────────────────┘  │
│         ↓ serial / MQTT                              │
│  ┌──────────────┐                                    │
│  │    ESP32     │ → motores DC / servos / piernas    │
│  └──────────────┘                                    │
└─────────────────────────────────────────────────────┘
         ↕ internet (cuando disponible)
┌──────────────────────┐  ┌──────────────────────────┐
│  Claude API (LLM)    │  │  ElevenLabs API (TTS)    │
└──────────────────────┘  └──────────────────────────┘
```

---

## 📋 Hardware

| Componente | Descripción | Estado |
|---|---|---|
| Jetson Nano 4GB (1ra gen ~2019) | Cerebro principal, CUDA Maxwell 128 cores | ✅ |
| Orbbec Astra Pro Plus | Depth + RGB, 30 Hz | ✅ Operativa |
| ESP32 | Control de servos/motores via serial/MQTT | ⏳ Pendiente integrar |
| Micrófono USB | Entrada de voz | ✅ |
| Altavoz + amp | Salida de audio | ✅ |
| Chasis con ruedas (Fase 1) | Ruedas DC + encoders + L298N | ⏳ Por adquirir |
| Piernas delta 3-actuadores (Fase 2) | 3 servos/actuadores por pierna | 🔬 Diseño |

---

## 🚀 Fases del Proyecto

### Fase 1 — Navegación con Ruedas (ACTUAL)
- Chasis diferencial o mecanum 4 ruedas
- Motores DC con encoders para odometría
- Driver L298N o similar
- ROS Navigation Stack: `move_base` + `gmapping` + `AMCL`
- Sensor principal: Orbbec Astra Pro Plus (depth → costmap de obstáculos)

### Fase 2 — Locomoción Bípeda Delta (FUTURA)
Inspirándose en la cinemática de las **impresoras 3D delta**, cada pierna usa **3 actuadores en paralelo** que controlan la posición del extremo (pie) en 3D mediante cadenas cinemáticas paralelas. Esto da:
- Mayor rigidez estructural vs cadena serial
- Velocidad de movimiento más alta
- Actuadores en la parte superior (menor inercia en extremidades)
- Similar al **NVIDIA Olaf** presentado para Disney

```
         [Cuerpo]
        /    |    \
      A1     A2    A3   ← 3 actuadores por pierna (montados arriba)
      |      |      |
       \     |     /
        [Efector/pie]
```

---

## 💻 Stack de Software

| Capa | Herramienta | Notas |
|---|---|---|
| OS | Ubuntu 18.04 + JetPack 4.6.4 | |
| Middleware | ROS Melodic | |
| Visión | YOLOv8n + OpenCV | ~15-20 fps |
| Depth | ros_astra_camera | 30 Hz ✅ |
| Skeleton | MediaPipe / NuiTrack | ⏳ |
| STT | Whisper tiny/base + openWakeWord | ⏳ probar |
| LLM | Claude API (nube) + llama3.2:3b Q4 (offline) | |
| TTS | Piper TTS (offline) + ElevenLabs (premium) | |
| Control | ESP32 via serial/MQTT | |
| Navegación | gmapping + move_base + AMCL | ⏳ |

---

## ⚡ Inicio Rápido

```bash
# 1. Clonar el repositorio en el Jetson
ssh rodri@192.168.55.1
git clone https://github.com/androdstark/jetson-nano-robot.git ~/robot_ws/src/jetson-nano-robot

# 2. Build del workspace ROS
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 3. Lanzar la cámara
roslaunch astra_camera astra_pro.launch

# 4. Ver el depth stream
python ~/robot_ws/src/jetson-nano-robot/scripts/depth_view.py
```

---

## 📁 Estructura del Repositorio

```
jetson-nano-robot/
├── docs/                    # Documentación técnica
│   ├── hardware.md          # Especificaciones de hardware
│   ├── phase1_wheels.md     # Guía Fase 1: ruedas
│   └── phase2_legs_delta.md # Diseño Fase 2: piernas delta
├── scripts/                 # Scripts Python standalone
│   ├── depth_view.py        # Visualización depth sin GPU
│   └── yolo_detect.py       # Detección de objetos YOLOv8
├── ros_packages/            # Paquetes ROS Melodic
│   ├── robot_navigation/    # move_base, costmaps, AMCL
│   ├── robot_perception/    # Nodos de visión
│   └── robot_control/       # Control ESP32 + servos
├── esp32/                   # Firmware ESP32
│   └── motor_control/       # Control de motores DC
├── config/                  # Archivos de configuración ROS
└── launch/                  # Archivos .launch de ROS
```

---

## 🔗 Referencias

- [ros_astra_camera](https://github.com/orbbec/ros_astra_camera)
- [openWakeWord](https://github.com/dscripka/openWakeWord)
- [Piper TTS](https://github.com/rhasspy/piper)
- [Ollama en Jetson](https://ollama.com/blog/jetson)
- [YOLOv8](https://github.com/ultralytics/ultralytics)
- [NVIDIA Olaf (Disney)](https://www.nvidia.com/en-us/research/robotics/)
- [Yahboom ROSMASTER X3 Plus](https://github.com/YahboomTechnology/ROSMASTERX3-PLUS)

---

## 📄 Licencia

MIT © 2026 Rodrigo Alvarado Anderson (@sranderson)
