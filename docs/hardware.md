# Hardware — Especificaciones Detalladas

## Jetson Nano 4GB (1ra gen ~2019)

- **OS:** Ubuntu 18.04 + JetPack 4.6.4
- **GPU:** Maxwell 128 CUDA cores
- **RAM:** 4 GB compartida CPU/GPU
- **Acceso SSH:** `ssh rodri@192.168.55.1` (via USB)
- **Catkin workspace:** `~/catkin_ws`

### Límites conocidos
- LLMs > 8B params no caben en 4GB
- RViz usa GPU intensivamente → pico de corriente
- Solución: `sudo nvpmodel -m 1` para modo 5W antes de RViz

## Orbbec Astra Pro Plus

- **Tipo:** Depth + RGB estructurada
- **Frecuencia:** 30 Hz
- **Topic ROS:** `/camera/depth/image_raw`
- **Estado:** ✅ Operativa desde sesión 24/03/2026
- **Latencia:** 0.031-0.034s por frame

### Notas de integración
- El SDK Python oficial **no tiene soporte ARM** → usar SDK C++ o `ros_astra_camera`
- Instalar udev rules antes de usar
- Consume bastante corriente USB — no conectar con muchos periféricos

## ESP32

- **Rol:** Control de motores DC, servos, comunicación I2C/SPI
- **Comunicación:** Serial UART o MQTT via WiFi
- **Estado:** ⏳ Disponible, pendiente de integrar

## Fuente de alimentación

- **Recomendada:** 5V/4A via conector barrel
- **Problema identificado:** Pico de corriente al usar RViz puede apagar el Jetson
- **Verificar:** Que la fuente pueda sostener 4A continuos
