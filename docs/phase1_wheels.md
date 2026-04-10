# Fase 1 — Navegación con Ruedas

## Objetivo

Robot móvil funcional con navegación autónoma usando ROS Navigation Stack sobre un chasis de ruedas.

## Hardware requerido

| Componente | Opción económica | Precio estimado |
|---|---|---|
| Chasis 4 ruedas | Genérico AliExpress | $15-30 |
| Motores DC con encoders | JGA25-370 o similar | $10-20 |
| Driver de motores | L298N | $3-5 |
| Batería LiPo | 3S 2200mAh | $15-25 |
| ESP32 | Ya disponible | $0 |

## Stack ROS Melodic

```
Orbbec depth → depthimage_to_laserscan → /scan
                                            ↓
                                      gmapping → /map
                                            ↓
                                    move_base ← goal (RVIZ/topic)
                                            ↓
                                  /cmd_vel → ESP32 → motores
```

## Configuración de nodos

### 1. depth_to_laserscan

```yaml
# config/depth_to_laserscan.yaml
scan_height: 1
output_frame_id: camera_link
range_min: 0.45
range_max: 8.0
```

### 2. gmapping

```yaml
# config/gmapping.yaml
throttle_scans: 1
base_frame: base_link
map_frame: map
odom_frame: odom
map_update_interval: 5.0
maxUrange: 6.0
sigma: 0.05
particles: 30
```

### 3. move_base (costmaps)

```yaml
# config/costmap_common.yaml
obstacle_range: 3.0
raytrace_range: 3.5
footprint: [[-0.15, -0.15], [-0.15, 0.15], [0.15, 0.15], [0.15, -0.15]]
inflation_radius: 0.2
```

## Odometría

La odometría se calcula en el ESP32 a partir de encoders y se publica como topic `/odom`:

```
Encoders (ESP32) → /odom (nav_msgs/Odometry) → ROS Navigation
```

## Comandos de lanzamiento

```bash
# Terminal 1 — cámara
roslaunch astra_camera astra_pro.launch

# Terminal 2 — navegación completa
roslaunch robot_navigation navigation.launch

# Terminal 3 — control ESP32
rosrun robot_control esp32_bridge.py
```
