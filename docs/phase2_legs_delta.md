# Fase 2 — Locomoción Bípeda con Piernas Delta

## Concepto: Piernas tipo Impresora Delta

Cada pierna usa **3 actuadores en paralelo** (similar a la cinemática de impresoras 3D delta).
Esto controla la posición del pie en el espacio 3D mediante una plataforma Stewart simplificada.

### Ventajas sobre cadena cinemática serial

| Característica | Serial (ej. rodilla+cadera) | Delta Paralela (3 actuadores) |
|---|---|---|
| Rigidez | Media | Alta |
| Velocidad máxima | Limitada | Alta |
| Inercia en extremidades | Alta (motores distribuidos) | Baja (actuadores en torso) |
| Complejidad cinemática | Simple | Media (cinemática inversa paralela) |
| Referencia | Humanoides clásicos | Impresoras delta, NVIDIA Olaf |

## Esquema mecánico

```
         [TORSO/CUERPO]
        /      |      \
      A1       A2      A3     ← 3 actuadores (montados en torso)
      |        |       |
    rod1     rod2    rod3     ← varillas de longitud fija
      \       |      /
       [PLATAFORMA/PIE]       ← efector final: pie del robot
```

## Cinemática Inversa (IK)

Dado un punto objetivo del pie `(x, y, z)`, calcular los ángulos de los 3 actuadores:

```python
import numpy as np

# Parámetros de la plataforma delta
R_BASE = 0.08    # radio de la base (torso) en metros
R_EFF = 0.03     # radio del efector (pie)
L_ROD = 0.12     # longitud de las varillas
L_ARM = 0.05     # longitud del brazo del actuador

ACTUATOR_ANGLES = [0, 120, 240]  # grados, posición angular de cada actuador

def delta_ik(x, y, z):
    """Cinemática inversa para plataforma delta.
    Retorna ángulos en grados para los 3 actuadores."""
    angles = []
    for theta_deg in ACTUATOR_ANGLES:
        theta = np.radians(theta_deg)
        x0 = R_BASE * np.cos(theta)
        y0 = R_BASE * np.sin(theta)
        xe = x + R_EFF * np.cos(theta)
        ye = y + R_EFF * np.sin(theta)
        dx = xe - x0
        dy = ye - y0
        dz = z
        # Distancia horizontal del efector al actuador
        d = np.sqrt(dx**2 + dy**2)
        # Calcular ángulo del actuador
        cos_beta = (L_ARM**2 + d**2 + dz**2 - L_ROD**2) / (2 * L_ARM * np.sqrt(d**2 + dz**2))
        if abs(cos_beta) > 1:
            raise ValueError(f"Punto ({x},{y},{z}) fuera del workspace")
        beta = np.degrees(np.arccos(cos_beta))
        angles.append(beta)
    return angles

# Ejemplo: mover pie a posición (0, 0, -0.15) — posición neutra
print(delta_ik(0.0, 0.0, -0.15))
```

## Gait Planning (marcha bípeda)

Ciclo de marcha básico (2 piernas):

```
Fase 1: Pierna A en soporte → Pierna B se levanta y avanza
Fase 2: Pierna B toca suelo → transferencia de peso
Fase 3: Pierna B en soporte → Pierna A se levanta y avanza
Fase 4: Repetir
```

### Trayectoria del pie (swing)

```python
def foot_trajectory(t, step_length=0.05, step_height=0.03, duration=0.5):
    """Genera trayectoria de pie en fase swing.
    t: tiempo normalizado [0, 1]
    Retorna (x, y, z) del pie."""
    x = step_length * t
    z = -0.12 + step_height * np.sin(np.pi * t)  # arco parabólico
    y = 0.0
    return (x, y, z)
```

## Referencia: NVIDIA Olaf (Disney)

El robot NVIDIA Olaf usa piernas con actuadores en paralelo montados en el torso,
permitiendo extremidades livianas y alta velocidad de movimiento. El mismo principio
se aplica aquí: los 3 servos/actuadores permanecen en el cuerpo, y solo varillas
ligeras llegan al pie.

**Links de referencia:**
- NVIDIA Isaac robotics platform
- MIT Cheetah (actuadores paralelos)
- Boston Dynamics Atlas kinematics

## Actuadores recomendados

| Opción | Tipo | Par | Velocidad | Precio |
|---|---|---|---|---|
| MG996R | Servo estándar | 9.4 kg·cm | 60°/0.19s | ~$5 c/u |
| DS3225 | Servo metal | 25 kg·cm | 60°/0.16s | ~$12 c/u |
| Dynamixel XL430 | Smart servo | 14.7 kg·cm | TTL | ~$50 c/u |
| Motor brushless + encoder | BLDC | Alto | Alto | ~$20 c/u |

**Para fase 2 inicial:** DS3225 (6 pierna × 3 actuadores = 18 servos total)
