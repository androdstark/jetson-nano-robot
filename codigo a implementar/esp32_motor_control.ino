/**
 * ESP32 Motor Control + Hall Encoder (KY-003)
 * L298N driver, 2 motores DC diferenciales
 * Comunicación Serial UART JSON (ArduinoJson v6)
 *
 * Pinout sugerido:
 *  Motor Izq : ENA=GPIO25, IN1=GPIO26, IN2=GPIO27
 *  Motor Der : ENB=GPIO14, IN3=GPIO12, IN4=GPIO13
 *  Encoder Izq: GPIO34 (input-only, no pull-up interno)
 *  Encoder Der: GPIO35 (input-only, no pull-up interno)
 */

#include <Arduino.h>
#include <ArduinoJson.h>  // v6

// ─── Parámetros configurables ───────────────────────────────────────────────
#define WHEEL_BASE     0.20f   // metros (distancia entre ruedas)
#define WHEEL_RADIUS   0.033f  // metros (radio de rueda)
#define TICKS_PER_REV  20      // número de imanes por rueda

#define PUBLISH_INTERVAL_MS  50   // cada cuántos ms publicar odometría
#define CMD_TIMEOUT_MS       500  // si no hay cmd, parar motores

// ─── Pines ──────────────────────────────────────────────────────────────────
#define PIN_ENA   25
#define PIN_IN1   26
#define PIN_IN2   27
#define PIN_ENB   14
#define PIN_IN3   12
#define PIN_IN4   13
#define PIN_ENC_L 34
#define PIN_ENC_R 35

// PWM (LEDC)
#define PWM_CH_L    0
#define PWM_CH_R    1
#define PWM_FREQ    20000   // 20 kHz — fuera del rango audible
#define PWM_BITS    8       // 0-255

// ─── Variables de encoder (IRAM para ISR rápida) ─────────────────────────────
volatile long ticks_l = 0;
volatile long ticks_r = 0;

void IRAM_ATTR isr_enc_l() { ticks_l++; }
void IRAM_ATTR isr_enc_r() { ticks_r++; }

// ─── Variables de control ────────────────────────────────────────────────────
float target_v = 0.0f;   // velocidad lineal deseada (m/s)
float target_w = 0.0f;   // velocidad angular deseada (rad/s)

unsigned long last_cmd_ms     = 0;
unsigned long last_publish_ms = 0;

// Snapshot previo para calcular delta
long prev_ticks_l = 0;
long prev_ticks_r = 0;
unsigned long prev_snapshot_ms = 0;

// ─── Utilidades de motor ─────────────────────────────────────────────────────
/**
 * Escribe velocidad a un motor.
 * speed: -1.0 (atrás máximo) … +1.0 (adelante máximo)
 */
void setMotorL(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  int pwm = (int)(fabsf(speed) * 255);
  if (speed >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  ledcWrite(PWM_CH_L, pwm);
}

void setMotorR(float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  int pwm = (int)(fabsf(speed) * 255);
  if (speed >= 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  }
  ledcWrite(PWM_CH_R, pwm);
}

void stopMotors() {
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ─── Cinemática diferencial v/w → vel_L, vel_R ──────────────────────────────
/**
 * vel_L y vel_R en m/s.  Luego se normalizan a [-1,1] dividiendo por
 * la velocidad máxima empírica del motor (ajustar MAX_WHEEL_SPEED).
 */
#define MAX_WHEEL_SPEED 0.5f   // m/s a PWM=255 — calibrar en campo

void cmdVelToMotors(float v, float w) {
  float vel_l = v - (w * WHEEL_BASE / 2.0f);
  float vel_r = v + (w * WHEEL_BASE / 2.0f);

  // Normalizar
  float norm_l = vel_l / MAX_WHEEL_SPEED;
  float norm_r = vel_r / MAX_WHEEL_SPEED;

  // Si alguno supera 1.0, escalar ambos proporcionalmente
  float max_abs = max(fabsf(norm_l), fabsf(norm_r));
  if (max_abs > 1.0f) {
    norm_l /= max_abs;
    norm_r /= max_abs;
  }

  setMotorL(norm_l);
  setMotorR(norm_r);
}

// ─── Cálculo de RPM ──────────────────────────────────────────────────────────
float ticksToRPM(long delta_ticks, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  float revs = (float)delta_ticks / (float)TICKS_PER_REV;
  return (revs / dt_s) * 60.0f;
}

// RPM → m/s en la llanta
float rpmToLinear(float rpm) {
  return rpm * 2.0f * PI * WHEEL_RADIUS / 60.0f;
}

// ─── Parseo de comandos Serial ────────────────────────────────────────────────
void parseSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;  // JSON inválido — ignorar

  if (doc.containsKey("v")) target_v = doc["v"].as<float>();
  if (doc.containsKey("w")) target_w = doc["w"].as<float>();
  last_cmd_ms = millis();
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);  // no bloquear en readStringUntil

  // Pines de dirección
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

  // LEDC PWM
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_ENA, PWM_CH_L);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_ENB, PWM_CH_R);

  stopMotors();

  // Encoders — GPIO34/35 son input-only, sin pull-up interno disponible
  // → usar resistencia externa de 10kΩ a 3.3V si el KY-003 es open-collector
  pinMode(PIN_ENC_L, INPUT);
  pinMode(PIN_ENC_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isr_enc_r, RISING);

  prev_snapshot_ms = millis();
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // 1. Leer comandos Serial
  parseSerial();

  // 2. Safety timeout
  if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    target_v = 0.0f;
    target_w = 0.0f;
  }

  // 3. Aplicar velocidades a motores
  cmdVelToMotors(target_v, target_w);

  // 4. Publicar odometría cada PUBLISH_INTERVAL_MS
  if ((now - last_publish_ms) >= PUBLISH_INTERVAL_MS) {
    last_publish_ms = now;

    // Snapshot atómico de ticks (deshabilitar interrupts brevemente)
    noInterrupts();
    long snap_l = ticks_l;
    long snap_r = ticks_r;
    interrupts();

    float dt_s = (now - prev_snapshot_ms) / 1000.0f;
    prev_snapshot_ms = now;

    long delta_l = snap_l - prev_ticks_l;
    long delta_r = snap_r - prev_ticks_r;
    prev_ticks_l = snap_l;
    prev_ticks_r = snap_r;

    float rpm_l = ticksToRPM(delta_l, dt_s);
    float rpm_r = ticksToRPM(delta_r, dt_s);
    float lv    = rpmToLinear(rpm_l);
    float rv    = rpmToLinear(rpm_r);

    // Preservar signo según dirección actual del motor
    // (los encoders Hall cuentan sin dirección — inferir del comando)
    if (target_v < 0.0f) { lv = -lv; rv = -rv; }

    StaticJsonDocument<128> out;
    out["lv"]      = lv;
    out["rv"]      = rv;
    out["ticks_l"] = snap_l;
    out["ticks_r"] = snap_r;

    serializeJson(out, Serial);
    Serial.print('\n');
  }
}
