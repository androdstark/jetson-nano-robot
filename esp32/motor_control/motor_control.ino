/**
 * Motor Control para ESP32 — Robot Jetson Nano
 * Fase 1: Control de motores DC via comandos ROS (serial)
 * 
 * Comunicación: Serial UART @ 115200 baud
 * Protocolo: JSON {"v": float, "w": float}  (velocidad lineal, angular)
 * 
 * Conexiones L298N:
 *   IN1 → GPIO 26  |  IN2 → GPIO 27  (Motor izquierdo)
 *   IN3 → GPIO 14  |  IN4 → GPIO 12  (Motor derecho)
 *   ENA → GPIO 25  |  ENB → GPIO 13  (PWM)
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// Pines L298N
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENA 25
#define ENB 13

// Parámetros del robot (ajustar según chasis)
const float WHEEL_BASE = 0.15;   // distancia entre ruedas en metros
const int MAX_PWM = 200;          // PWM máximo (0-255)

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  Serial.println("{\"status\": \"ESP32 motor controller ready\"}");
}

void setMotor(int in1, int in2, int en, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(en, constrain(speed, 0, MAX_PWM));
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, line) == DeserializationError::Ok) {
      float v = doc["v"] | 0.0f;  // velocidad lineal m/s
      float w = doc["w"] | 0.0f;  // velocidad angular rad/s
      
      // Diferencial: vL = v - w*d/2, vR = v + w*d/2
      float vL = v - w * WHEEL_BASE / 2.0;
      float vR = v + w * WHEEL_BASE / 2.0;
      
      int pwmL = (int)(vL * MAX_PWM);
      int pwmR = (int)(vR * MAX_PWM);
      
      setMotor(IN1, IN2, ENA, pwmL);
      setMotor(IN3, IN4, ENB, pwmR);
    }
  }
}
