/**
 * OMRON E6B2-CWZ6C pinout
 * - Brown - Vcc
 * - Black - Phase A
 * - White - Phase B
 * - Orange - Phaze Z
 * - Blue - GND
 *
 * LPD3806-600BM-G5-24C pinout
 * - Green - Phase A
 * - White - Phase B
 * - Red - Vcc
 * - Black - GND
 */

 #include <Arduino.h>
 #include <avr/io.h>       // Incluye las definiciones de los registros
 // Definición de pines
 #define OUTPUT_A  2  // Motor encoder: PE5
 #define OUTPUT_B  3  // Motor encoder: PE4
 
 #define REF_OUT_A 18 // Pendulum encoder: PD3
 #define REF_OUT_B 19 // Pendulum encoder: PD2
 
 #define MOTOR_ENCODER_PPR      2400
 #define PENDULUM_ENCODER_PPR   2400
 #define SHAFT_R 0.00573
 
 #define PWM_PIN 10
 #define INA_PIN 6
 #define INB_PIN 7
 
 #define MAX_STALL_U 10
 #define POSITION_LIMIT  0.7   // HAY QUE ARREGLAR ESTO
 #define THETA_THRESHOLD (PI / 8)   // 22.5 grados

  // --------------------------
 // Setpoint
 float cartpositionsetpoint=0;
 float anglepositionsetpoint=PI;
 // Ganancias LQR (valores de ejemplo, sintonizar según el modelo)
 //const float LQR_K[4] = {1300.0, 220.0, 360.0, 260}; // 0.3 de variación
 //const float LQR_K[4] = {1400.0, 190.0, 400.0, 250}; // 0.3 de variación
 float LQR_K[4] = {-1800,220,-250,300};  // Inicializado en cero, pero se actualizará por serial
  
 // Variables globales para los encoders
 volatile long encoderValue = 0;
 volatile long lastEncoded = 0;
 
 volatile long refEncoderValue = 0;
 volatile long lastRefEncoded = 0;
 
 // Variables para tiempo utilizando micros() para mayor resolución
 unsigned long lastTimeMicros = 0;
 
 // Variables para el péndulo y el carro
 float theta = 0.0;
 float last_theta = 0.0;
 float last_w_filtered = 0.0;
 float last_v_filtered = 0.0;
 float filter_alpha = 0.5; // Filtro simple para la velocidad angular. Se planea añadir filtro de kalman.
 
 float x = 0.0;
 float last_x = 0.0;
 float u = 0.0;
 
 float dt = 0.0;
 
 unsigned long i = 0;
 unsigned long a = 0;
 
 // Declaración de funciones
 void encoderHandler();
 void refEncoderHandler();
 
 float avoidStall(float u) {
   if (fabs(u) < MAX_STALL_U) {
     return u > 0 ? 2 + MAX_STALL_U : -2 - MAX_STALL_U; // cambie stall
   }
   return u;
 }
 
 float saturate(float v, float maxValue) {
   return (fabs(v) > maxValue) ? (v > 0 ? maxValue : -maxValue) : v;
 }
 
 float getAngle(long pulses, long ppr) {
   return 2.0 * PI * pulses / ppr;
 }
 
 float getCartDistance(long pulses, long ppr) {
   return 2.0 * PI * pulses / ppr * SHAFT_R;
 }
 
 void driveMotor(float u) {
   int dir = (u > 0) ? 1 : (u < 0) ? -1 : 0;
   analogWrite(PWM_PIN, fabs(u));
   switch (dir) {
     case 1:
       digitalWrite(INA_PIN, HIGH);
       digitalWrite(INB_PIN, LOW);
       break;
     case -1:
       digitalWrite(INA_PIN, LOW);
       digitalWrite(INB_PIN, HIGH);
       break;
     default:
       digitalWrite(INA_PIN, LOW);
       digitalWrite(INB_PIN, LOW);
       break;
   }
 }
 
 boolean isControllable(float theta) {
   return fabs(anglepositionsetpoint-theta) < THETA_THRESHOLD;
 }
 
 void setup() {
   // Configurar PWM a 31kHz en pin 10
   TCCR2B = (TCCR2B & 0b11111000) | 0x01;
 
   // Configuración de pines para encoders
   pinMode(OUTPUT_A, INPUT_PULLUP);
   pinMode(OUTPUT_B, INPUT_PULLUP);
   pinMode(REF_OUT_A, INPUT_PULLUP);
   pinMode(REF_OUT_B, INPUT_PULLUP);
 
   // Configuración de interrupciones para encoders
   attachInterrupt(digitalPinToInterrupt(OUTPUT_A), encoderHandler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(OUTPUT_B), encoderHandler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(REF_OUT_A), refEncoderHandler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(REF_OUT_B), refEncoderHandler, CHANGE);
 
   // Configuración de pines para motor
   pinMode(PWM_PIN, OUTPUT);
   pinMode(INA_PIN, OUTPUT);
   pinMode(INB_PIN, OUTPUT);
   digitalWrite(INA_PIN, LOW);
   digitalWrite(INB_PIN, LOW);
 
   Serial.begin(115200);
   Serial.println("Time,Angle,Angular Velocity,Cart Position,Cart Speed,PWM");

   lastTimeMicros = micros(); // Inicializar tiempo
 }
 
 void loop() {
   // Usar micros() para mayor precisión en dt
   readLQRfromSerial();
   unsigned long nowMicros = micros();
   dt = (nowMicros - lastTimeMicros) / 1000000.0;  // dt en segundos
   lastTimeMicros = nowMicros;
   
   // Lectura y filtrado del ángulo del péndulo
   theta = getAngle(refEncoderValue, PENDULUM_ENCODER_PPR);
   float w = (theta - last_theta) / dt;
   theta = normalizeAngle(getAngle(refEncoderValue, PENDULUM_ENCODER_PPR));
   float w_filtered = filter_alpha * w + (1.0 - filter_alpha) * last_w_filtered;
   last_theta = theta;
   last_w_filtered = w_filtered;
   
   // Lectura y cálculo de la posición y velocidad del carro
   x = (getCartDistance(encoderValue, MOTOR_ENCODER_PPR))*10;
   last_x = x;
   float v = (x - last_x) / dt;
   float v_filtered = filter_alpha * v + (1.0 - filter_alpha) * last_v_filtered;
   last_v_filtered = v_filtered;
      
   // Aplicar la ley de control LQR si el péndulo está en rango y la posición es segura
   if (isControllable(theta) && fabs(x) < POSITION_LIMIT) {
     float state[4] = {theta, w_filtered, x, v };
     u = (LQR_K[0]*(anglepositionsetpoint-state[0]) - LQR_K[1]*state[1] + LQR_K[2]*(cartpositionsetpoint-state[2]) - LQR_K[3]*state[3] );
     u = saturate(avoidStall(u), 255);
   } else {
     u = 0.0;
   }
   driveMotor(u);
   if (a % 20 == 0){
      Serial.print(millis() / 1000.0, 4); Serial.print(',');
      Serial.print((theta*(180.0/PI)),      4);     Serial.print(',');
      Serial.print(w_filtered*(180.0/PI), 4);     Serial.print(',');
      Serial.print(x*(25),          4);     Serial.print(',');
      Serial.print(v*(25),          4);     Serial.print(',');
      Serial.println(u,        4);

   }
 a++;
  }

 void encoderHandler() {
   int MSB = (PINE & (1 << PE5)) >> PE5;
   int LSB = (PINE & (1 << PE4)) >> PE4;
   int encoded = (MSB << 1) | LSB;
   int sum = (lastEncoded << 2) | encoded;
 
   if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
     encoderValue++;
   }
   if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
     encoderValue--;
   }
   lastEncoded = encoded;
 }
 
 void refEncoderHandler() {
   int MSB = (PIND & (1 << PD3)) >> PD3;
   int LSB = (PIND & (1 << PD2)) >> PD2;
   int encoded = (MSB << 1) | LSB;
   int sum = (lastRefEncoded << 2) | encoded;
 
   if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
     refEncoderValue++;
   }
   if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
     refEncoderValue--;
   }
   lastRefEncoded = encoded;
 }
 
void readLQRfromSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); 
    input.trim();  // Elimina espacios en blanco o saltos de línea

    int idx = 0;
    int lastIndex = 0;
    for (int i = 0; i < 4; i++) {
      idx = input.indexOf(',', lastIndex);
      String value;
      if (idx == -1 && i < 3) {
        Serial.println("Error: Faltan valores LQR_K.");
        return;
      }

      value = (idx == -1) ? input.substring(lastIndex) : input.substring(lastIndex, idx);
      LQR_K[i] = value.toFloat();
      lastIndex = idx + 1;
    }

    Serial.print("LQR_K actualizado: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(LQR_K[i], 2);
      Serial.print(i < 3 ? ", " : "\n");
    }
  }
}

float normalizeAngle(float angle) {
  if (angle < 0)
    return angle + 2 * PI;
  if (angle >= 2 * PI)
    return angle - 2 * PI;
  return angle;
}
