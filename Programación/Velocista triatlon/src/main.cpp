#include <Arduino.h>
#include <BluetoothSerial.h>

#define D1 39
#define D2 34
#define D3 35
#define D4 32
#define D5 33
#define D6 25
#define D7 26
#define D8 27

// ===== Motores ====
#define IN1A 22
#define IN1B 23 //bts 1
#define IN2A 4
#define IN2B 15 //bts 2
int VMAX = 180; //APROX RPM
int VBASE = 160; //APROX RPM
int VMIN = 140; //APROX RPM

#define led 2
#define boton 17

// ===== PWM's =====
int PWM1 = VBASE;  //pwm de la izquierda
int PWM2 = VBASE;  //pwm de la derecha

// Configuración de PWM para control de motores
const int frequency = 1000;
const int resolution = 8;

// ===== Canales PWM del ESP32 =====
const int ledChannel = 0;
const int ledChannel1 = 1;
const int ledChannel2 = 2;
const int ledChannel3 = 3;

// ===== Constantes y vaiables PID =====
float kp = -2;     //proporcional-presente
float ki = 0.01;  //integral-pasado
float kd = 0.5;   //derivativo-futuro
float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float outputPID = 0;
float error = 0, lastError = 0, integral = 0, derivative = 0;
float setpoint = 400;
int   correccion = 0;

// ========= Sensores =========
const int sensores[8] = {D1, D2, D3, D4, D5, D6, D7, D8}; // pines de la regleta - se puede poner esto en vez del define -
int valor_blanco[8];
int valor_negro[8];
int valor_umbrales[8];
bool valor_binario[8];


/*left = izq
  rigth = der*/

// ===== INICIALIZACION DEL BLUETOOTH =====
BluetoothSerial SerialBT;

// ===== FUNCIONES =====
void motores(int izq, int der) {  //0 hasta 255 adelante 0 hasta -255 atras

  if (izq >= 0) {
    ledcWrite(ledChannel, izq);
    ledcWrite(ledChannel1, 0);  //analog
  } else {
    izq = izq * (-1);
    ledcWrite(ledChannel1, izq);
    ledcWrite(ledChannel, 1);
  }
  //motor derecho//
  if (der >= 0) {
    ledcWrite(ledChannel2, der);
    ledcWrite(ledChannel3, 0);
  } else {
    der = der * (-1);
    ledcWrite(ledChannel3, der);
    ledcWrite(ledChannel2, 1);
  }
}

void prenderLedPorTiempo(unsigned long tiempo_ms){
    digitalWrite(led, LOW);
    delay(tiempo_ms);
    digitalWrite(led, HIGH);
}

void precaucion(unsigned long espera) {
motores(0,0);
prenderLedPorTiempo(espera);
}

int calcularPID(int lectura) {
  error = setpoint - lectura;
  integral += error;
  derivative = error - lastError;
  lastError = error;
  return kp * error + ki * integral + kd * derivative;
}

void calibrar_sensores(){ //si se presiona una vez el boton empieza a leer valores del boton blanco, si se presiona 2 lee negros
  digitalWrite(led, HIGH); // LED ON: inicio calibración

  // Presionar el boton para medir blanco (creo que se debe mantener presionado - no estoy seguro)
  while (digitalRead(boton) == 0) {}
  delay(10);
  for (int x = 0; x < 8; x++) {
    int valor_prom = 0;
    for (int i = 0; i < 10; i++) valor_prom += analogRead(sensores[x]);
    valor_blanco[x] = valor_prom / 10;
  }

  while (digitalRead(boton) == 1)  {}
  delay(10);
  while (digitalRead(boton) == 0) {}// se usa en 0 por el pull up
  delay(10);
  for (int x = 0; x < 8; x++) {
    int valor_prom = 0;
    for (int i = 0; i < 10; i++) valor_prom += analogRead(sensores[x]);
    valor_negro[x] = valor_prom / 10;
  }

   // Calcular e imprimir umbrales
  for (int x = 0; x < 8; x++) {
    valor_umbrales[x] = (valor_blanco[x] + valor_negro[x]) / 2;
  }
  SerialBT.println("[CAL] UMBRALES calculados:");


  digitalWrite(led, LOW); // LED OFF: fin calibración

  while (digitalRead(boton) == LOW) {}
  delay(10);
}


void setup() {
  Serial.begin(115200);

  // declaracion de pines regleta
  for (int i = D1; i >= D8; i++){
    pinMode(i, OUTPUT);
  }

  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  pinMode(boton, INPUT_PULLUP);

  ledcSetup(ledChannel, frequency, resolution);
  ledcAttachPin(IN1A, ledChannel);

  ledcSetup(ledChannel1, frequency, resolution);
  ledcAttachPin(IN1B, ledChannel1);

  ledcSetup(ledChannel2, frequency, resolution);
  ledcAttachPin(IN2A, ledChannel2);

  ledcSetup(ledChannel3, frequency, resolution);
  ledcAttachPin(IN2B, ledChannel3);
 
  precaucion(5000);  
}

void loop() {

   if(SerialBT.available() >0) {
    char dato = SerialBT.read();
    switch (dato){
    case '1':
      kp += 0.05f;
      break;
    case '2':
      kd += 0.05f;
      break;
    case '3':
      VBASE += 10;
      SerialBT.print("la velocidad base ahora es:");
      SerialBT.println(VBASE);
      //VBASE = constrain(baseSpeed, 0, 255);
      break; 

    default:
      break;
    }

  }

}

