#include "DHT.h"
#include <WiFi.h>
#include <HTTPClient.h>

#define DHTPIN 22 // original  14
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

const int numSensores = 8;
String nombreSensor[numSensores] = {"TE", "HU", "US", "US", "FR", "RP", "IR", "IN"};
int idSensor[numSensores] = {01, 01, 02, 03, 04, 05, 06, 07};
String estacion[] = {"01"};

const int ldrPin = 34;
int pinTriggerUS1 = 12; // los originales son 23     // ESTE ES DEL DE ADELANTE
int pinEchoUS1 = 13; // original -> 21               // ESTE ES DEL DE ADELANTE
int pinTriggerUS2 = 23 ; // original -> 12           // ESTE ES DEL DE ATRÁS
int pinEchoUS2 = 21 ; // original -> 13              // ESTE ES DEL DE ATRÁS
const int sensorPin = 18; // pin inclinación 
const int sensorPinRPM = 19;
const int sensorPinInfrarrojo = 5;
int Rojo = 26;
int Amarillo = 25;
int focoA = 32;
int focoB = 33;
int focoC = 27; // 35

long duration, distance, duration1, distance1;
unsigned long tiempoAnterior = 0;
volatile int contadorPulsos = 0;
float rpm;
int umbral = 500;
int vInfrarrojo = 0;
int datoRecibido = 0;

void setup() {
  Serial.begin(115200);

  pinMode(Amarillo, OUTPUT);
  pinMode(Rojo, OUTPUT);
  pinMode(focoA, OUTPUT);
  pinMode(focoB, OUTPUT);
  pinMode(focoC, OUTPUT);
  pinMode(pinTriggerUS1, OUTPUT);
  pinMode(pinEchoUS1, INPUT);
  pinMode(pinTriggerUS2, OUTPUT);
  pinMode(pinEchoUS2, INPUT);
  pinMode(sensorPin, INPUT);
  pinMode(sensorPinRPM, INPUT);
  pinMode(ldrPin, INPUT);
  pinMode(sensorPinInfrarrojo, INPUT);

  attachInterrupt(digitalPinToInterrupt(sensorPinRPM), contarPulsos, RISING);
  tiempoAnterior = millis();
  dht.begin();
}

void loop() {
  if (Serial.available() > 0)
  {
    datoRecibido = Serial.parseInt();
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  String disp, disp1;
  int sensorValue;
  String ver, hor;
  float percentage;

  digitalWrite(pinTriggerUS1, LOW);
  delayMicroseconds(10);
  digitalWrite(pinTriggerUS1, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTriggerUS1, LOW);

  duration = pulseIn(pinEchoUS1, HIGH);
  distance = 17 * duration / 1000;
  disp = String(distance);

  digitalWrite(pinTriggerUS2, LOW);
  delayMicroseconds(10);
  digitalWrite(pinTriggerUS2, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTriggerUS2, LOW);

  duration1 = pulseIn(pinEchoUS2, HIGH);
  distance1 = 17 * duration1 / 1000;
  disp1 = String(distance1);

  if (distance1 > 21) {
    digitalWrite(Amarillo, LOW);
    digitalWrite(Rojo, LOW);
  } else if (distance1 <= 20 && distance1 > 11) {
    digitalWrite(Amarillo, HIGH);
    digitalWrite(Rojo, LOW);
  } else if (distance1 <= 10) {
    digitalWrite(Amarillo, LOW);
    digitalWrite(Rojo, HIGH);
  }

  sensorValue = digitalRead(sensorPin);
  ver = String("Inclinado");
  hor = String("Horizontal");
  String obstaculo;
  int valorLDR = analogRead(ldrPin);
  const int ldrMin = 0;
  const int ldrMax = 4095;
  int porcentajeLDR = map(valorLDR, ldrMin, ldrMax, 0, 100);
  int status;

  if (datoRecibido == 1) {
    status = 1;
    digitalWrite(focoA, HIGH);
    digitalWrite(focoB, HIGH);
    digitalWrite(focoC, HIGH);
  } else {
    if (porcentajeLDR < 20) 
    {
      status= 0;
      digitalWrite(focoA, LOW);
      digitalWrite(focoB, LOW); //LOW
      digitalWrite(focoC, LOW);
    } else
   {
      status = 1;
      digitalWrite(focoA, HIGH);
      digitalWrite(focoB, HIGH);
      digitalWrite(focoC, HIGH);
    }
  }

  unsigned long tiempoActual = millis();
  unsigned long tiempoTranscurrido = tiempoActual - tiempoAnterior;

  if (tiempoTranscurrido >= 1000) {
    detachInterrupt(digitalPinToInterrupt(sensorPinRPM));
    rpm = (contadorPulsos * 60.0) / (tiempoTranscurrido / 1000.0);
    contadorPulsos = 0;
    tiempoAnterior = tiempoActual;
    attachInterrupt(digitalPinToInterrupt(sensorPinRPM), contarPulsos, RISING);
  }

  vInfrarrojo = digitalRead(sensorPinInfrarrojo);
  if (vInfrarrojo == LOW) {
    obstaculo = String("Obstaculo detectado");
  } else {
    obstaculo = String("Libre :D");
  }
  Serial.println();

  for (int i = 0; i < numSensores; i++) {
    int sensorCode = 0;
    if (nombreSensor[i] == "TE") {
      sensorCode = 1;
    } else if (nombreSensor[i] == "HU") {
      sensorCode = 2;
    } else if (nombreSensor[i] == "US") {
      if (idSensor[i] == 2) {
        sensorCode = 3;
      } else {
        sensorCode = 4;
      }
    }  else if (nombreSensor[i] == "FR") {
      sensorCode = 5;
    } else if (nombreSensor[i] == "RP") {
      sensorCode = 6;
    } else if (nombreSensor[i] == "IR") {
      sensorCode = 7;
      } else if (nombreSensor[i] == "IN") {
      sensorCode = 8;
    }

    switch (sensorCode) {
      case 1:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (t)); // SENSOR TEMPERATURA
        break;
      case 2:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (h)); // SENSOR HUMEDAD
        break;
      case 3:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (disp)); // SENSOR DISTANCIA 1
        break;
      case 4:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (disp1)); // SENSOR DISTANCIA 2, EL DE ATRÁS CON LOS LEDS
        break;
      case 5:
        if (datoRecibido == 1)
        {
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (porcentajeLDR)); // SENSOR FOTORESISTENCIA CON EL FOCO ENCENDIDO
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + status);
        } else if (datoRecibido == 0)
        {
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (porcentajeLDR));  // SENSOR FOTORESISTENCIA CON EL FOCO ENCENDIDO
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + status);
        }
        break;
      case 6:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (rpm)); // SENSOR RPM
        break;
      case 7:
        Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (obstaculo)); // SENSOR OBSTACULOS (INFRAROJO)
        break;
      case 8:
        if (sensorValue == HIGH) {
          Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (ver)); // SENSOR INCLINACION
        } else {
          Serial.println(nombreSensor[i] + "-" + estacion[0] + "-" + idSensor[i] + "-" + (hor)); // SENSOR INCLINACION
        }
        break;
      default:
        Serial.println("Caso no manejado");
        break;
    }
  }
  delay(6000);
}

void contarPulsos() {
  contadorPulsos++;
}