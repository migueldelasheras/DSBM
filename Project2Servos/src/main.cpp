#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

//Configuración bluetooth

BluetoothSerial SerialBT;

uint8_t incomingX; 
uint8_t incomingY; 
int posicion;

//Configuración Servos

#define XPIN 18
#define YPIN 21

Servo servoX;
Servo servoY;

void BTConnect(){
  SerialBT.begin("ESP32Servos");
  while(!SerialBT.connected()) {
    Serial.println("Esperando conexión bluetooth...");
    delay(1000);
  }
  Serial.println("Conectado y preparado para leer");

}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  BTConnect();
  servoX.attach(XPIN);
  servoY.attach(YPIN);
  servoX.write(90);
  servoY.write(90);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (SerialBT.available())
  {

    incomingX = SerialBT.read();
    servoX.write(incomingX);
    Serial.print(incomingX);
    Serial.print("-");

    incomingY = SerialBT.read();
    servoY.write(incomingY);
    Serial.println(incomingY);

  }
  
  delay(200);

}