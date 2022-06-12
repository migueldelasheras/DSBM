#include <Arduino.h>
#include <BluetoothSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//configuración bluetooth

BluetoothSerial SerialBT;
String name = "ESP32Servos";
bool connected;

//Configuracion Joystick

#define SWPIN 14 //Pin del botón de cambio de velocidad
#define YPIN 12 //Pin del eje Y del joystick
#define XPIN 13 //Pin del eje X del joystick

//Configuración freeRTOS

QueueHandle_t xSemaforoX;
QueueHandle_t xSemaforoY;

//variables globales

#define MAX_ANGLE 180
#define MIN_ANGLE 0

int posicionX = 90; //posición del servoX
int posicionY = 90; //posición del servoY
int velocidad = 3; //cantidad de grados que aumentan o disminuyen en cada medicion

int lecturaX = 0; //lectura analógica del joystick (eje X)
int lecturaY = 0; //lectura analógica del joystick (eje Y)

uint8_t angleX = 0; //ángulo X calculado a partir del valor analógico obtenido (eje X)
uint8_t angleY = 0; //ángulo Y calculado a partir del valor analógico obtenido (eje Y)
long lastTime; //última vez que se produjo una interrupción de cambio de velocidad

//Método para la conexión bluetooth
void BTConnect(){
  SerialBT.begin("ESP32Joystick",true);
  Serial.println("The device started in master mode, make sure remote BT device is on!"); 

  SerialBT.enableSSP();
  
  do {
      Serial.println("Trying to connect..."); 
      delay(500);
      connected = SerialBT.connect(name);
    } while (SerialBT.hasClient() == 0 && !connected);

  Serial.println("Connected Succesfully!");

}

//rutina de interrupción para el cambio de velocidad
void isr(){
  //control para que no se produzcan varias interrupciones seguidas por problemas del botón
  if (millis()- lastTime > 200) 
  {
    lastTime = millis();
    if(velocidad == 3) velocidad = 10;
    else velocidad = 3;
  }
  
}

//Tarea para la lectura de la posición del eje X
void vTaskX(void *parameters){
  
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

  for(;;){

    lecturaX = analogRead(XPIN); //Lectura del valor analógico
    angleX = map(lecturaX,0,4095,0,180); //conversión del valor analógico en un ángulo 0-180

    xSemaphoreTake(xSemaforoX,xTicksToWait);

    if (angleX>=95)
    {
      posicionX += velocidad;
      if(posicionX > MAX_ANGLE) posicionX = 180;
    } 
    else if (angleX <=75)
    {
      posicionX -= velocidad;
      if(posicionX < MIN_ANGLE) posicionX = 0;
    }

    xSemaphoreGive(xSemaforoX);

    vTaskDelay(200/portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

//Tarea para la lectura de la posición del eje Y
void vTaskY(void *parameters){
  
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

  for(;;){

    lecturaY = analogRead(YPIN); //Lectura del valor analógico
    angleY = map(lecturaY,0,4095,0,180); //conversión del valor analógico en un ángulo 0-180

    xSemaphoreTake(xSemaforoY,xTicksToWait);

    if (angleY>=95)
    {
      posicionY += velocidad;
      if(posicionY > MAX_ANGLE) posicionY = 180;
    } 
    else if (angleY <=75)
    {
      posicionY -= velocidad;
      if(posicionY < MIN_ANGLE) posicionY = 0;
    }

    xSemaphoreGive(xSemaforoY);
    
    vTaskDelay(200/portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

//Tarea para enviar los datos a la ESP32Servos
void vTaskSend(void *parameters){
  
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

  for(;;){

    xSemaphoreTake(xSemaforoX, xTicksToWait);
    Serial.print("Eje X: ");
    Serial.println(posicionX);
    SerialBT.write(posicionX);
    xSemaphoreGive(xSemaforoX);

    xSemaphoreTake(xSemaforoY,xTicksToWait);
    Serial.print("Eje Y: ");
    Serial.println(posicionY);
    SerialBT.write(posicionY);
    xSemaphoreGive(xSemaforoY);

    vTaskDelay(200/portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

void app_main(){
  //Creación de Semáforos 
  xSemaforoX = xSemaphoreCreateMutex();
  xSemaforoY = xSemaphoreCreateMutex();

  //Creación de tareas
  if(xSemaforoX !=NULL && xSemaforoY !=NULL){
    xTaskCreatePinnedToCore(vTaskX, "Task eje X", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskY, "Task eje y", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSend, "Task Send", 1000, NULL, 1, NULL, 0);
  }
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  BTConnect();
  
  //Configuración de los pines

  pinMode(SWPIN,INPUT_PULLUP);
  pinMode(YPIN,INPUT);
  pinMode(XPIN,INPUT);
  attachInterrupt(SWPIN,isr,RISING);

  app_main();

}

void loop() {

}