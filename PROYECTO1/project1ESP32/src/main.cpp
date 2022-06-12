#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <string>
#include <BluetoothSerial.h>
#include <bits/stdc++.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define CONFIG_SW_COEXIST_ENABLE 1

//configuración de WI-FI y mqtt

const char* ssid = "Livebox6-FF88"; 
const char* password = "4SdtcxRd2ESE";

#define TOPICTEMP "esi/room1/temp"
#define TOPICLUM "esi/room1/lum"
#define TOPICWIND "esi/room1/wind"
#define TOPICSOUND "esi/room1/sound"
#define TOPICRAIN "esi/room1/rain"
#define BROKER_IP "192.168.1.138"
#define BROKER_PORT 2883 

WiFiClient espClient;
PubSubClient client(espClient);

//configuración de bluetooth

uint8_t address[6]  = {0xFF, 0xDB, 0x01, 0x00, 0x81, 0x82};
String name = "BT07";
char *pin = "1234";
bool connected;

BluetoothSerial SerialBT;

//Configuración freeRTOS

QueueHandle_t xQueueWind;
QueueHandle_t xQueueTemp;
QueueHandle_t xQueueLight;
QueueHandle_t xQueueSound;
QueueHandle_t xQueueRain;


//Declaración de funciones

void app_main();

//Método para la conexión WI-FI
void wifiConnect()
{
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//Método para la conexión mqtt
void mqttConnect() {
  client.setServer(BROKER_IP, BROKER_PORT);
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");

    if (client.connect("ESP32Client1")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");

      delay(5000);  //* Wait 5 seconds before retrying
    }
  }
}

//Método para la conexión Bluetooth
void BTConnect(){
  SerialBT.begin("ESP32",true);
  SerialBT.setPin(pin);
  Serial.println("The device started in master mode, make sure remote BT device is on!"); 

  SerialBT.enableSSP();
  
  do {
      Serial.println("Trying to connect..."); 
      delay(500);
      connected = SerialBT.connect(address);
    } while (SerialBT.hasClient() == 0 && !connected);

  Serial.println("Connected Succesfully!");

}

//Tarea para recibir los datos de la STM32 mediante Bluetooth
void vTaskReceive(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
  String recibido = "";
  for(;;){
    
    while (SerialBT.available())
    {
      Serial.println("Recibiendo nuevos datos...");
      recibido = SerialBT.readString();
      Serial.println(recibido);
      
      char datos[20];
      strcpy(datos, recibido.c_str());

      char *token = strtok(datos,","); //dato viento
      xStatus = xQueueSendToBack(xQueueWind,token,xTicksToWait);
      if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola wind");

      token = strtok(NULL,","); //dato temperatura
      xStatus = xQueueSendToBack(xQueueTemp,token,xTicksToWait);
      if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola temperature");

      token = strtok(NULL,","); //dato luminosidad
      xStatus = xQueueSendToBack(xQueueLight,token,xTicksToWait);
      if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola light");

      token = strtok(NULL,","); //dato sonido
      xStatus = xQueueSendToBack(xQueueSound,token,xTicksToWait);
      if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola sound");

      token = strtok(NULL,","); //dato lluvia
      char llueve = '0';
      if(token[0] == '0') xStatus = xQueueSendToBack(xQueueRain,&llueve,0);
      else {
        llueve = '1';
        xStatus = xQueueSendToBack(xQueueRain,&llueve,xTicksToWait);
      }
      if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola rain");  
      vTaskDelay(5000/portTICK_RATE_MS); 
      
    }
  }
}

void vTaskWind(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
  char buffer[8];
  char envio[30];
  for(;;){
    xStatus = xQueueReceive(xQueueWind, buffer,xTicksToWait);
    if (xStatus == pdPASS)
    {
      Serial.println(buffer);
      sprintf(envio,"{\"wind\":%s}",buffer);
      client.publish(TOPICWIND,envio);
      vTaskDelay(4000/portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void vTaskTemp(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
  char buffer[8];
  char envio[30];
  for(;;){
    xStatus = xQueueReceive(xQueueTemp, buffer,xTicksToWait);
    if (xStatus == pdPASS)
    {
      Serial.println(buffer);
      sprintf(envio,"{\"temperature\":%s}",buffer);
      client.publish(TOPICTEMP,envio);
      vTaskDelay(4000/portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void vTaskLight(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
  char buffer[8];
  char envio[30];
  for(;;){
    xStatus = xQueueReceive(xQueueLight, buffer,xTicksToWait);
    if (xStatus == pdPASS)
    {
      Serial.println(buffer);
      sprintf(envio,"{\"luminity\":%s}",buffer);
      client.publish(TOPICLUM,envio);
      vTaskDelay(4000/portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void vTaskSound(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
  char buffer[8];
  char envio[30];
  for(;;){
    xStatus = xQueueReceive(xQueueSound, buffer,xTicksToWait);
    if (xStatus == pdPASS)
    {
      Serial.println(buffer);
      sprintf(envio,"{\"sound\":%s}",buffer);
      client.publish(TOPICSOUND,envio);
      vTaskDelay(4000/portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void vTaskRain(void *parameters){

  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 500 / portTICK_RATE_MS;
  char buffer[8];
  char envio[30];
  for(;;){
    xStatus = xQueueReceive(xQueueRain, buffer,xTicksToWait);
    if (xStatus == pdPASS)
    {
      Serial.println(buffer);
      sprintf(envio,"{\"rain\":%s}",buffer);
      client.publish(TOPICRAIN,envio);
      vTaskDelay(4000/portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void setup(){
  Serial.begin(115200);
  BTConnect();
  wifiConnect();
  mqttConnect();
  app_main();

}

void app_main(){

  //Creación de las colas

  xQueueWind = xQueueCreate( 1, sizeof(int));
  xQueueTemp = xQueueCreate( 1, sizeof(int));
  xQueueLight = xQueueCreate( 1, sizeof(int));
  xQueueSound = xQueueCreate( 1, sizeof(int));
  xQueueRain = xQueueCreate( 1, sizeof(char));

  //Creación de las tareas

  Serial.println("Intentando crear tareas");

  if (xQueueWind !=NULL && xQueueTemp !=NULL && xQueueLight !=NULL && xQueueSound !=NULL && xQueueRain !=NULL )
  {
    Serial.println("CREANDO TAREAS");
    xTaskCreatePinnedToCore(vTaskReceive,"Task receive", 2000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskWind,"Task wind", 2000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskTemp,"Task temp", 2000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskLight,"Task light", 2000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSound,"Task sound", 2000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskRain,"Task rain", 2000, NULL, 2, NULL, 0);

  }

}

void loop() {
  
}