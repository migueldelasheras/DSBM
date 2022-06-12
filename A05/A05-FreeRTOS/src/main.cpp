#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


//PINES que se van a utilizar
#define btn 18 //ESP32 pin GPIO18 (BOTÓN)
#define LIGHT_SENSOR_PIN 32 // ESP32 pin GIOP32
#define TEMPERATURE_SENSOR_PIN 33 //ESP32 pin GPIO33

//manejadores de tareas
TaskHandle_t senderHandle;
TaskHandle_t lightHandle;
TaskHandle_t temperatureHandle;

//colas para el envío de datos entre tareas
QueueHandle_t xQueueLight;
QueueHandle_t xQueueTemp;

const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k

//configuración de WI-FI y mqtt
const char* ssid = "AndroidApp"; 
const char* password = "12345678";

#define TOPIC "A12/NODOA" //TOPIC en el que publicará este nodo
#define BROKER_IP "192.168.43.174" //direccion IP del broker
#define BROKER_PORT 2883 //puerto en el que escucha el broker

WiFiClient espClient;
PubSubClient client(espClient);

void app_main();

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

//método manejador de interrupciones para suspender o reanudar las tareas cada vez que se pulsa el botón
void isr(){
  
  if(eTaskGetState(senderHandle)==eSuspended){
    Serial.println("Resuming tasks");
    vTaskResume(lightHandle);
    vTaskResume(temperatureHandle);
    vTaskResume(senderHandle);
    } else {
      Serial.println("Suspending tasks");
      vTaskSuspend(lightHandle);
      vTaskSuspend(temperatureHandle);
      vTaskSuspend(senderHandle);
    }
}

//Tarea para la medición del porcentaje de luz
void taskLight(void * parameters){

  portBASE_TYPE xStatus;
  //const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

  for (;;)
  {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    lightValue = (lightValue*100)/4095;
    Serial.print("light percentage: ");
    Serial.println(lightValue);
    xStatus = xQueueSendToBack(xQueueLight,&lightValue,0); //se añade el valor medido a la cola para que lo recoja el método encargado de publicar en mqtt
    if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola light");
    vTaskDelay(4000/ portTICK_PERIOD_MS); //espera de 4 segundos hasta la proxima medición
  }
  vTaskDelete(NULL); 
  
}

//Tarea para la medición de la temperatura
void taskTemperature(void * parameters){

  portBASE_TYPE xStatus;
  //const portTickType xTicksToWait = 500 / portTICK_RATE_MS;

  for(;;){
    int a= analogRead(TEMPERATURE_SENSOR_PIN);
    float R = 4095/a-1.0;
    R = R0*R;
    float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15;

    Serial.print("Temperatura: ");
    Serial.println(temperature);

    xStatus = xQueueSendToBack(xQueueTemp,&temperature,0);//Se añade el valor medido en la cola para que lo recoja el método encargado de publicar en mqtt
    if(xStatus !=pdPASS) Serial.println("No se ha podido añadir elemento a la cola temperature");
    vTaskDelay(3000/ portTICK_PERIOD_MS);//espera de 3 segundos hasta la proxima medición
  }
  vTaskDelete(NULL);
}

//Tarea para la publicación en mqtt de los valores medidos en las otras tareas
void taskSend(void * parameters){

  portBASE_TYPE xStatusL;
  portBASE_TYPE xStatusT;
  
  //const portTickType xTicksToWait = 10000 / portTICK_RATE_MS;
  
  //buffer con los ultimos valores obtenidos para saber si se ha actualizado el valor
  int bufferLight=0;
  float bufferTemp=0;

  float temperature=0;
  int lightValue=0;

  for(;;){
    
    //timeout = 0 para que no se espere en caso de que no haya nuevos datos en la cola
    xStatusL = xQueueReceive( xQueueLight, &bufferLight, 0 );
    xStatusT = xQueueReceive( xQueueTemp, &bufferTemp, 0 );

    char chtemp[8];
    dtostrf(temperature,4,2,chtemp);
    char chlight[4];
    dtostrf(lightValue,3,0,chlight);

    char message[100] = "Temperature value = ";
    strcat(message,chtemp);
    strcat(message," C - Light percentage(%) = ");
    strcat(message, chlight);

    //Si no es pdPASS no se han actualizado los datos de los sensores
    if (xStatusT == pdPASS)
    {
      strcat(message," Temperature updated ");
      temperature = bufferTemp;
    }
    if (xStatusL == pdPASS)
    {
      strcat(message," Light updated ");
      lightValue = bufferLight;
    }
    
    client.publish(TOPIC,message);
    vTaskDelay(2000/portTICK_PERIOD_MS); //espera de 2 segundos hasta la próxima publicación de datos
  }
  vTaskDelete(NULL);
}

void setup() {

  Serial.begin(9600);
  pinMode(btn,INPUT_PULLDOWN);
  attachInterrupt(btn,isr,RISING);

  wifiConnect();
  mqttConnect();
  app_main();

}

//método utilizado para la creacion de las tareas y las colas
void app_main(){

  //Creamos las colas con una capacidad máxima de 5
  xQueueLight = xQueueCreate( 1, sizeof( int ) );
  xQueueTemp = xQueueCreate( 1, sizeof( float ) );

  //Si no ha habido problemas en la creación de las colas creamos las tareas
  if (xQueueLight !=NULL && xQueueTemp!=NULL)
  {
    xTaskCreate(taskLight,"task light",1000,NULL,1,&lightHandle);
    xTaskCreate(taskTemperature,"task temperature",1000,NULL,1,&temperatureHandle);
    
    /*En esta tarea es necesario asignar más cantidad de memoria porque el uso del WI-FI puede provocar excepciones de stack overflow 
    si no se le asigna memoria suficiente*/
    xTaskCreate(taskSend,"task Send",3000,NULL,1,&senderHandle);
  }

}

//Con esta implementación de freeRTOS no es necesario utilzar el método loop
void loop() {
  // put your main code here, to run repeatedly:
  
}