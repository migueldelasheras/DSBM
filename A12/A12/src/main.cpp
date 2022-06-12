#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <string>

#define LIGHT_SENSOR_PIN 32 // ESP32 pin GIOP32
#define TEMPERATURE_SENSOR_PIN 33 //ESP32 pin GPIO33

const int B = 4275;               // B value of the thermistor
const int R0 = 100000;            // R0 = 100k

const char* ssid = "AndroidApp"; 
const char* password = "12345678";

#define TOPIC "A12/NODOA"
#define BROKER_IP "192.168.43.174" //direccion IP del broker
#define BROKER_PORT 2883 //puerto en el que escucha el broker

WiFiClient espClient;
PubSubClient client(espClient);

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

void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);
  wifiConnect();
  mqttConnect();
}

void loop() {
  // reads the input on analog pin (value between 0 and 4095)
  
  int lightValue = analogRead(LIGHT_SENSOR_PIN);
  int a = analogRead(TEMPERATURE_SENSOR_PIN);

  float R = 4095/a-1.0;
  R = R0*R;
  float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15;

  lightValue=(lightValue*100)/4095;

  char chtemp[8];
  dtostrf(temperature,4,2,chtemp);

  char chlight[4];
  dtostrf(lightValue,3,0,chlight);

  char message[100] = "Temperature value = ";
  strcat(message,chtemp);
  strcat(message," C - Light percentage(%) = ");
  strcat(message, chlight);

  client.publish(TOPIC,message);

  delay(500);

  
}