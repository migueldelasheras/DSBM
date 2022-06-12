#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>

#define RXD2 16
#define TXD2 17 //Este no se va a utilizar

const char* ssid = "AndroidApp";
const char* password = "12345678";

#define TOPIC "esi/lab7"
#define BROKER_IP "192.168.43.174"
#define BROKER_PORT 2883

WiFiClient espClient;
PubSubClient client(espClient);

void wifiConnect()
{
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    
  }

}

void mqttConnect() {
  client.setServer(BROKER_IP, BROKER_PORT);
  while (!client.connected()) {
    

    if (client.connect("ESP32Client1")) {
      
    } else {

      delay(5000);  //* Wait 5 seconds before retrying

    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  wifiConnect();
  mqttConnect();  
}

void loop() {
  // put your main code here, to run repeatedly:

  char state=Serial2.read();
  
    if (state=='0')
    {
      client.publish(TOPIC,"Estado del semaforo: GREEN");
      //Serial.println("Green");
    }else if (state=='1')
    {
      client.publish(TOPIC,"Estado del semaforo: YELLOW");
      //Serial.println("Yellow");
    }else if(state=='2'){
      client.publish(TOPIC,"Estado del semaforo: RED");
      //Serial.println("Red");
    }
    
  
  
}