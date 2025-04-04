#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "cafe";
const char* password = "xyzblades";
WiFiServer server(33300);  // Порт должен совпадать с портом в Python-коде



void setup() {
  Serial.begin(115200);
  pinMode(16, OUTPUT);  // open
  pinMode(17, OUTPUT);  // close
  bool stat = 0;

  // Подключаемся к WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("connected");
    while (client.connected()) {
      if (client.available()) {
        String message = client.readStringUntil('\n');

        if (message == "open") {
          digitalWrite(16, HIGH);
          delay(1000);
          bool stat = false;
          digitalWrite(16, LOW);
          Serial.println("open recieved");
          client.println("0");
        }

        if (message == "close") {
          digitalWrite(17, HIGH);
          Serial.println("close recieved");
          delay(1000);
          bool stat = false;
          digitalWrite(17, LOW);
          client.println("1");
        }


        
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}