#include <Arduino.h>
#include <WiFi.h>


#define WIFI_SSID "GalaxyS23"
#define WIFI_PASS "123456789S23"

#define LED_BUILTIN 2





void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.println("Connecting to WiFi");
}

bool isConnected = false;

void loop() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {

      // print connected and print the SSID
      Serial.println("Connected to WiFi");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());


      isConnected = true;
      digitalWrite(LED_BUILTIN, HIGH);
    
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED) {
      // print disconnected
      Serial.println(".");
      isConnected = false;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(1000);
  }


}

