/**
 * ESP-NOW
 * 
 * Sender
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
// Mac address of the slave
uint8_t peer1[] = {0x80, 0x7D, 0x3A, 0x59, 0x24, 0xED};
typedef struct message {
   int red;
   int green;
   int blue;
};
struct message myMessage;
void onSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.println("Status:");
  Serial.println(sendStatus);
}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("ESP-Now Sender");
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register the peer
  Serial.println("Registering a peer");
  esp_now_add_peer(peer1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  Serial.println("Registering send callback function");
  esp_now_register_send_cb(onSent);
}
void loop() {
  myMessage.red = 10;
  myMessage.green = 80;
  myMessage.blue = 180;
  Serial.println("Send a new message");
  esp_now_send(NULL, (uint8_t *) &myMessage, sizeof(myMessage));
  delay(60000);
}