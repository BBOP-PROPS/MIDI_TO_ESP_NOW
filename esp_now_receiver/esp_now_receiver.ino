/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    26
#define LED_COUNT  59
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;

//Structure example to receive data
//Must match the sender structure
typedef struct test_struct {
  int x;
  int y;
} test_struct;

//Create a struct_message called myData
test_struct myData;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  midi_struct midiData;
  if (len >= sizeof(midiData))
  {
    memcpy(&midiData, incomingData, sizeof(midiData));
    // Middle C is pitch 60 so put into center of LED strip and make sure it is on the strip
    int ledIndex = midiData.pitch - 30;
    ledIndex = max(ledIndex, 0);
    ledIndex = min(ledIndex, LED_COUNT);
    uint32_t color;
    if (midiData.midiCommand == 0x90) // NoteOn
    {
      color = strip.Color(  0,   0,   0, 255); // Pure white
    }
    else
    {
      color = strip.Color(  0,   0,   0,   0); // Off
    }
    strip.setPixelColor(ledIndex, color);         //  Set pixel's color (in RAM)
    strip.show();
  }

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("pitch: ");
  Serial.println(midiData.pitch);
  Serial.print("command: ");
  Serial.println(midiData.midiCommand);
  Serial.println();

//  memcpy(&myData, incomingData, sizeof(myData));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
//  Serial.print("x: ");
//  Serial.println(myData.x);
//  Serial.print("y: ");
//  Serial.println(myData.y);
//  Serial.println();
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
}
 
void loop() {
//  for(int i=0; i<strip.numPixels(); i++)
//  {
//    strip.setPixelColor(i, strip.Color(  0,   0,   0, 255));         //  Set pixel's color (in RAM)
//    strip.show();                          //  Update strip to match
//  }
}
