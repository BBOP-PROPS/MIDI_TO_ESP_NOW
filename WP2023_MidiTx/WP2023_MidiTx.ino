#include <MIDI.h>
#include <esp_now.h>
#include <WiFi.h>

// Adafruit makes a MIDI board that can work with the ESP32. https://www.adafruit.com/product/4740
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI); // Need to use different serial port for MIDI to avoid conflict with serial debug printing
// -----------------------------------------------------------------------------

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;


// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Do whatever you want when a note is pressed.
  midi_struct midiData;
  midiData.midiCommand = 0x90;
  midiData.channel = channel;
  midiData.pitch = pitch;
  midiData.velocity = velocity;
  esp_err_t result = esp_now_send(0, (uint8_t *) &midiData, sizeof(midiData));
  // TODO handle error case
  
  // Try to keep your callbacks short (no delays ect)
  // otherwise it would slow down the loop() and have a bad impact
  // on real-time performance.
  char buf[100];
  sprintf(buf, "NoteOn: Channel: %d, Pitch: %d, Velocity: %d", channel, pitch, velocity);
  Serial.println(buf);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
  midi_struct midiData;
  midiData.midiCommand = 0x80;
  midiData.channel = channel;
  midiData.pitch = pitch;
  midiData.velocity = velocity;
  esp_err_t result = esp_now_send(0, (uint8_t *) &midiData, sizeof(midiData));
  // TODO handle error case

  char buf[100];
  sprintf(buf, "NoteOff: Channel: %d, Pitch: %d, Velocity: %d", channel, pitch, velocity);
  Serial.println(buf);
}

// -----------------------------------------------------------------------------

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress1[] = {0x94, 0xE6, 0x86, 0x3D, 0x59, 0xB8}; // Mark's receiver
uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x03, 0xC3, 0xE0}; // Ring1
//uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x02, 0x82, 0x24}; // Not installed as of 1/19/2023
//78:21:84:9D:A6:10
//uint8_t broadcastAddress2[] = {0xFF, , , , , };
//uint8_t broadcastAddress3[] = {0xFF, , , , , };

typedef struct test_struct {
  int x;
  int y;
} test_struct;

test_struct test;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("Starting Up");
  esp_now_register_send_cb(OnDataSent);
  Serial.println("Registered MIDI");
  
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  // memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }
  // /// register third peer
  // memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  //}
  
    // Connect the handleNoteOn function to the library,
    // so it is called upon reception of a NoteOn.
    MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function

    // Do the same for NoteOffs
    MIDI.setHandleNoteOff(handleNoteOff);

    // Initiate MIDI communications, listen to all channels
    MIDI.begin(MIDI_CHANNEL_OMNI);
}
 
void loop() {
//  test.x = random(0,20);
//  test.y = random(0,20);
// 
//  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
//   
//  if (result == ESP_OK) {
//    Serial.println("Sent with success");
//  }
//  else {
//    Serial.println("Error sending the data");
//  }
//  delay(2000);
    // Call MIDI.read the fastest you can for real-time performance.
    MIDI.read();

    // There is no need to check if there are messages incoming
    // if they are bound to a Callback function.
    // The attached method will be called automatically
    // when the corresponding message has been received.
}    
