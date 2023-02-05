#include <MIDI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>

// This section should be common to both the receiver and transmitter
enum ledCommand_e : byte
{
  SOLID_COLOR = 1,
  CHUNKY,
};
typedef struct ledCommand_struct
{
  byte command; // TODO consider adding blend speed
  byte data[16]; // TODO try using a union
};
// End common section

// Adafruit makes a MIDI board that can work with the ESP32. https://www.adafruit.com/product/4740
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI); // Need to use different serial port for MIDI to avoid conflict with serial debug printing

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;

const ledCommand_struct neutralPalette = { CHUNKY, { 0, 0xE9, 0xE9, 0xA0, 81, 0x40, 0x40, 0x40, 165, 0xFF, 0xFF, 0xB0, 255, 0xE9, 0xE9, 0xA0 } };
const ledCommand_struct happyPalette = { CHUNKY, { 0, 0xFF, 0x99, 0x00, 130, 0xFF, 0xB0, 0x00, 220, 0x80, 0x4D, 0x00, 255, 0xFF, 0x99, 0x00 } };
const ledCommand_struct sadPalette = { CHUNKY, { 0, 0x00, 0x00, 0xFF, 81, 0x30, 0x30, 0xFA, 165, 0x00, 0x00, 0x93, 255, 0x00, 0x00, 0xFF } };
const ledCommand_struct angryPalette = { CHUNKY, { 0, 0xFF, 0x00, 0x00, 120, 0xFF, 0x20, 0x00, 125, 0xC0, 0x00, 0x00, 255, 0xFF, 0x00, 0x00 } };
const ledCommand_struct solidRed = { SOLID_COLOR, {0xFF, 0x00, 0x00} };

// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  bool broadcastCommand = true;
  ledCommand_struct ledCommand;
  switch(pitch)
  {
    case 48:
      ledCommand = neutralPalette;
      break;
    case 49:
      ledCommand = happyPalette;
      break;
    case 50:
      ledCommand = sadPalette;
      break;
    case 51:
      ledCommand = angryPalette;
      break;
    case 60:
      ledCommand.command = SOLID_COLOR;
      ledCommand.data[0] = 0xE9;
      ledCommand.data[1] = 0xE9;
      ledCommand.data[2] = 0xA0;
      break;
    case 61:
      ledCommand.command = SOLID_COLOR;
      ledCommand.data[0] = 0xFF;
      ledCommand.data[1] = 0x99;
      ledCommand.data[2] = 0;
      break;
    case 62:
      ledCommand.command = SOLID_COLOR;
      ledCommand.data[0] = 0;
      ledCommand.data[1] = 0;
      ledCommand.data[2] = 0xFF;
      break;
    case 63:
      ledCommand.command = SOLID_COLOR;
      ledCommand.data[0] = 0xFF;
      ledCommand.data[1] = 0;
      ledCommand.data[2] = 0;
      break;
    default:
      broadcastCommand = false;
      break;
  }
  Serial.println(ledCommand.data[0]);
  Serial.println(ledCommand.data[1]);
  Serial.println(ledCommand.data[2]);
  Serial.println(ledCommand.data[3]); 
  if (broadcastCommand)
  {
    esp_err_t result = esp_now_send(0, (uint8_t *) &ledCommand, sizeof(ledCommand));
    broadcastCommand = false;
  }
  // Do whatever you want when a note is pressed.
//  midi_struct midiData;
//  midiData.midiCommand = 0x90;
//  midiData.channel = channel;
//  midiData.pitch = pitch;
//  midiData.velocity = velocity;
//  esp_err_t result = esp_now_send(0, (uint8_t *) &midiData, sizeof(midiData));
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
#if 0
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
#endif
}

void handlePitchBend(byte channel, int bend)
{
  char buf[100];
  sprintf(buf, "Pitch Bend: Channel: %d, Bend: %d", channel, bend);
  Serial.println(buf);
}
uint8_t broadcastAddressx[] = {0x40, 0x22, 0xD8, 0x05, 0x52, 0x78};


void handleControlChange(byte channel, byte data1, byte data2)
{
  bool broadcastCommand = true;
  ledCommand_struct ledCommand;
  switch (data1)
  {
    case 7:
    {
      ledCommand.command = 1;
      CHSV hsv(data2 * 2, 255, 255);
      CRGB rgb;
      hsv2rgb_rainbow(hsv, rgb);
      ledCommand.data[0] = rgb.r;
      ledCommand.data[1] = rgb.g;
      ledCommand.data[2] = rgb.b;
    }
    break;
    default:
    broadcastCommand = false;
    break;
  }
  
  if (broadcastCommand)
  {
    esp_err_t result = esp_now_send(broadcastAddressx, (uint8_t *) &ledCommand, sizeof(ledCommand));
    broadcastCommand = false;
  }

  char buf[100];
  sprintf(buf, "Control Change: Channel: %d, data1: %d, data2: %d", channel, data1, data2);
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
uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x02, 0x82, 0x24}; // Ring A
uint8_t broadcastAddress2[] = {0x40, 0x22, 0xD8, 0x05, 0x52, 0x78}; // Ring B
uint8_t broadcastAddress3[] = {0x40, 0x22, 0xD8, 0x03, 0xC3, 0xE0}; // Ring C
uint8_t broadcastAddress4[] = {0x94, 0xE6, 0x86, 0x3D, 0x59, 0xB8}; // Mark's receiver

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

  delay(2000); // Give time for the power to settle down before starting the radio
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
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
   }
  // /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
   }
  
  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
//  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);
  // Do the same for NoteOffs
  MIDI.setHandleNoteOff(handleNoteOff);

  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);
}
 
void loop() {
  // Call MIDI.read the fastest you can for real-time performance.
  if ( MIDI.read())
  {
    int b1 = MIDI.getType();
    Serial.print("Type: ");
    Serial.print(b1);
  }

  // There is no need to check if there are messages incoming
  // if they are bound to a Callback function.
  // The attached method will be called automatically
  // when the corresponding message has been received.
}    
