// Brownsburg High School Winter Percussion 2023 LED controller
// Mark Nierzwick
// Scott Dial
// ESP-now example from Rui Santos

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <MIDI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h> // Using version 3.3.3 of FastLED to overcome a bug in the newer versions of the library

//#define TEST_AT_HOME

//#define _DEBUG_
#if defined _DEBUG_
   char printBuf[100];
   #define debug_print(...) \
     sprintf(printBuf, __VA_ARGS__); \
     Serial.print(printBuf)
   #define debug_println(...) \
     sprintf(printBuf, __VA_ARGS__); \
     Serial.println(printBuf)
#else
   #define debug_print(...)
   #define debug_println(...)
#endif

// This section should be common to both the receiver and transmitter
enum ledCommand_e : byte
{
  SOLID_COLOR = 1,
  CHUNKY,
  FLASH,
  BRIGHTNESS
};

typedef struct ledCommand_struct
{
  byte effect;
  uint16_t blendSpeedMSec;
  byte data[24]; // TODO try using a union
};
// End common section

enum sendState_e : byte
{
  SEND_IDLE = 0,
  READY_TO_SEND,
  RESPONSE_PENDING
};

typedef struct sendMsg_struct
{
  sendState_e sendState;
  uint32_t sendTime;
  uint8_t retries;
  ledCommand_struct ledCommand;
};

#define DEFAULT_BLEND_SPEED 2000

#define RINGA 0
#define RINGB 1
#define RINGC 2

#ifdef TEST_AT_HOME
#define NUM_RECEIVERS 2
const uint8_t sendAddress[NUM_RECEIVERS][6] = {
  {0x94, 0xE6, 0x86, 0x3B, 0x5E, 0x3C}, // Mark's receiver
  {0x94, 0xE6, 0x86, 0x3D, 0x59, 0xB8}  // Mark's receiver
};
#else
#define NUM_RECEIVERS 3
const uint8_t sendAddress[NUM_RECEIVERS][6] = {
  {0x40, 0x22, 0xD8, 0x02, 0x82, 0x24}, // Ring A
  {0x40, 0x22, 0xD8, 0x05, 0x52, 0x78}, // Ring B
  {0x40, 0x22, 0xD8, 0x03, 0xC3, 0xE0}  // Ring C
};
#endif
sendMsg_struct sendMsg[NUM_RECEIVERS];

MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI); // Need to use different serial port for MIDI to avoid conflict with serial debug printing

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;

//const ledCommand_struct introPalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0xFF, 0x99, 0x00, 130, 0x50, 0x30, 0x00, 180, 0x80, 0x4D, 0x00, 255, 0xFF, 0x99, 0x00 } };
const ledCommand_struct goldenHourPalette = { CHUNKY, 100, { 0, 0xFF, 0x00, 0x00, 35, 0xFF, 0x3C, 0x48, 61, 0xFF, 0x30, 0x30, 105, 0xFC, 0x05, 0x30, 150, 0xFF, 0x00, 0x00, 255, 0xFF, 0x00, 0x00 } };
//const ledCommand_struct lightBluePalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0x20, 0x20, 0x80, 81, 0x30, 0x30, 0x80, 165, 0x10, 0x10, 0x40, 255, 0x20, 0x20, 0x80 } };
const ledCommand_struct darkBluePalette = { CHUNKY, 100, { 0, 0x00, 0x00, 0xFF, 81, 0x00, 0x00, 0x80, 135, 0x00, 0x00, 0x20, 165, 0x00, 0x00, 0xFF, 255, 0x00, 0x00, 0xFF } };
const ledCommand_struct greenPalette = { CHUNKY, 100, { 0, 0x00, 0xFF, 0x00, 81, 0x30, 0x80, 0x00, 135, 0x10, 0x20, 0x00, 165, 0x00, 0xFF, 0x00, 255, 0x00, 0xFF, 0x00 } };

const ledCommand_struct neutralPalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0xE9, 0xE9, 0xA0, 81, 0x40, 0x40, 0x40, 165, 0xFF, 0xFF, 0xB0, 255, 0xE9, 0xE9, 0xA0 } };
const ledCommand_struct happyPalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0xFF, 0x99, 0x00, 130, 0xFF, 0xB0, 0x00, 220, 0x80, 0x4D, 0x00, 255, 0xFF, 0x99, 0x00 } };
const ledCommand_struct sadPalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0x00, 0x00, 0xFF, 81, 0x30, 0x30, 0xFA, 165, 0x00, 0x00, 0x93, 255, 0x00, 0x00, 0xFF } };
const ledCommand_struct angryPalette = { CHUNKY, DEFAULT_BLEND_SPEED, { 0, 0xFF, 0x00, 0x00, 120, 0xFF, 0x20, 0x00, 125, 0xC0, 0x00, 0x00, 255, 0xFF, 0x00, 0x00 } };

// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks
// Try not to do too much processing or add delays in callbacks
void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  bool broadcastCommand = true;
  ledCommand_struct ledCommand;

  switch(pitch)
  {
    case 48: // off
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0;
      ledCommand.data[1] = 0;
      ledCommand.data[2] = 0;
      break;
    case 49: // warm white
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 5000;
      ledCommand.data[0] = 0xFF;
      ledCommand.data[1] = 0xFF;
      ledCommand.data[2] = 0xE6;
      break;
    case 50: // red
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0xFF;
      ledCommand.data[1] = 0x00;
      ledCommand.data[2] = 0x00;
      break;
    case 51: // magenta
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0xFF;
      ledCommand.data[1] = 0x00;
      ledCommand.data[2] = 0xA0;
      break;
    case 52:
      ledCommand = goldenHourPalette;
      break;
    case 53: // pale blue
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 3500;
      ledCommand.data[0] = 0x10;
      ledCommand.data[1] = 0x10;
      ledCommand.data[2] = 0x60;
      break;
    case 54: // A dark blue B, C pale blue
      broadcastCommand = false;
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0x00;
      ledCommand.data[1] = 0x00;
      ledCommand.data[2] = 0xFF;
      RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
      ledCommand.data[0] = 0x10;
      ledCommand.data[1] = 0x10;
      ledCommand.data[2] = 0x60;
      RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
      RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
      break;
    case 55: // C dark blue A, B pale blue
      broadcastCommand = false;
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0x00;
      ledCommand.data[1] = 0x00;
      ledCommand.data[2] = 0xFF;
      RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
      ledCommand.data[0] = 0x10;
      ledCommand.data[1] = 0x10;
      ledCommand.data[2] = 0x60;
      RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
      RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
      break;
    case 56: // B dark blue A, C pale blue
      broadcastCommand = false;
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0x00;
      ledCommand.data[1] = 0x00;
      ledCommand.data[2] = 0xFF;
      RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
      ledCommand.data[0] = 0x10;
      ledCommand.data[1] = 0x10;
      ledCommand.data[2] = 0x60;
      RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
      RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
      break;
    case 57: // dark blue twinkle
      ledCommand = darkBluePalette;
      break;
    case 58: // Green
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 2600;
      ledCommand.data[0] = 0;
      ledCommand.data[1] = 0xA0;
      ledCommand.data[2] = 0;
      break;
    case 59: // green twinkle
      ledCommand = greenPalette;
      break;
    case 60: // warm white
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 3500;
      ledCommand.data[0] = 0xA0;
      ledCommand.data[1] = 0xA0;
      ledCommand.data[2] = 0x20;
      break;
    case 61: // off
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 3500;
      ledCommand.data[0] = 0;
      ledCommand.data[1] = 0;
      ledCommand.data[2] = 0;
      break;
    
    case 72: // Turns off
      ledCommand.effect = SOLID_COLOR;
      ledCommand.blendSpeedMSec = 100;
      ledCommand.data[0] = 0;
      ledCommand.data[1] = 0;
      ledCommand.data[2] = 0;
      break;
    case 73: // Flash
      ledCommand.effect = FLASH;
      ledCommand.blendSpeedMSec = 1000;
      ledCommand.data[0] = 0xE9;
      ledCommand.data[1] = 0xE9;
      ledCommand.data[2] = 0xA0;
      break;
    default:
      broadcastCommand = false;
      break;
  }

  if (broadcastCommand)
  {
    RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
    broadcastCommand = false;
  }

  debug_println("NoteOn: Channel: %d, Pitch: %d, Velocity: %d", channel, pitch, velocity);
}

#if 0
void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
  debug_println("NoteOff: Channel: %d, Pitch: %d, Velocity: %d", channel, pitch, velocity);
}
#endif
#if 0
void handlePitchBend(byte channel, int bend)
{
  debug_println("Pitch Bend: Channel: %d, Bend: %d", channel, bend);
}
#endif

void handleControlChange(byte channel, byte data1, byte data2)
{
  bool broadcastCommand = true;
  ledCommand_struct ledCommand;
  switch (data1)
  {
    case 7: // volume knob is 7 and is used for color selector
    {
      ledCommand.effect = BRIGHTNESS;
      ledCommand.blendSpeedMSec = 0;
      ledCommand.data[0] = map(data2,0,127,0,255);
      debug_println("Brightness: %d", ledCommand.data[0]);
    }
    break;
    default:
    broadcastCommand = false;
    break;
  }
  
  if (broadcastCommand)
  {
    RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
    broadcastCommand = false;
  }

  debug_println("Control Change: Channel: %d, data1: %d, data2: %d", channel, data1, data2);
}

esp_now_peer_info_t peerInfo;

// callback when data is sent
// Don't do too much processing in the callback. Retries will be sent in main loop.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  int8_t receiverNum = LookupReceiver(mac_addr);
  if ((receiverNum >= 0) && (sendMsg[receiverNum].sendState == RESPONSE_PENDING))
  {
    if (status != ESP_NOW_SEND_SUCCESS)
    {
      sendMsg[receiverNum].retries++;
      if (sendMsg[receiverNum].retries < 20)
      {
        sendMsg[receiverNum].sendState = READY_TO_SEND; // Send again
        debug_println("Retrying receiver %d", receiverNum);
      }
      else
      {
        sendMsg[receiverNum].sendState = SEND_IDLE; // Give up
        debug_print("Too many retries sending to ");
        debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      }
    }
    else
    {
      SetMonitorSegment(receiverNum);
      sendMsg[receiverNum].sendState = SEND_IDLE;
    }
  }
}

#define NUM_MON_LEDS 33
CRGB leds[NUM_MON_LEDS];
bool updateMonitor = false;

void setup() {
  Serial.begin(115200);
  
  FastLED.addLeds<WS2812B, 23, GRB>(leds, NUM_MON_LEDS);
  FastLED.setBrightness(50);
  fill_solid(leds, NUM_MON_LEDS, CRGB::Black); // Want to make sure all the LEDs are off before starting the radio
  FastLED.show();

  delay(2000); // Give time for the power to settle down before starting the radio
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("Starting Up");
  esp_now_register_send_cb(OnDataSent);
  Serial.println("Registered MIDI");

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  for (int i = 0; i < NUM_RECEIVERS; i++)
  {
    // register peer
    memcpy(peerInfo.peer_addr, sendAddress[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
//  MIDI.setHandleNoteOff(handleNoteOff);
//  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);

  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  MIDI.read();
//  if ( MIDI.read()) // printing MIDI messages only needed for debugging
//  {
//    int b1 = MIDI.getType();
//    Serial.print("Type: ");
//    Serial.print(b1);
//  }
#ifdef TEST_AT_HOME
while (Serial.available() == 0) {
  }
  int pitch = Serial.parseInt();
  if ((pitch > 1) && (pitch < 256))
  {
    handleNoteOn(1, (byte)pitch, 1);
  }
#endif
  EVERY_N_MILLISECONDS(10)
  {
    CheckMessagesToSend();
  }
  if (updateMonitor)
  {
    FastLED.show();
    updateMonitor = false;
  }
}

void RequestSendMsg(const uint8_t receiverNum, const uint8_t *data, size_t len)
{
  if (receiverNum < NUM_RECEIVERS)
  {
    sendMsg[receiverNum].sendTime = millis();
    memcpy(&sendMsg[receiverNum].ledCommand, data, len);
    sendMsg[receiverNum].retries = 0;
    sendMsg[receiverNum].sendState = READY_TO_SEND;
  }
}

void CheckMessagesToSend(void)
{
  esp_err_t result;
  for (int i = 0; i < NUM_RECEIVERS; i++)
  {
    if (sendMsg[i].sendState == READY_TO_SEND)
    {
      debug_println("Sending to receiver %d", i);
      result = esp_now_send(sendAddress[i], (uint8_t *) &sendMsg[i].ledCommand, sizeof(sendMsg[i].ledCommand));
      if (result == ESP_OK)
      {
        sendMsg[i].sendState = RESPONSE_PENDING;
      }
      else
      {
        if (millis() - sendMsg[i].sendTime > 500)
        {
          sendMsg[i].sendState = SEND_IDLE; // Give up
          Serial.print("Couldn't initiate sending a message to receiver ");
          Serial.println(i);
        }
      }
    }
  }
}

int8_t LookupReceiver(const uint8_t *mac_addr)
{
  int8_t receiverNum = -1; // Assume invalid until we find it
  for (int i = 0; i < NUM_RECEIVERS; i++)
  {
    if (memcmp(mac_addr, sendAddress[i], 6) == 0)
    {
      receiverNum = i;
      break;
    }
  }
  return receiverNum;
}

void SetMonitorSegment(uint8_t receiver)
{
  if (receiver < 3)
  {
    CRGB color;
    if (sendMsg[receiver].ledCommand.effect == SOLID_COLOR)
    {
      color = CRGB(sendMsg[receiver].ledCommand.data[0], sendMsg[receiver].ledCommand.data[1], sendMsg[receiver].ledCommand.data[2]);
    }
    else if (sendMsg[receiver].ledCommand.effect == CHUNKY)
    {
      color = CRGB(sendMsg[receiver].ledCommand.data[1], sendMsg[receiver].ledCommand.data[2], sendMsg[receiver].ledCommand.data[3]);
    }
    else
    {
      return; // Don't update the monitor for other types of messages
    }
    fill_solid(&leds[(NUM_MON_LEDS / 3) * receiver], NUM_MON_LEDS / 3, color);
    updateMonitor = true; // Tell main loop to update the led strip
  }
}
