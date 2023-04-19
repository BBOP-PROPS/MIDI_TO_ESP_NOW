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
#include <esp_wifi.h>

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
  byte data[24];
};

typedef struct broadcastMsg_struct
{
  uint8_t resendCtr;
  uint16_t sequenceNum;
  ledCommand_struct ledCmd[3];
};

// End common section

//enum sendState_e : byte
//{
//  SEND_IDLE = 0,
//  READY_TO_SEND,
//  RESPONSE_PENDING
//};

broadcastMsg_struct bcastMsg;
bool readyToBroadcast = false;

//typedef struct sendMsg_struct
//{
//  sendState_e sendState;
//  uint32_t sendTime;
//  uint8_t retries;
//  ledCommand_struct ledCommand[3];
//};

#define RINGA 0
#define RINGB 1
#define RINGC 2

#ifdef TEST_AT_HOME
#define NUM_RECEIVERS 3
const uint8_t sendAddress[NUM_RECEIVERS][6] = {
  {0x94, 0xE6, 0x86, 0x3B, 0x5E, 0x3C}, // Mark's receiver
  {0x94, 0xE6, 0x86, 0x3D, 0x59, 0xB8}, // Mark's receiver
  {0xC8, 0xF0, 0x9E, 0xEC, 0x22, 0x24}  // Mark's receiver dev kit
};
#else
#define NUM_RECEIVERS 3
const uint8_t sendAddress[NUM_RECEIVERS][6] = {
  {0x40, 0x22, 0xD8, 0x02, 0x82, 0x24}, // Ring A
  {0x40, 0x22, 0xD8, 0x05, 0x52, 0x78}, // Ring B
  {0x40, 0x22, 0xD8, 0x03, 0xC3, 0xE0}  // Ring C
};
#endif
//sendMsg_struct sendMsg;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI); // Need to use different serial port for MIDI to avoid conflict with serial debug printing

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;

const ledCommand_struct goldenHourPalette = { CHUNKY, 100, { 0, 0xFF, 0x00, 0x00, 35, 0xFF, 0x3C, 0x48, 61, 0xFF, 0x30, 0x30, 105, 0xFC, 0x05, 0x30, 150, 0xFF, 0x00, 0x00, 255, 0xFF, 0x00, 0x00 } };
const ledCommand_struct darkBluePalette =   { CHUNKY, 100, { 0, 0x00, 0x00, 0xFF, 81, 0x00, 0x00, 0x80, 135, 0x00, 0x00, 0x20, 165, 0x00, 0x00, 0xFF, 255, 0x00, 0x00, 0xFF } };
const ledCommand_struct greenPalette =      { CHUNKY, 100, { 0, 0x00, 0xFF, 0x00, 81, 0x30, 0x80, 0x00, 135, 0x10, 0x20, 0x00, 165, 0x00, 0xFF, 0x00, 255, 0x00, 0xFF, 0x00 } };

const ledCommand_struct whiteTwinkle =   { CHUNKY, 1875, { 0, 0xE9, 0xE9, 0xA0, 54, 0x51, 0x51, 0x39, 89, 0xFF, 0xFF, 0xB0, 150, 0xFF, 0xFF, 0xB0, 255, 0xE9, 0xE9, 0xA0 } };
const ledCommand_struct redTwinkle =     { CHUNKY, 1875, { 0, 0xFF, 0x00, 0x00, 54, 0xa0, 0x00, 0x00, 71, 0xff, 0x40, 0x20, 89, 0x9e, 0x20, 0x00, 255, 0xFF, 0x00, 0x00 } };
const ledCommand_struct magentaTwinkle = { CHUNKY, 1875, { 0, 0xFF, 0x00, 0x60, 54, 0xF2, 0x00, 0x62, 71, 0xff, 0xc3, 0xa3, 89, 0x9e, 0x00, 0x40, 255, 0xFF, 0x00, 0x60 } };
const ledCommand_struct purpleTwinkle =  { CHUNKY, 1875, { 0, 0x65, 0x00, 0x90, 54, 0xaa, 0x00, 0x80, 71, 0xf4, 0x40, 0x80, 89, 0x30, 0x00, 0x25, 255, 0x65, 0x00, 0x90 } };
const ledCommand_struct blueTwinkle =    { CHUNKY, 1875, { 0, 0x00, 0x00, 0xFF, 54, 0x00, 0x00, 0x80, 71, 0x00, 0x00, 0x20, 89, 0x00, 0x00, 0xFF, 255, 0x00, 0x00, 0xFF } };
const ledCommand_struct greenTwinkle =   { CHUNKY, 1875, { 0, 0x00, 0xFF, 0x00, 54, 0x30, 0x80, 0x00, 71, 0x10, 0x20, 0x00, 89, 0x00, 0xFF, 0x00, 255, 0x00, 0xFF, 0x00 } };
const ledCommand_struct yellowTwinkle =  { CHUNKY, 1875, { 0, 0xFF, 0x99, 0x00, 54, 0xFF, 0xB0, 0x00, 71, 0x80, 0x4D, 0x00, 89, 0xFF, 0x80, 0x00, 255, 0xFF, 0x99, 0x00 } };
const ledCommand_struct orangeTwinkle =  { CHUNKY, 1875, { 0, 0xf0, 0x40, 0x00, 54, 0x40, 0x20, 0x00, 71, 0xe0, 0x60, 0x00, 89, 0x60, 0x20, 0x00, 255, 0xf0, 0x40, 0x00 } };

// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks
// Try not to do too much processing or add delays in callbacks
void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  bool allSame = true;
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
      allSame = false;
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
      allSame = false;
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
      allSame = false;
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
      ledCommand = greenTwinkle;
      break;
    case 60: // warm white twinkle
      ledCommand = whiteTwinkle;
      break;
    case 61: // red twinkle
      ledCommand = redTwinkle;
      break;
    case 62: // magenta twinkle
      ledCommand = magentaTwinkle;
      break;
    case 63: // purple twinkle
      ledCommand = purpleTwinkle;
      break;
    case 64: // blue twinkle
      ledCommand = blueTwinkle;
      break;
    case 65: // green twinkle
      ledCommand = greenTwinkle;
      break;
    case 66: // yellow twinkle
      ledCommand = yellowTwinkle;
      break;
    case 67: // orange twinkle
      ledCommand = orangeTwinkle;
      break;
    case 68: // white twinkle
      ledCommand = whiteTwinkle;
      break;
    case 69: // Turns off
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
      allSame = false;
      break;
  }

  if (allSame)
  {
    RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
    RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
    allSame = false;
  }
  readyToBroadcast = true;
  
  debug_println("NoteOn: Channel: %d, Pitch: %d, Velocity: %d", channel, pitch, velocity);
}

bool sendNewBrightness = false;
uint8_t newBrightness = 255;

void handleControlChange(byte channel, byte data1, byte data2)
{
//  ledCommand_struct ledCommand;
  switch (data1)
  {
    case 7: // volume knob is 7 and is used for color selector
    {
//      ledCommand.effect = BRIGHTNESS;
//      ledCommand.blendSpeedMSec = 0;
//      ledCommand.data[0] = map(data2,0,127,0,255);
      newBrightness = map(data2,0,127,0,255);
      debug_println("Brightness: %d", newBrightness);
      sendNewBrightness = true;
    }
    break;
    default:
    break;
  }
  
//  RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
//  RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
//  RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
//  readyToBroadcast = true;

  debug_println("Control Change: Channel: %d, data1: %d, data2: %d", channel, data1, data2);
}

esp_now_peer_info_t peerInfo;

// callback when data is sent
// Don't do too much processing in the callback. Retries will be sent in main loop.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  int8_t receiverNum = LookupReceiver(mac_addr);
//  if ((receiverNum >= 0) && (sendMsg[receiverNum].sendState == RESPONSE_PENDING))
//  {
//    if (status != ESP_NOW_SEND_SUCCESS)
//    {
//      sendMsg[receiverNum].retries++;
//      if (sendMsg[receiverNum].retries < 20)
//      {
//        sendMsg[receiverNum].sendState = READY_TO_SEND; // Send again
//        debug_println("Retrying receiver %d", receiverNum);
//      }
//      else
//      {
//        sendMsg[receiverNum].sendState = SEND_IDLE; // Give up
//        debug_print("Too many retries sending to ");
//        debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
//               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//      }
//    }
//    else
//    {
//      sendMsg[receiverNum].sendState = SEND_IDLE;
//    }
//  }
  if (status != ESP_NOW_SEND_SUCCESS)
  {
    debug_print("Failure sending to ");
    debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  }
//  else
//  {
//    debug_print("Success sending to ");
//    debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
//           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  }
}

void setup() {
  Serial.begin(115200);

  bcastMsg.sequenceNum = 0;
  
  delay(2000); // Give time for the power to settle down before starting the radio
  WiFi.mode(WIFI_STA);
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("Primary channel before ");
  Serial.println(primary);
  if (esp_wifi_set_channel(13, second) != ESP_OK)
  {
    esp_restart();
  }
  esp_wifi_get_channel(&primary, &second);
  Serial.print("Primary channel after ");
  Serial.println(primary);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    esp_restart();
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
      esp_restart();
    }
  }

  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
  MIDI.setHandleControlChange(handleControlChange);

  // Initiate MIDI communications, listen to all channels
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.println("Finished initialization.");
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
//while (Serial.available() == 0) {
//  }
//  int pitch = Serial.parseInt();
//  if ((pitch > 1) && (pitch < 256))
//  {
//    handleNoteOn(1, (byte)pitch, 1);
//  }
#endif
  EVERY_N_MILLISECONDS(10)
  {
    CheckMessagesToSend();
  }
  EVERY_N_MILLISECONDS(100)
  {
    if (sendNewBrightness)
    {
      sendNewBrightness = false;
      SendBrightness();
    }
  }
//  EVERY_N_MILLISECONDS(2000)
//  {
//    static uint8_t pitch = 48;
//    handleNoteOn(1, (byte)pitch++, 1);
//    if (pitch > 70)
//      pitch = 48;
//  }
//  EVERY_N_MILLISECONDS(10000)
//  {
//    int counter = 0;
//    esp_err_t result = ESP_OK;
//    while (result == ESP_OK)
//    {
//      result = esp_now_send(0, (uint8_t *) &bcastMsg, sizeof(bcastMsg));
//      counter++;
//      if (result == ESP_OK)
//      {
//        readyToBroadcast = false;
//      }
//      else
//      {
//        Serial.println("Couldn't initiate broadcasting the message ");
//        Serial.println(result);
//      }
//    }
//    debug_println("Failed after %d sends", counter);
//  }
}

void RequestSendMsg(const uint8_t receiverNum, const uint8_t *data, size_t len)
{
  if (receiverNum < NUM_RECEIVERS)
  {
    memcpy(&bcastMsg.ledCmd[receiverNum], data, len);
  }
}

void CheckMessagesToSend(void)
{
  esp_err_t result;
  if (readyToBroadcast)
  {
    bcastMsg.resendCtr = 0; // This is always zero on the transmitter since it is initiating the message
    bcastMsg.sequenceNum++;
    result = esp_now_send(0, (uint8_t *) &bcastMsg, sizeof(bcastMsg));
    if (result == ESP_OK)
    {
      readyToBroadcast = false;
    }
    else
    {
      Serial.println("Couldn't initiate broadcasting the message ");
      Serial.println(result);
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

void SendBrightness(void)
{
  ledCommand_struct ledCommand;
  ledCommand.effect = BRIGHTNESS;
  ledCommand.blendSpeedMSec = 0;
  ledCommand.data[0] = newBrightness;
  RequestSendMsg(RINGA, (uint8_t *) &ledCommand, sizeof(ledCommand));
  RequestSendMsg(RINGB, (uint8_t *) &ledCommand, sizeof(ledCommand));
  RequestSendMsg(RINGC, (uint8_t *) &ledCommand, sizeof(ledCommand));
  readyToBroadcast = true;
}
