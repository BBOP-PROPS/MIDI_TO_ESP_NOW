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

int8_t myReceiverNum = -1;
broadcastMsg_struct bcastMsg;
uint16_t lastSequenceNum = 0;

#define NUM_LEDS 600
#define FRAME_RATE_MSEC 25

#define PRACTICE_MODE_PIN 16
#define TEST_MODE_PIN 17

#define HIGH_BRIGHTNESS 255
#define LOW_BRIGHTNESS 10
#define TIMEOUT_PERIOD_MSEC 60 * 5 * 1000

enum mode_e
{
  SHOW = 0,
  PRACTICE,
  TEST
};
mode_e displayMode = PRACTICE;

uint8_t colorIndex[NUM_LEDS];
uint8_t whichPalette = 0;

DEFINE_GRADIENT_PALETTE( black_gp ) { 
    0,    0x00, 0x00, 0x00,
  255,    0x00, 0x00, 0x00,
};

CRGBPalette16 currentPalette(black_gp);
CRGBPalette16 targetPalette(black_gp);

// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

bool newCommandReceived = false;
ledCommand_struct ledCommand;
unsigned long lastCommandTime = 0;
byte activeEffect = 0;

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

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  esp_err_t result;
  if (len >= sizeof(broadcastMsg_struct))
  {
    memcpy(&bcastMsg, incomingData, sizeof(broadcastMsg_struct));
    debug_println("sequence Num %d, resend %d", bcastMsg.sequenceNum, bcastMsg.resendCtr);
    if ((bcastMsg.sequenceNum > lastSequenceNum) || // This prevents processing the same message twice
       ((bcastMsg.sequenceNum <= 1) && (lastSequenceNum > 1))) // This allows the transmitter to be reset or roll over
    {
      lastSequenceNum = bcastMsg.sequenceNum;
      memcpy(&ledCommand, &bcastMsg.ledCmd[myReceiverNum], sizeof(ledCommand_struct));
      newCommandReceived = true;
    }
    if (bcastMsg.resendCtr == 0) // if this is original message, then forward to peers
    {
      bcastMsg.resendCtr++;
      result = esp_now_send(0, (uint8_t *) &bcastMsg, sizeof(bcastMsg));
      if (result != ESP_OK)
      {
        Serial.println("Couldn't initiate forwarding the message ");
      }
    }
  }
  else
  {
    Serial.println("OnDataRecv: Packet too short");
  }
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  // Use pins 16 and 17 for toggle switch
  pinMode(PRACTICE_MODE_PIN, INPUT_PULLUP);
  pinMode(TEST_MODE_PIN, INPUT_PULLUP);
    
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
#ifdef TEST_AT_HOME
  FastLED.addLeds<WS2812B, 23, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 25, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 26, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 27, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 32, GRB>(leds, NUM_LEDS);
#else
  FastLED.addLeds<WS2812B, 23, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 25, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 26, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 27, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, 32, RGB>(leds, NUM_LEDS);
#endif
  FastLED.setBrightness(LOW_BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Want to make sure all the LEDs are off before starting the radio
  FastLED.show();

  uint8_t chunk = 10; // make sure length is a multiple of chunk or trap it later
  for (int i = 0; i < NUM_LEDS; i+=chunk) {
    uint8_t color = random8();
    for (int j = 0; j < chunk; j++)
    {
      colorIndex[i + j] = color;
    }
  }
  
  delay(1000); // Wait a bit for the power to settle before starting the radio

  //Set device as a Wi-Fi Station
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
  uint8_t mac_addr[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
  myReceiverNum = LookupReceiver(mac_addr);
  debug_println("%02x:%02x:%02x:%02x:%02x:%02x",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (myReceiverNum < 0)
  {
    Serial.println("This mac address isn't in the list!");
    esp_restart();
  }
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    esp_restart();
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Register the other two rings so we can forward messages
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  for (int i = 0; i < NUM_RECEIVERS; i++)
  {
    // register peer
    if (memcmp(mac_addr, sendAddress[i], 6) != 0)
    {
      memcpy(peerInfo.peer_addr, sendAddress[i], 6);
      if (esp_now_add_peer(&peerInfo) != ESP_OK)
      {
        Serial.println("Failed to add peer");
        esp_restart();
      }
    }
  }

//  pinMode(22, OUTPUT);
  powerOnDisplay();
  Serial.println("Finished initialization.");
}
 
void loop() {
  if (millis() - lastCommandTime > TIMEOUT_PERIOD_MSEC)
  {
    targetPalette = black_gp;
  }
  EVERY_N_MILLISECONDS(2000) { checkToggleSwitch(); }
  if (displayMode == TEST)
  {
    testPattern();
    lastCommandTime = millis(); // Don't timeout in test mode
  }

  if (newCommandReceived)
  {
    newCommandReceived = false;
    processNewCommand();
    lastCommandTime = millis();
  }
  
  EVERY_N_MILLISECONDS(FRAME_RATE_MSEC)
  {
    updateEffect();
//    digitalWrite(22, HIGH);
    FastLED.show(); // 18 msec with 600 LEDs and FastLED 3.3.3
//    digitalWrite(22, LOW);
  }
}

void updateEffect(void)
{
  updateBlend();
  switch(activeEffect)
  {
    case SOLID_COLOR:
      {
        for (int i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = ColorFromPalette(currentPalette, 0);
        }
      }
      break;
    case CHUNKY:
      {
        for (int i = 0; i < NUM_LEDS; i++)
        {
          colorIndex[i] += 2;
        }
        for (int i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = ColorFromPalette(currentPalette, colorIndex[i]);
        }
      }
      break;
    default:
      break;  
  }
}

float blendIncrement = 0.0F;
float blendCountF = 0.0F;
int blendCount = 255;

void startBlend(uint16_t blendTimeMSec)
{
  blendIncrement = 255.0F / ((float)blendTimeMSec / FRAME_RATE_MSEC);
  blendCountF = 0.0F;
  blendCount = 0;
}

void updateBlend(void)
{
  if (blendCount < 255)
  {
    float newBlendCountF = blendCountF + blendIncrement;
    int blends = (int)(newBlendCountF - blendCount);
    if (blends >= 50)
    {
      currentPalette = targetPalette;
      blendCount = 255;
    }
    else
    {
      for (int i = 0; i < blends; i++)
      {
        nblendPaletteTowardPalette(currentPalette, targetPalette);
      }
      blendCountF = newBlendCountF;
      blendCount += blends;
    }
  }
}

void checkToggleSwitch(void)
{
  mode_e tmpMode = SHOW;
  if (digitalRead(PRACTICE_MODE_PIN) == 0)
  {
    tmpMode = PRACTICE;
  }
  else if (digitalRead(TEST_MODE_PIN) == 0)
  {
    tmpMode = TEST;
  }
  if (tmpMode != displayMode)
  {
    displayMode = tmpMode;
    switch (displayMode)
    {
      case SHOW:
        FastLED.setBrightness(HIGH_BRIGHTNESS);
        break;
      case PRACTICE:
        FastLED.setBrightness(LOW_BRIGHTNESS);
        break;
      case TEST:
        FastLED.setBrightness(LOW_BRIGHTNESS);
        break;
      default: // This should never happen but it's good coding practice to have a default
        break;
    }
    FastLED.show();
  }
}

void processNewCommand(void)
{
  switch(ledCommand.effect)
  {
    case SOLID_COLOR:
    {
//      debug_println("Solid Color %d %d %d", ledCommand.data[0], ledCommand.data[1], ledCommand.data[2]);
      setTargetPaletteRGB(ledCommand.data[0], ledCommand.data[1], ledCommand.data[2]);
      activeEffect = ledCommand.effect;
      startBlend(ledCommand.blendSpeedMSec);
    }
      break;
    case CHUNKY:
      setTargetPalette(ledCommand.data, sizeof(ledCommand.data));
      activeEffect = ledCommand.effect;
      startBlend(ledCommand.blendSpeedMSec);
      break;
    case FLASH:
      setCurrentPaletteRGB(ledCommand.data[0], ledCommand.data[1], ledCommand.data[2]);
      startBlend(ledCommand.blendSpeedMSec);
      break;
    case BRIGHTNESS:
      FastLED.setBrightness(ledCommand.data[0]);
      FastLED.show();
      debug_print("Brightness: %d", ledCommand.data[0]);
      break;
    default:
      Serial.println("processCommand: Unknown command");
      break;
  }
}

void setTargetPaletteRGB(byte r, byte g, byte b)
{
  byte bytes[8];
  
  bytes[0] = 0;
  bytes[1] = r;
  bytes[2] = g;
  bytes[3] = b;
  bytes[4] = 255;
  bytes[5] = r;
  bytes[6] = g;
  bytes[7] = b;
  targetPalette.loadDynamicGradientPalette(bytes);
}

void setCurrentPaletteRGB(byte r, byte g, byte b)
{
  byte bytes[8];
  
  bytes[0] = 0;
  bytes[1] = r;
  bytes[2] = g;
  bytes[3] = b;
  bytes[4] = 255;
  bytes[5] = r;
  bytes[6] = g;
  bytes[7] = b;
  currentPalette.loadDynamicGradientPalette(bytes);
}

void setTargetPalette(byte* bytes, int len)
{
  if (len % 4 != 0)
  {
    Serial.println("malformed palette");
    return;
  }
  targetPalette.loadDynamicGradientPalette(bytes);
}

#define TEST_PATTERN_DELAY 1000
void testPattern(void)
{
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(TEST_PATTERN_DELAY); // Note that using delay() prevents FastLED from updating for the delay period. This is intenional here.
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(TEST_PATTERN_DELAY);
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(TEST_PATTERN_DELAY);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
}

void powerOnDisplay(void)
{
  if ((digitalRead(PRACTICE_MODE_PIN) == 1) && (digitalRead(TEST_MODE_PIN) == 1))
  {
    fill_solid(leds, NUM_LEDS, CRGB::Green);
  }
  else
  {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  }
  FastLED.show();
  delay(2000);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
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
