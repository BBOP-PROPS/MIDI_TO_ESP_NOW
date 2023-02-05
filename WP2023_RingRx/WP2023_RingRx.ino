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
#include <FastLED.h>

// This section should be common to both the receiver and transmitter
enum ledCommand_e : byte
{
  SOLID_COLOR = 1,
  CHUNKY,
};

typedef struct ledCommand_struct
{
  byte command;
  byte data[16];
};
// End common section

//#define TEST_AT_HOME

#define NUM_LEDS 300

#define PRACTICE_MODE_PIN 16
#define TEST_MODE_PIN 17

#define HIGH_BRIGHTNESS 255
#define LOW_BRIGHTNESS 10

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

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  if (len <= sizeof(ledCommand_struct))
  {
    memcpy(&ledCommand, incomingData, sizeof(ledCommand_struct));
    newCommandReceived = true;
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
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
//  pinMode(22, OUTPUT);
}
 
void loop() {
  EVERY_N_MILLISECONDS(2000) { checkToggleSwitch(); }
  if (displayMode == TEST)
  {
    testPattern();
  }

  if (newCommandReceived)
  {
    newCommandReceived = false;
    processNewCommand();
  }
  
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = ColorFromPalette(currentPalette, colorIndex[i]);
  }
  nblendPaletteTowardPalette(currentPalette, targetPalette, 80);
  EVERY_N_MILLISECONDS(20)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      colorIndex[i] += 2;
    }
  }
//  digitalWrite(22, HIGH);
  FastLED.show(); // 19 msec with V3.5 and 9 msec with 3.3.3
//  digitalWrite(22, LOW);
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
  switch(ledCommand.command)
  {
    case SOLID_COLOR:
    {
      Serial.println(ledCommand.data[0]);
      Serial.println(ledCommand.data[1]);
      Serial.println(ledCommand.data[2]);
      setCurrentPaletteRGB(ledCommand.data[0], ledCommand.data[1], ledCommand.data[2]);
    }
      break;
    case CHUNKY:
      setCurrentPalette(ledCommand.data, sizeof(ledCommand.data));
      break;
    default:
      Serial.println("processCommand: Unknown command");
      break;
  }
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
  targetPalette.loadDynamicGradientPalette(bytes);
}

void setCurrentPalette(byte* bytes, int len)
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
}
