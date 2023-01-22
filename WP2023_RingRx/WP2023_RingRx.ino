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


#define NUM_LEDS 300

uint32_t colors[] = {
        CRGB::AliceBlue,
        CRGB::Amethyst,
        CRGB::AntiqueWhite,
        CRGB::Aqua,
        CRGB::Aquamarine,
        CRGB::Azure,
        CRGB::Beige,
        CRGB::Bisque,
        CRGB::Black,
        CRGB::BlanchedAlmond,
        CRGB::Blue,
        CRGB::BlueViolet,
        CRGB::Brown,
        CRGB::BurlyWood,
        CRGB::CadetBlue,
        CRGB::Chartreuse,
        CRGB::Chocolate,
        CRGB::Coral,
        CRGB::CornflowerBlue,
        CRGB::Cornsilk,
        CRGB::Crimson,
        CRGB::Cyan,
        CRGB::DarkBlue,
        CRGB::DarkCyan,
        CRGB::DarkGoldenrod,
        CRGB::DarkGray,
        CRGB::DarkGreen,
        CRGB::DarkKhaki,
        CRGB::DarkMagenta,
        CRGB::DarkOliveGreen,
        CRGB::DarkOrange,
        CRGB::DarkOrchid,
        CRGB::DarkRed,
        CRGB::DarkSalmon,
        CRGB::DarkSeaGreen,
        CRGB::DarkSlateBlue,
        CRGB::DarkSlateGray,
        CRGB::DarkTurquoise,
        CRGB::DarkViolet,
        CRGB::DeepPink,
        CRGB::DeepSkyBlue,
        CRGB::DimGray,
        CRGB::DodgerBlue,
        CRGB::FireBrick,
        CRGB::FloralWhite,
        CRGB::ForestGreen,
        CRGB::Fuchsia,
        CRGB::Gainsboro,
        CRGB::GhostWhite,
        CRGB::Gold,
        CRGB::Goldenrod,
        CRGB::Gray,
        CRGB::Green,
        CRGB::GreenYellow,
        CRGB::Honeydew,
        CRGB::HotPink,
        CRGB::IndianRed,
        CRGB::Indigo,
        CRGB::Ivory,
        CRGB::Khaki,
        CRGB::Lavender,
        CRGB::LavenderBlush,
        CRGB::LawnGreen,
        CRGB::LemonChiffon,
        CRGB::LightBlue,
        CRGB::LightCoral,
        CRGB::LightCyan,
        CRGB::LightGoldenrodYellow,
        CRGB::LightGreen,
        CRGB::LightGrey,
        CRGB::LightPink,
        CRGB::LightSalmon,
        CRGB::LightSeaGreen,
        CRGB::LightSkyBlue,
        CRGB::LightSlateGray,
        CRGB::LightSteelBlue,
        CRGB::LightYellow,
        CRGB::Lime,
        CRGB::LimeGreen,
        CRGB::Linen,
        CRGB::Magenta,
        CRGB::Maroon,
        CRGB::MediumAquamarine,
        CRGB::MediumBlue,
        CRGB::MediumOrchid,
        CRGB::MediumPurple,
        CRGB::MediumSeaGreen,
        CRGB::MediumSlateBlue,
        CRGB::MediumSpringGreen,
        CRGB::MediumTurquoise,
        CRGB::MediumVioletRed,
        CRGB::MidnightBlue,
        CRGB::MintCream,
        CRGB::MistyRose,
        CRGB::Moccasin,
        CRGB::NavajoWhite,
        CRGB::Navy,
        CRGB::OldLace,
        CRGB::Olive,
        CRGB::OliveDrab,
        CRGB::Orange,
        CRGB::OrangeRed,
        CRGB::Orchid,
        CRGB::PaleGoldenrod,
        CRGB::PaleGreen,
        CRGB::PaleTurquoise,
        CRGB::PaleVioletRed,
        CRGB::PapayaWhip,
        CRGB::PeachPuff,
        CRGB::Peru,
        CRGB::Pink,
        CRGB::Plaid,
        CRGB::Plum,
        CRGB::PowderBlue,
        CRGB::Purple,
        CRGB::Red,
        CRGB::RosyBrown,
        CRGB::RoyalBlue,
        CRGB::SaddleBrown,
        CRGB::Salmon,
        CRGB::SandyBrown,
        CRGB::SeaGreen,
        CRGB::Seashell,
        CRGB::Sienna,
        CRGB::Silver,
        CRGB::SkyBlue,
        CRGB::SlateBlue,
        CRGB::SlateGray,
        CRGB::Snow,
        CRGB::SpringGreen,
        CRGB::SteelBlue,
        CRGB::Tan,
        CRGB::Teal,
        CRGB::Thistle,
        CRGB::Tomato,
        CRGB::Turquoise,
        CRGB::Violet,
        CRGB::Wheat,
        CRGB::White,
        CRGB::WhiteSmoke,
        CRGB::Yellow,
        CRGB::YellowGreen
    };
int NumColors = sizeof(colors) / sizeof(uint32_t);

// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

typedef struct midi_struct {
  byte midiCommand;
  byte channel;
  byte pitch;
  byte velocity;
} midi_struct;


//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  midi_struct midiData;
  if (len >= sizeof(midiData))
  {
    memcpy(&midiData, incomingData, sizeof(midiData));
    // Middle C is pitch 60 so put into center of LED strip and make sure it is on the strip
    int ledIndex = midiData.pitch - 30;
    ledIndex = max(ledIndex, 0);
    ledIndex = min(ledIndex, NumColors - 1);
    uint32_t color;
    if (midiData.midiCommand == 0x90) // NoteOn
    {
      fill_solid(leds, NUM_LEDS, colors[ledIndex]);
      FastLED.show();
    }
  }

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("pitch: ");
  Serial.println(midiData.pitch);
  Serial.print("command: ");
  Serial.println(midiData.midiCommand);
  Serial.println();
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  FastLED.addLeds<WS2812B, 23, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, 25, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, 26, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, 27, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.addLeds<WS2812B, 32, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(30);
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Want to make sure all the LEDs are off before starting the radio
  FastLED.show();

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
}
 
void loop() {
}
