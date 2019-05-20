#include <Adafruit_GFX.h>

#include <gfxfont.h>

#include <Adafruit_ILI9341_8bit.h>

Adafruit_ILI9341_8bit_STM tft = Adafruit_ILI9341_8bit_STM();

void setup() {
  Serial.begin(115200);

  Serial.println("ILI9341 Test!");

  tft.begin();

  // read diagnostics (optional but can help debug problems)
}

void loop() {
  tft.fillScreen(ILI9341_PINK); 
}
