#include <Wire.h>

// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// https://learn.adafruit.com/adafruit-neopixel-uberguide/best-practices

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            7

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      300

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int RXLED = 17;

#define MAX_UCHAR 254
float intensity;
unsigned char mask[NUMPIXELS];
unsigned char max_intensity = 150;
const float pi = 3.14159;
int run_time_ms;
float location;
int center;


float gaussian_pdf(float x, float mu, float sigma) {
  return 1. / sqrt(2 * pi * sigma * sigma) * exp(-(x - mu) * (x - mu) / (2 * sigma * sigma));
}


void initialize_mask() {
  float width = (float) NUMPIXELS / 3.;
  float sigma = width / 5.;
  float maximum = gaussian_pdf(0., 0., sigma);

  for (int i = 0; i < NUMPIXELS; i++) {
    mask[i] = MAX_UCHAR * gaussian_pdf(i, NUMPIXELS / 2., sigma) / maximum;
  }
}

int get_mask_value(int i, int center) {
  int relative_location = (i - center) + NUMPIXELS / 2.;
  if (relative_location >= NUMPIXELS || relative_location < 0) {
    return 0;
  } else {
    return mask[relative_location];
  }
}


void apply_masks(int center) {
  for (int i = 0; i < pixels.numPixels(); i++) {
    int intensity = max_intensity * (get_mask_value(i, center) / (float) MAX_UCHAR);
    pixels.setPixelColor(i, pixels.Color(intensity, intensity, intensity));
  }
  pixels.show();
  delay(100);
}

void setup()
{
  pinMode(RXLED, OUTPUT);
  Serial.begin(9600); //This pipes to the serial monitor

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.show(); // Initialize all pixels to 'off'

  initialize_mask();
  
  delay(100);
}

void loop()
{
  digitalWrite(RXLED, LOW);
  run_time_ms = millis();
  location = (run_time_ms % 10000) / (float) 10000;
  center = round(location * pixels.numPixels());
  apply_masks(center);
  Serial.print(location);
  Serial.print(" ");
  Serial.println(center);
  digitalWrite(RXLED, HIGH);
}
