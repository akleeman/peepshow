#include <Wire.h>
#include <VL53L0X.h>

#define NUMSENSORS          2

// The distance sensor
VL53L0X sensors[NUMSENSORS];
float ratios[]= {0., 0.};
int centers[] = {100, 200};
int xshut_pins[] = {6, 8};
int addresses[] = {21, 22};
int relative_location;

//#define LONG_RANGE
// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
//#define HIGH_SPEED
//#define HIGH_ACCURACY

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIXEL_PIN            6
#define NUMPIXELS      300

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

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


void apply_masks() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    intensity = 0.;
    for (int j = 0; j < NUMSENSORS; j++) {
      intensity += ratios[j] * get_mask_value(i, centers[j]);
    }
    int int_intensity = max_intensity * intensity;
    Serial.print(intensity);
    Serial.print(" ");
    pixels.setPixelColor(i, pixels.Color(0, intensity, intensity));
  }
  pixels.show();
  Serial.println("");
  delay(100);
}


void configure_single_sensor(VL53L0X sensor) {
    sensor.init();
    sensor.setTimeout(500);

    #if defined LONG_RANGE
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor.setSignalRateLimit(0.1);
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

    #if defined HIGH_SPEED
      // reduce timing budget to 20 ms (default is about 33 ms)
      sensor.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
      // increase timing budget to 200 ms
      sensor.setMeasurementTimingBudget(200000);
    #endif
}

void initialize_sensors() {
    // Set all xshut pins to low, we'll bring them up one
    // at a time.
    for (int j = 0; j < NUMSENSORS; j++) {
        pinMode(xshut_pins[j], OUTPUT);
        digitalWrite(xshut_pins[j], LOW);
    }
    delay(500);
  
    for (int j = 0; j < NUMSENSORS; j++) {
        //SENSOR
        pinMode(xshut_pins[j], INPUT);
        delay(150);
    
        Serial.print("Initializing sensor ");
        Serial.println(j);
        Serial.print("Setting address to ");
        Serial.println(addresses[j]);
        delay(100);
        sensors[j].init(true);
        sensors[j].setAddress(addresses[j]);
    
        configure_single_sensor(sensors[j]);
    }
}


void read_sensors() {
    int dist;
    for (int j = 0; j < NUMSENSORS; j++) {
        dist = sensors[j].readRangeSingleMillimeters();
        if (sensors[j].timeoutOccurred()) {
            Serial.print(" TIMEOUT");
        } else {
          ratios[j] = dist / 1000.;
          Serial.print(j); Serial.print(" : "); Serial.print(ratios[j]); Serial.print("  ");
        }
        delay(10);
    }
    Serial.println("");
}


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  initialize_sensors();

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.show(); // Initialize all pixels to 'off'

  initialize_mask();
  
  delay(100);

}

void loop()
{
  read_sensors();
  apply_masks();
  Serial.println();
  delay(10);
}
