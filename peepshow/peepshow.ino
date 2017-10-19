/*
 * An example of using multiple sensors to control a NEOPIXEL light strip.
 * Each sensor is assigned a center pixel around which a gaussian density
 * is used to define a light curve which is tied to that sensors distance
 * reading.
 */
#include <Wire.h>
#include <VL53L0X.h>

#define NUMSENSORS          2

// Variables related to the distance sensor
VL53L0X sensors[NUMSENSORS];
// The ratio is used to scale the intensity of each curve, it
// should take a value between 0 and 1
float ratios[]= {0., 0.};
// The pixel numbers around which each sensor's curve is centered
int centers[] = {100, 200};
// In order to set the address of each sensor on startup they
// need to be wired to different pins which are defined here.
int xshut_pins[] = {6, 8};
// The new address of each sensor.
int addresses[] = {21, 22};

//#define LONG_RANGE
// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
//#define HIGH_SPEED
//#define HIGH_ACCURACY

// Variables related to the light strip
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIXEL_PIN            6
#define NUMPIXELS      300

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define MAX_UCHAR 254
float intensity;
unsigned char curve[NUMPIXELS];
unsigned char max_intensity = 150;


// Computes the Gaussian density which is used to scale the light intensity
// around some center LED.
float gaussian_pdf(float x, float mu, float sigma) {
  return 1. / sqrt(2 * pi * sigma * sigma) * exp(-(x - mu) * (x - mu) / (2 * sigma * sigma));
}


// Creates an intensity curve that speeds up the apply step by precomputing
// the intensity curve that is centered around each sensor. 
void initialize_curve() {
  float width = (float) NUMPIXELS / 3.;
  float sigma = width / 5.;
  float maximum = gaussian_pdf(0., 0., sigma);

  for (int i = 0; i < NUMPIXELS; i++) {
    curve[i] = MAX_UCHAR * gaussian_pdf(i, NUMPIXELS / 2., sigma) / maximum;
  }
}


// Get the curve value for pixel `i` for a sensor that is centered at
// pixel `center`.
int get_curve_value(int i, int center) {
  int relative_location = (i - center) + NUMPIXELS / 2.;
  if (relative_location >= NUMPIXELS || relative_location < 0) {
    return 0;
  } else {
    return curve[relative_location];
  }
}


// Overlays the curves for each sensor and sets the intensity at each pixel.
void set_pixels() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    intensity = 0.;
    for (int j = 0; j < NUMSENSORS; j++) {
      intensity += ratios[j] * get_curve_value(i, centers[j]);
    }
    int int_intensity = max_intensity * intensity;
    pixels.setPixelColor(i, pixels.Color(0, intensity, intensity));
  }
  delay(10);
  pixels.show();
}


// Sets the signal processing preferences a give sensor.
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

// Loops through each sensor and assigns a new address.  This
// is required as all the sensors communicate over the i2c
// interface but fire up with the same address.
void initialize_sensors() {
    // Set all xshut pins to low, we'll bring them up one
    // at a time.
    for (int j = 0; j < NUMSENSORS; j++) {
        pinMode(xshut_pins[j], OUTPUT);
        digitalWrite(xshut_pins[j], LOW);
    }
    delay(100);
  
    for (int j = 0; j < NUMSENSORS; j++) {
        // By changing the PIN mode we've set the pin
        // to 'not LOW' (which is different than HIGH
        pinMode(xshut_pins[j], INPUT);
        delay(5);
    
        Serial.print("Initializing sensor ");
        Serial.print(j);
        Serial.print(" with address ");
        Serial.println(addresses[j]);
        delay(5);
        sensors[j].init(true);
        sensors[j].setAddress(addresses[j]);
    
        configure_single_sensor(sensors[j]);
    }
}

// reads the distance from each sensor and converts it to an intensity
// ratio which is later used to scale the intensity.
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
    }
    Serial.println("");
}


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // Assign each sensor a different address and configure them.
  initialize_sensors();
  // This initializes the NeoPixel library.
  pixels.begin(); 
  // Initialize all pixels to 'off'
  pixels.show();
  // Precompute the intensity curve.
  initialize_curve();  
  delay(100);
}

void loop()
{
  // Read each sensor's value and turn it into an intensity ratio
  read_sensors();
  // Refresh the light strip.
  set_pixels();
  delay(10);
}
