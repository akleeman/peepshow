/*
 * An example of using multiple sensors to control a NEOPIXEL light strip.
 * Each sensor is assigned a center pixel around which a gaussian density
 * is used to define a light curve which is tied to that sensors distance
 * reading.
 */
#include <Wire.h>
#include <VL53L0X.h>
#include <Filters.h>

#define NUMSENSORS          10
#define MAXSENSORS          10

// Variables related to the distance sensor
VL53L0X sensors[NUMSENSORS];
bool sensor_available[NUMSENSORS];
// The ratio is used to scale the intensity of each curve, it
// should take a value between 0 and 1
float ratios[]= {0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};
// The pixel numbers around which each sensor's curve is centered
const int centers[] = {30, 60, 90, 120, 150, 180, 210, 240, 270};
// In order to set the address of each sensor on startup they
// need to be wired to different pins which are defined here.
// Negative numbers indicate shift register pins.
const int xshut_pins[] = {5, 6, 7, 8, 9, 10, A0, A1, A2, A3};
// The new address of each sensor.
const int addresses[] = {30, 31, 32, 33, 35, 36, 37, 38, 39, 40};
const int address_at_power_on = 41;
unsigned long sensor_init_timeout = 10000;

#define MAX_DIST      1200.
#define ZERO_DIST     400
#define MIN_DIST      200.
#define SMOOTHING     0.8

byte shift_register_data = 0;
const int shift_register_data_pin = 4;
const int shift_register_latch_pin = 5;
const int shift_register_clock_pin = 6;

FilterOnePole filters[NUMSENSORS];

//#define LONG_RANGE
// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
#define HIGH_SPEED
//#define HIGH_ACCURACY
//#define MAXDIST      0.01  

// Variables related to the light strip
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIXEL_PIN        4
#define NUMPIXELS      300
#define MAXPIXELS      300

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define MAX_UCHAR 254
float intensity;
unsigned char curve[NUMPIXELS];
float max_intensity = 254.;
const float pi = 3.14159;

// Computes the Gaussian density which is used to scale the light intensity
// around some center LED.
float gaussian_pdf(float x, float mu, float sigma) {
  return 1. / sqrt(2 * pi * sigma * sigma) * exp(-(x - mu) * (x - mu) / (2 * sigma * sigma));
}


// Creates an intensity curve that speeds up the apply step by precomputing
// the intensity curve that is centered around each sensor. 
void initialize_curve() {
  float width = (float) NUMPIXELS / 3.;
  float sigma = width / 1.;
  float maximum = gaussian_pdf(0., 0., sigma);

  for (int i = 0; i < NUMPIXELS; i++) {
    curve[i] = MAX_UCHAR * gaussian_pdf(i, NUMPIXELS / 2., sigma) / maximum;
  }
}


// Get the curve value for pixel `i` for a sensor that is centered at
// pixel `center`.
float get_curve_value(int i, int center) {
  int relative_location = (i - center) + NUMPIXELS / 2.;
  if (relative_location >= NUMPIXELS || relative_location < 0) {
    return 0.;
  } else {
    return ((float) curve[relative_location] / MAX_UCHAR);
  }
}


// Overlays the curves for each sensor and sets the intensity at each pixel.
void set_pixels() {
  for (int i = 0; i < NUMPIXELS; i++) {
    intensity = 1.;
    for (int j = 0; j < NUMSENSORS; j++) {
      intensity -= (1. - ratios[j]) * get_curve_value(i, centers[j]);
    }
    intensity = min(max(0., intensity), 1.);
    intensity = max_intensity * intensity * intensity;
    pixels.setPixelColor(i, pixels.Color(0., intensity, intensity));
  }
  delay(20);
  pixels.show();
}


// Sets the signal processing preferences a give sensor.
void configure_single_sensor(VL53L0X sensor) {
    sensor.init();
    sensor.setTimeout(1000);

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
    sensor.startContinuous();
}


void initialize_shift_register() {
    pinMode(shift_register_latch_pin, OUTPUT);
    pinMode(shift_register_data_pin, OUTPUT);  
    pinMode(shift_register_clock_pin, OUTPUT);
}


void write_to_shift_register(byte data) {
    digitalWrite(shift_register_latch_pin, LOW);
    shiftOut(shift_register_data_pin,
             shift_register_clock_pin,
             MSBFIRST,
             data);
    digitalWrite(shift_register_latch_pin, HIGH);
}

void turn_off_one_sensor(int xshut_pin) {      
    if (xshut_pin> 0) {
       pinMode(xshut_pins[xshut_pin], OUTPUT);
       digitalWrite(xshut_pins[xshut_pin], LOW);
    } else if (xshut_pin <= 0) {
       // Set the corresponding bit in the shift register byte
       // to low, then update the register.
       bitWrite(shift_register_data, -xshut_pin, LOW);
       write_to_shift_register(shift_register_data);
    }
}


void turn_off_all_sensors() {
    // Set all xshut pins to low, we'll bring them up one
    // at a time.
    for (int j = 0; j < MAXSENSORS; j++) {
      turn_off_one_sensor(j);
    }
    // Just in case we make sure the shift register is fully off.
    shift_register_data = 0;
    write_to_shift_register(shift_register_data);
}

void turn_on_one_sensor(int xshut_pin) {      
    if (xshut_pin> 0) {
       // By changing the PIN mode we've set the pin
       // to 'not LOW' (which is different than HIGH
       pinMode(xshut_pin, INPUT);        
    } else if (xshut_pin <= 0) {
       // Set the corresponding bit in the shift register byte
       // to high, then update the register.
       bitWrite(shift_register_data, -xshut_pin, HIGH);
       write_to_shift_register(shift_register_data);
    }
}

bool is_sensor_available() {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address_at_power_on);
    byte error = Wire.endTransmission();
    return error == 0;
    Serial.print(" "); Serial.print(error); Serial.print(" ");
}

bool wait_for_sensor() {
  unsigned long start_time = millis();
  while(millis() <= (start_time + sensor_init_timeout)) {
    if (is_sensor_available()) {
      return true;
    }
    delay(50);
  }
  return false;
}

bool initialize_one_sensor(int j) {
    Serial.print("Initializing sensor ");
    Serial.print(j);
    Serial.print(" on pin ");
    Serial.print(xshut_pins[j]);
    Serial.print(" with address ");
    Serial.print(addresses[j]);
    Serial.print(". xshut on");
    turn_on_one_sensor(xshut_pins[j]);
    if (wait_for_sensor()) {
      Serial.print(". found");
      sensors[j].setTimeout(1000);
      if (!sensors[j].init()) {
        sensor_available[j] = false;
        return false;
      }
      Serial.print(". initialized");
      sensors[j].setAddress(addresses[j]);
      Serial.print(". readdressed");
      configure_single_sensor(sensors[j]);
      Serial.println(". configured ---");
    } else {
      Serial.println(". INIT TIMEDOUT.");
      return false;
    }
    return true;
}

// Loops through each sensor and assigns a new address.  This
// is required as all the sensors communicate over the i2c
// interface but fire up with the same address.
void initialize_sensors() {
    turn_off_all_sensors();
    delay(100);
    for (int j = 0; j < NUMSENSORS; j++) {
        sensor_available[j] = initialize_one_sensor(j);
        if (!sensor_available[j]) {
          Serial.print("Turning off sensor ");
          Serial.print(j);
          Serial.println(" so it doesn't interfere with others");
          turn_off_one_sensor(j);
        }
        delay(100);
    }
}

// reads the distance from each sensor and converts it to an intensity
// ratio which is later used to scale the intensity.
void read_sensors() {
    int dist;
    Serial.print("* ");
    for (int j = 0; j < NUMSENSORS; j++) {
        if (!sensor_available[j]) {
          ratios[j] = 1.;
          Serial.print("NAN, ");
        } else {
          dist = sensors[j].readRangeContinuousMillimeters();
          dist = min(dist, MAX_DIST);
          filters[j].input(dist);
          dist = filters[j].output();
          if (sensors[j].timeoutOccurred()) {
              Serial.print("NaN,");
          } else {
            if (dist < MIN_DIST) {
              dist = MAX_DIST;
            }
            ratios[j] = SMOOTHING * ratios[j] + (1. - SMOOTHING) * max(0., (dist - ZERO_DIST) / (MAX_DIST - ZERO_DIST));
            Serial.print(ratios[j]); Serial.print(",    ");
          }
       }
    }
    Serial.print("    "); Serial.print(millis());
    Serial.println("");
}


void initialize_filters() {
  for (int j = 0; j < NUMSENSORS; j++) {  
    filters[j] = FilterOnePole(LOWPASS, 5.0);
  }
}

void set_ratio(double x) {
  for (int j = 0; j < NUMSENSORS; j++) {
    ratios[j] = x;
  }
}

void flash_pixels() {
  set_ratio(0.);
  set_pixels();  
  set_ratio(1.);
  set_pixels();
  set_ratio(0.);
  set_pixels();  
}

void setup()
{
  delay(100);
  Serial.begin(9600);
  Wire.begin();
  while(!Serial) {
    delay(10);
  }
  Serial.println("setup");
  initialize_shift_register();
  // Assign each sensor a different address and configure them.
  initialize_sensors();
  // This initializes the NeoPixel library.
  pixels.begin(); 
  // Initialize all pixels to 'off'
  pixels.show();
  flash_pixels();
  // Precompute the intensity curve.
  initialize_curve();
  Serial.println("Finished setup");
}

void loop()
{
  // Read each sensor's value and turn it into an intensity ratio
  read_sensors();
  // Refresh the light strip.
  set_pixels();
  delay(5);
}
