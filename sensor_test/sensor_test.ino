/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>


#define NUMSENSORS          3

// The distance sensor
VL53L0X sensors[NUMSENSORS];
float ratios[]= {0., 0.};
int centers[] = {100, 200};
int xshut_pins[] = {5, 6};
int addresses[] = {21, 22};
int relative_location;
float ratio;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY


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
          ratios[j] = dist;
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
}

void loop()
{
  
  read_sensors();
  Serial.println();
  delay(10);
}
