/*
This example shows how to set up and read multiple VL53L1X sensors connected to
the same I2C bus. Each sensor needs to have its XSHUT pin connected to a
different Arduino pin, and you should change sensorCount and the xshutPins array
below to match your setup.

For more information, see ST's application note AN4846 ("Using multiple VL53L0X
in a single design"). The principles described there apply to the VL53L1X as
well.
*/

#include <Wire.h>
#include <VL53L1X.h>
#define SENSOR_HQ_WIRE Wire1
// The number of sensors in your system.
const uint8_t sensorCount = 2;
pin_size_t I2C_SDA  = 6;
pin_size_t I2C_SCL  = 7;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {1,2};

VL53L1X sensors[sensorCount];

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    sensors[i].setBus(&SENSOR_HQ_WIRE);
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i);
 
    sensors[i].startContinuous(50);
  }
}

void loop()
{
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    Serial.print(sensors[i].read());
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print('\t');
  }
  Serial.println();
}
