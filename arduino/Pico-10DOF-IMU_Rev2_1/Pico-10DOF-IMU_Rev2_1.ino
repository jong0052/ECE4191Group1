// Author: Yide Tao
// Date: 07/10/2023
// Template: https://www.waveshare.com/wiki/Pico-10DOF-IMU
// Zero Position Update
//To Do list 
/*
1. Set up the pyrial communication and the activation code system.
2. Test
3. Test the ToF sensors. 
4.
*/
///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------Libraries-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
#include "imu.h" //IMU sensors Code
#include "SensorFusion.h" // Packages for sensor fusion

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <Arduino_GFX_Library.h> //Display package
#include <Wire.h> // I2c library

#include <Adafruit_VL53L0X.h> // Package for Low Quality Sensors
#include <VL53L1X.h> // Package for High Quality Sensors

///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------Variables------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
// 1. Display 
#define TFT_BL 13
Arduino_DataBus *bus = new Arduino_RPiPicoSPI(8 /* DC */, 9 /* CS */, 10 /* SCK */, 11 /* MOSI */, UINT8_MAX /* MISO */, spi1 /* spi */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 12 /* RST */, 3 /* rotation */, true /* IPS */, 135, 240, 52, 30, 60, 40);

// 2. IMU Variables
IMU_EN_SENSOR_TYPE enMotionSensorType;
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

// 3. Other sensor Variables
float PRESS_DATA=0;
float TEMP_DATA=0;

// 4. I2C wires
#define I2C_SDA 6
#define I2C_SCL 7

// 5. Sensor Fusion Module Variables
SF fusion; // Sensor Fusion Module
float deltat;
float prev_yaw, current_yaw, current_yaw_unnormalised;
float lin_v_now = 0;
float lin_v_prev = 0;

long prevT = 0;
long currT = 0;
float dt = 0;
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float gz_max = 0;
float ay_max = 0;
float Pi_fx = 3.14159265;
uint8_t u8Buf[3];

bool first_measure = true;
float lin_v_offset_main = 0;
float yaw_offset_main = 0;

// 6. ToF sensors
float tof_0x_arr[6] = {0, 0, 0, 0, 0, 0};
bool sensor_ready = false;
///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------IMU Helper Functions-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0); // Use modulo operator to get the remainder
  if (angle < 0) {
    angle += 360.0; // Make sure the angle is positive
  }
  return angle;
}
///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------Serial Communication Functions-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
// Serial Code 
// 101: Wait
// 201: Start
// 221: Measure
// Is Stationary
// 0: Not Stationary
// 1: Is Stationary
int serial_code = 101; // Which is wait
int is_stationary = 0;
float serial_data = 0;

/////////////////////////////
// 1. Function ParseString //
/////////////////////////////
void parseStringToFloats(String input, int &serial_code) {
  int index = input.indexOf("[") + 1; // Find the starting index of the values
  int endIndex = input.indexOf("]"); // Find the ending index of the values

  String valuesStr = input.substring(index, endIndex); // Extract the values between '[' and ']'
  valuesStr.replace(" ", ""); // Remove any spaces
  serial_code = valuesStr.substring(0, 3).toInt(); // All of the code have three digits
}

/////////////////////////////
// 2. Function readCommand //
/////////////////////////////

void readCommand(String input, int &serial_code, float &serial_data) {
  int index = input.indexOf("[") + 1; // Find the starting index of the values
  int endIndex = input.indexOf("]"); // Find the ending index of the values
  
  String valuesStr = input.substring(index, endIndex); // Extract the values between '[' and ']'
  valuesStr.replace(" ", ""); // Remove any spaces

  int commaIndex = valuesStr.indexOf(","); // Find the index of the first comma
  
  serial_code = valuesStr.substring(0, commaIndex).toInt(); // Convert the substring to a float and assign to variable
  valuesStr = valuesStr.substring(commaIndex + 1); // Remove the parsed value and comma
  
  commaIndex = valuesStr.indexOf(",");
  serial_data = valuesStr.substring(0, commaIndex).toFloat();
}

///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------ToF Sensor Functions-----------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
//Define which Wire objects to use, may depend on platform
// or on your configurations.
#define SENSOR_WIRE1 Wire // I2C for LQ ToF
#define SENSOR_WIRE2 Wire 
#define SENSOR_WIRE3 Wire 
#define SENSOR_WIRE4 Wire 
#define SENSOR_WIRE5 Wire
#define SENSOR_WIRE6 Wire 
#define SENSOR_WIRE_hq Wire  

                      
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++VL53L1CX++++++++++++++++++++++++++++++++++++++++++++++++++++//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define the two high quality ToF Sensors

const uint8_t hq_sensorCount = 2;
const uint8_t xshutPins[hq_sensorCount] = {2,3};
VL53L1X sensors_hq[hq_sensorCount];

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++VL53L0X++++++++++++++++++++++++++++++++++++++++++++++++++++//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup mode for doing reads
typedef enum {
  RUN_MODE_DEFAULT = 1,
  RUN_MODE_ASYNC,
  RUN_MODE_GPIO,
  RUN_MODE_CONT
} runmode_t;

runmode_t run_mode = RUN_MODE_DEFAULT;
uint8_t show_command_list = 1;

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint8_t sensor_status; // status from last ranging in continuous.
} sensorList_t;

// Actual object, could probably include in structure above61
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;
Adafruit_VL53L0X sensor5;
Adafruit_VL53L0X sensor6;


// Setup for 4 sensors
sensorList_t sensors[] = {
    {&sensor1, &SENSOR_WIRE1, 0x21, 14, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
    
    {&sensor2, &SENSOR_WIRE2, 0x22, 19, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
    
    {&sensor3, &SENSOR_WIRE3, 0x23, 21, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
    
    {&sensor4, &SENSOR_WIRE4, 0x24, 28, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
     
    {&sensor5, &SENSOR_WIRE5, 0x25, 27, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
    {&sensor6, &SENSOR_WIRE6, 0x26, 26, -1,
     Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY, 0, 0},
     
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
int sensor_l0x[COUNT_SENSORS];
int sensor_l1x[2];

const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//++++++++++++++++++++++++++++++++++++++++++Sensor Functions++++++++++++++++++++++++++++++++++++++++++++++++++++//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//====================================================================

void Initialize_sensors() {
  //====================================================================
  // Initialise  VL53L0X Sensors
  //====================================================================
  // Check for All Sensors are Found
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(100);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      //Serial.print(i, DEC);
      //Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    //Serial.println("No valid sensors found");
    while (1)
      ;
  }
  //====================================================================
  // Initialise  VL53L1X Sensors
  //====================================================================
  // Disable/reset all sensors by driving their XSHUT pins low.
  
}

void Initialize_hq_sensors()
{
  for (uint8_t i = 0; i < hq_sensorCount; i++)
   {
      pinMode(xshutPins[i], OUTPUT);
      digitalWrite(xshutPins[i], LOW);
   }
   // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < hq_sensorCount; i++)
  {
    sensors_hq[i].setBus(&SENSOR_WIRE_hq);
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors_hq[i].setTimeout(200);
    if (!sensors_hq[i].init())
    {
      //Serial.print("Failed to detect and initialize VL53L1X sensor ");
      //Serial.println(i);
      while (1);
    }
    sensors_hq[i].setAddress(0x31 + i);
    sensors_hq[i].setDistanceMode(VL53L1X::Long);
    sensors_hq[i].setMeasurementTimingBudget(50000);
    sensors_hq[i].startContinuous(50);
  }
}

//====================================================================
// Simple Sync read sensors.
//====================================================================
int sens1_bias[8] = {-10, -15, -20, -10, 0, 0, 0, 0};
int sens2_bias[8] = {-30, -10, -15, -30, -10, -15, -15, -30};
//int sens2_bias2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void read_sensors() {
  // First use simple function
  uint16_t ranges_mm[COUNT_SENSORS];
  for (int i = 0; i < COUNT_SENSORS; i++) {
    ranges_mm[i] = sensors[i].psensor->readRange();
    delay(60);
  } 
  
  for (int i = 0; i < COUNT_SENSORS; i++) {
    if (ranges_mm[i] < 40){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[0];
    }  
    else if (ranges_mm[i] < 100){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[1];
    }  
    else if (ranges_mm[i] < 150){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[2];
    }  
    else if (ranges_mm[i] < 250){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[3];
    }  
    else if (ranges_mm[i] < 350){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[4];
    }  
    else if (ranges_mm[i] < 500){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[5];
    }  
    else if (ranges_mm[i] < 800){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[6];
    }  
    else if (ranges_mm[i] < 1200){
      sensor_l0x[i] = ranges_mm[i] - sens1_bias[7];
    }  
    else{
      sensor_l0x[i] = 4040;
    }
  }
  
  for (uint8_t i = 0; i < hq_sensorCount; i++)
  {
    if (sensors_hq[i].timeoutOccurred()) { 
      sensor_l1x[i] = 4040;
    }else {
      int sensor_l1x_raw = sensors_hq[i].read();
      if (i == 0){
       if (sensor_l1x_raw > 490 and sensor_l1x_raw < 512){
        sensor_l1x[i] = sensor_l1x_raw - 0;
       }
       else {
       sensor_l1x[i] = sensor_l1x_raw - 0;
       }
      }
      // Sensor 2
      if (i == 1){
        if (sensor_l1x_raw < 40){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[0];
         }
         else if (sensor_l1x_raw < 115){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[1];
         }
         else if (sensor_l1x_raw < 135){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[2];
         }
         else if (sensor_l1x_raw < 230){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[3];
         }
         else if (sensor_l1x_raw < 345){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[4];
         }
         else if (sensor_l1x_raw < 485){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[5];
         }
         else if (sensor_l1x_raw < 980){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[6];
         }
         else if (sensor_l1x_raw < 1200){
          sensor_l1x[i] = sensor_l1x_raw - sens2_bias[7];
         }
         else if (sensor_l1x_raw >= 1200){
            sensor_l1x[i] = 4040;
         }
      }
       
    }
    delay(30);
  }
  
}

float read_sensors_i_1(int i){
   delay(300);
   uint16_t ranges_mm = sensors[i].psensor->readRange();
   float output = ranges_mm;
   return output;
}

float read_sensors_i_2(int i){
   delay(300);
   float output;
   int sensor_l1x_raw;
   if (sensors_hq[i].timeoutOccurred()) { 
      output = 4040;
    }else {
      sensor_l1x_raw = sensors_hq[i].read();
    }
   output = sensor_l1x_raw;
   return output;
}
///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------Set Up-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  // 1. Reset SDA and SCL
  rp2040.idleOtherCore();
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  // 2. Run Display
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(RED);
  // 3. Run Serial
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(5000); // Time for Sensors, IMU, Display and Sensors to set up

  // Wait for Pi to send the setup code
  gfx->fillScreen(BLACK);
  while (serial_code != 201){
    gfx->setTextColor(WHITE);
    gfx->setCursor(20, 10);
    gfx->println("Waiting For Pi");
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      parseStringToFloats(data, serial_code);
    }
    delay(500);
  }
  // 3.2 Initialise the IMU sensors
  
	imuInit(&enMotionSensorType, gfx);
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE);
  gfx->setCursor(20, 10);
  gfx->println("Initiating NMNI Algorithm");
  delay(2000);
  
  for (int i = 0; i < 100; i++){
    // Implment NMNI Algorithm
    imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
    gz = stGyroRawData.s16Z;
    ay = stAccelRawData.s16Y*9.8*(2.0/32767.5);
    if (gz_max < abs(gz)){
      gz_max = abs(gz);
    }
    if (ay_max < abs(ay)){
      ay_max = abs(ay);
    }
   }
  //Serial.print("Max gz when stationary:");
  //Serial.println(gz_max);
  rp2040.resumeOtherCore();
}
///////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------Set Up Sensors-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
void setup1(){
  delay(5000);
  //Serial.begin(115200);
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(400000);
  // wait until serial port opens ... For 5 seconds max
  // initialize all of the pins.
  //Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);

    if (sensors[i].interrupt_pin >= 0)
      pinMode(sensors[i].interrupt_pin, INPUT_PULLUP);
  }
  //Serial.println(F("Starting..."));
  Initialize_hq_sensors();// Must be start last
  Initialize_sensors();// Must be start last
  sensor_ready = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------Loop Module (MultiThreading)-----------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////
 
void loop() {
  // put your main code here, to run repeatedly:
  // Equate the previous Yaw to the current Yaw
  // Change the Previous Velocity and Time to the Current Velocity and Time
  prevT = currT;
  lin_v_prev = lin_v_now;
  
  // Call the get data function
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  // Nullify the IMU when the Robot is not moving
  if (is_stationary == 1) 
  {
    ax = 0;
    ay = 0;
    az = stAccelRawData.s16Z*9.8*(2.0/32767.5);
    gx = 0;
    gy = 0;
    gz = 0;
  }
  else if (abs(stGyroRawData.s16Z) < gz_max){
    // Normalise the data
    ax = stAccelRawData.s16X*9.8*(2.0/32767.5); 
    ay = stAccelRawData.s16Y*9.8*(2.0/32767.5);
    az = stAccelRawData.s16Z*9.8*(2.0/32767.5);
    gx = 0;
    gy = 0;
    gz = 0;
  }
  else
  {
    // Normalise the data
    ax = stAccelRawData.s16X*9.8*(2.0/32767.5); 
    ay = stAccelRawData.s16Y*9.8*(2.0/32767.5);
    az = stAccelRawData.s16Z*9.8*(2.0/32767.5);
    
    // According to the documentation the FS_SEL = 2
    gx = stGyroRawData.s16X*(Pi_fx/180)*(1000.0 / 32767.5);
    gy = stGyroRawData.s16Y*(Pi_fx/180)*(1000.0 / 32767.5);
    ///(32.8*180);
    gz = stGyroRawData.s16Z*(Pi_fx/180)*(1000.0 / 32767.5);
  }
  
  
  ///(32.8*180);
  
  mx = stMagnRawData.s16X*0.6;
  my = stMagnRawData.s16Y*0.6;
  mz = stMagnRawData.s16Z*0.6;
  // Update the filter
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat); 

  // Get roll, pitch and yaw
  currT = micros(); // Measure the current time spent
  if (first_measure == true)
  {
    delay(1000);
    dt = (currT - prevT)*0.000001;
    yaw_offset_main = fusion.getYaw();
    lin_v_now = 0;
    first_measure = false;
  }
  else
  {
    current_yaw_unnormalised = fusion.getYaw() - yaw_offset_main;
    current_yaw = normalizeAngle(current_yaw_unnormalised);
    // Calculate the Linear velocity
    dt = (currT - prevT)*0.000001;

    // Reset the Linear Velocity when the robot is not moving.
    if (is_stationary == 1) {
     lin_v_now = 0; 
    }
    else{
     if (abs(ay) > ay_max){
      lin_v_now = lin_v_prev + dt*ay;
     }
     else{
      lin_v_now = lin_v_prev;
     }
    }
  }
  gfx->fillScreen(BLACK);
  gfx->setTextSize(2/* x scale */, 2 /* y scale */, 1 /* pixel_margin */);
  gfx->setTextColor(WHITE);
  gfx->setCursor(0, 10);
  gfx->print("Yaw: ");
  gfx->println(current_yaw);
  gfx->setCursor(140, 10);
  gfx->print("v: ");
  gfx->println(lin_v_now);
  gfx->setCursor(20, 30);
  gfx->print("G_Y_t: ");
  gfx->println(gz_max);
  if (sensor_ready == true){
    gfx->setTextColor(GREEN);
    gfx->setCursor(20, 50);
    gfx->print("Sensor: Ready");
  //delay(200);
  } else {
    gfx->setTextColor(RED);
    gfx->setCursor(20, 50);
    gfx->print("Sensor: Not Ready");
  }
  if (is_stationary == 0){
    gfx->setTextColor(GREEN);
    gfx->setCursor(20, 70);
    gfx->print("Stationary: False");
  //delay(200);
  } else {
    gfx->setTextColor(RED);
    gfx->setCursor(20, 70);
    gfx->print("Stationary: True");
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------Setup Module 2 (MultiThreading)-----------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////

// --------------------------------------<Control Codes>-----------------------------------
// [Read Functions]
// 221: Read Yaw, lin_v_now.
// 222: Sensor Measure.
// [Write Functions]
// 320: Change stationary
// 321: Reset Angle
// 322: Localise
// ----------------------------------------------------------------------------------------
void print_serial(float [], int);
void localise_centre(float, float );
void loop1(){
  // Checks fow which code
  if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      readCommand(data, serial_code, serial_data);
      
      if (serial_code == 221) // Read IMU
      {
        Serial.print("Pico Angle: [");
        float print_ore[2];
        print_ore[0] = current_yaw;
        print_ore[1] = lin_v_now;
        if(Serial.availableForWrite()>0){
          print_serial(print_ore, 2);
        }
      }
      else if (serial_code == 222) // Read Sensors
      {
        read_sensors();
        Serial.print("Pico Sensor: [");
        float print_sens[8];
        for (int i = 0; i < COUNT_SENSORS; i++) {
          print_sens[i] = sensor_l0x[i];
        }
        for (int i = 0; i < hq_sensorCount; i++) {
          print_sens[i + COUNT_SENSORS] = sensor_l1x[i];
        }
        if(Serial.availableForWrite()>0){
          print_serial(print_sens, 8);
        }
      }
     else if (serial_code == 320){
        if (serial_data == 1){
          is_stationary = 1;
        }else{
          is_stationary = 0;
        }
     }
     else if (serial_code == 321){
        float new_offset = serial_data - current_yaw;
        yaw_offset_main = yaw_offset_main - new_offset;
     }
     else if (serial_code == 322){
        float x;
        float y;
        float car_l = 240;
        float car_w = 195;
        float x1, x2;
        y = 600 - (read_sensors_i_1(0)/3 + read_sensors_i_1(1)/3 + read_sensors_i_1(2)/3) - (car_l/2);
        x1 = -600 + 0.6*read_sensors_i_1(3)+0.4*read_sensors_i_2(1) +  (car_w/2);
        x2 = 600 - 0.6*read_sensors_i_1(5)-0.4*read_sensors_i_2(0) -  (car_w/2);
        x = (x1 + x2)/2;
        Serial.print("Pico Location: [");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.println("]");
     }
  }
}
void print_serial(float ser_arr[], int arr_size){
  for (int i = 0; i < arr_size; i++){
    Serial.print(ser_arr[i]);
    if(i < arr_size-1){
      Serial.print(", ");
    }
  }
  Serial.print("]");
  Serial.println();
}
