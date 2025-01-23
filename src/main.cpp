/*
  CrayUncleBurton.com OtterWorks
  OtterPilot v3.7.5 Hardware Test
  by Bryan A. "CrazyUncleBurton" Thompson

  Last Updated 1/23/2025

  
  This is code that the hardware manufacturer uses to test the hardware.

  Github Repository:  OtterPilot-v3.7.5-Hardware-Test
  https://github.com/CrazyUncleBurton/OtterPilot-v3.7.5-Hardware-Test

  This program is exclusively for the OtterWorks OtterPilot v3.7.5 Hardware.

  NOTE:  This is not firmware for the OtterPilot v3.7.5!
  
  The production firmware is located Here:

  Github Repository:  OtterPilot-v3.6.0
  https://github.com/CrazyUncleBurton/OtterPilot-v3.6.0
  

*/


// Includes

// Microcontroller Board Support for LilyGo T-Display S3 Touch
// 
// Note:  This is like the board support file for the LilyGo T-Display S3 Touch
// and contains info about which GPIO are connected to which physical pins.
// This is for the LilyGo board only.  There are more System-specific pin 
// mappings below.

#include "pin_config.h"

// Needed to parse Arduino syntax like delay and libraries like Wire and SPI 
// and Serial
#include <Arduino.h>

// Arduino I2C library
#include <Wire.h>

// SPI needed by one of the Adafruit libraries
#include <SPI.h>

// Servo Driver Library
#include <Adafruit_PWMServoDriver.h>

// This is a special version, not one downloaded through PlatformIO.  It's in
// the custom libraries folder, not the lob deps folder.  Recommend changing
// this to LVGL so we can use a UI app like Squareline.
#include <TFT_eSPI.h>


// System-Specific GPIO Pin Config Definitions

// Note:  There are more pin definitions in the file pin_config.h, which holds
// Info on the LilyGo microcontroller board itself.


// G2 is the calibration input (internal connection fed from PWM Controller 
// PWM4, not from the receiver)
#define FREQUENCY_INPUT4 2

// G3 communicates with the Traxxas lighting system (See the unfinished
// code below)
#define TRAXXAS_LIGHTING 3 

// G10 is the VIN_MEAS input.
// This needs to be set up as an analog input
#define VIN_MEAS 10

// G11 is the RX_CH3 auxillary input from an auxilliary channel on the receiver
#define FREQUENCY_INPUT3 11

// G12 is the RX_CH2 throttle input from the receiver
#define FREQUENCY_INPUT2 12

// G13 is the RX_CH1 steering input from the receiver
#define FREQUENCY_INPUT 13

// Display Power
#define PIN_POWER_ON 15

// Display Lighting Power
#define PIN_LCD_BL 38


// TFT Display Config
unsigned colour = 0xFFFF;
TFT_eSPI tft = TFT_eSPI();


// Button Config
int currentState_1; // the current reading from the input pin
int currentState_2; // the current reading from the input pin


// PWM Module Config
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


// Variables

// Steering Channel
volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long Pulses = 0;         // Current reading, as computed by the interrupt
volatile long last_Pulses = 1500; // Initial safe value
int PulseWidth = 0;

// Throttle Channel
volatile long StartTime2 = 0;
volatile long CurrentTime2 = 0;
volatile long Pulses2 = 0;         // Current reading, as computed by the interrupt
volatile long last_Pulses2 = 1500; // Initial safe value
int PulseWidth2 = 0;

// Aux Channel
volatile long StartTime3 = 0;
volatile long CurrentTime3 = 0;
volatile long Pulses3 = 0;         // Current reading, as computed by the interrupt
volatile long last_Pulses3 = 1500; // Initial safe value
int PulseWidth3 = 0;

// Calibration Channel
volatile long StartTime4 = 0;
volatile long CurrentTime4 = 0;
volatile long Pulses4 = 0;         // Current reading, as computed by the interrupt
volatile long last_Pulses4 = 1500; // Initial safe value
int PulseWidth4 = 0;


// Measure Pulsewidth of Signals from RC Receiver

// Timer interrupt service routine to update the Pulse width counter in
// microseconds for CH1 Steering Channel
void PulseTimer()
{
  CurrentTime = micros();
  int PulseState = digitalRead(FREQUENCY_INPUT);
  if (PulseState == 0)
  {
    Pulses = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
  if (PulseState == 1)
  {
    StartTime = CurrentTime;
  }
}

// Timer interrupt service routine to update the Pulse width counter in
// microseconds for CH2 Throttle Channel
void PulseTimer2()
{
  CurrentTime2 = micros();
  int PulseState2 = digitalRead(FREQUENCY_INPUT2);
  if (PulseState2 == 0)
  {
    Pulses2 = CurrentTime2 - StartTime2;
    StartTime2 = CurrentTime2;
  }
  if (PulseState2 == 1)
  {
    StartTime2 = CurrentTime2;
  }
}

// Timer interrupt service routine to update the Pulse width counter in
// microseconds for CH3 / Aux Channel
void PulseTimer3(){
  CurrentTime3 = micros();
  int PulseState3=digitalRead(FREQUENCY_INPUT3);
  if (PulseState3==0) {
    Pulses3 = CurrentTime3 - StartTime3;
    StartTime3 = CurrentTime4;
  }
  if (PulseState3==1) {
    StartTime3 = CurrentTime3;
  }
}

// Timer interrupt service routine to update the Pulse width counter in
// microseconds for G2 / Calibration Channel
void PulseTimer4(){
  CurrentTime4 = micros();
  int PulseState4=digitalRead(FREQUENCY_INPUT4);
  if (PulseState4==0) {
    Pulses4 = CurrentTime4 - StartTime4;
    StartTime4 = CurrentTime4;
  }
  if (PulseState4==1) {
    StartTime4 = CurrentTime4;
  }
}


// For the given channel, configure it to output a PWM signal with the pulse
// width specified in microseconds
// Assuming the packet is 11000us, divide the pulse value by 11000 and
// multiply by 4096 for the 12 bit PWM engine

void configure_output_channel_pulse(int channel, int pulse)
{

  // Limit minimum pulse width to 1100us
  if (pulse < 1100)
    pulse = 1100;

  // Limit maximum pulse width to 1900us
  if (pulse > 1900)
    pulse = 1900;

  pulse = pulse * 4096 / 11111; // Output 90 of these pulses per second
  pwm.setPWM(channel, 0, pulse);
}


void setup()
{
  // Traxxas Lighting Setup
  

  // Radio Channel Frequency Measurement Inputs
  pinMode(FREQUENCY_INPUT, INPUT_PULLUP);
  pinMode(FREQUENCY_INPUT2, INPUT_PULLUP);
  pinMode(FREQUENCY_INPUT3, INPUT_PULLUP);
  pinMode(FREQUENCY_INPUT4, INPUT_PULLUP);


  // Configure interrupts for RC Frequency Measurement
  // Steering / Channel 1
  attachInterrupt(digitalPinToInterrupt(FREQUENCY_INPUT), PulseTimer, CHANGE);

  // Throttle / Channel 2
  attachInterrupt(digitalPinToInterrupt(FREQUENCY_INPUT2), PulseTimer2, CHANGE);

  // Channel 3 
  attachInterrupt(digitalPinToInterrupt(FREQUENCY_INPUT3), PulseTimer3, CHANGE);

  // Calibration / Channel 4
  attachInterrupt(digitalPinToInterrupt(FREQUENCY_INPUT4), PulseTimer4, CHANGE);


  // TFT Display Setup
  pinMode(PIN_POWER_ON, OUTPUT);    // enables the LCD and to run on battery
  pinMode(PIN_LCD_BL, OUTPUT);      // controls the LCD backlight
  digitalWrite(PIN_POWER_ON, HIGH); // Turn LCD Power On
  digitalWrite(PIN_LCD_BL, HIGH);   // Turn LCD Backlight On


  // VIN Measurement Setup
  pinMode(VIN_MEAS, INPUT);


  // Serial Setup
  // Note:  Serial communication is slow, even at 115,200bps.  Enable only for
  // debugging
  Serial.begin(115200);  // be sure to set USB CDC On Boot: "Enabled" ???
  Serial.println("Serial Console Ready");


  // I2C (Wire) Setup

  // Internal I2C Bus
  // Rest of sensors are connected here:  Touch, Accelerometer, Magnetometer, 
  // anything connected to the QWIIC connector on the 
  Wire.begin(18, 17);  
  Wire.setClock(1000000UL); // Running at 1MHz bus speed
  
  // External I2C Bus
  // Connect GPS and possibly MicroSD card here
  Wire1.begin(43,44);
  Wire1.setClock(750000UL); // Running at 750kHz bus speed


  // Configure PCA9685 PWM driver
  // pwm.reset(); // WARNING!!  Doing this first seems to brick the microcontroller!
  // To recover, put the microcontroller into bootloader mode:
  // a)  hold the Boot0 button.
  // b)  Press and release the black RESET button.
  // c)  Release left button
  // d)  Upload your program to the microcontroller
  // e)  Press and release reset
  pwm.begin();
  delay(100); // Wait for the sensors to start functioning after instantiation
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(90);
  pwm.setOutputMode(true);

  // Splash Screen
  tft.init();
  // Note:  tft.setRotation() is how we specify the orientation of the text
  // displayed on the screen:
  // 1 - Horizontal, up with circle to right
  // 3 - Horizontal, up is with circle to the left.
  // 2 and 4 would also work, showing the text across the short dimension of
  // the LCD
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);              // Clear Screen
  tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Colors
  tft.setTextFont(2);                     // Splash Screen
  tft.drawString("OtterPilot Hardware Test", 20, 30, 4);
  tft.setTextColor(TFT_BLUE, TFT_BLACK); // Set Text and Background Colors
  tft.drawString("v3.7.5", 115, 70, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Colors
  // tft.drawString("JC Edition", 110, 110, 4);
  tft.drawString("crazyuncleburton.com", 35, 110, 4);
  delay(3000);
}


void loop()
{
  // Normal Screen Splash
  tft.fillScreen(TFT_BLACK);             // Clear Screen
  tft.setCursor(0, 5, 2);               // Set Cursor Position and Font
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set Text and Background Color
  tft.print("Testing Hardware: ");


  // VIN Voltage Measurement
  // We chose the divider to get 1.625V out with an input of 6.5V (divide by 4)
  // G10_VIN_MEAS value is VIN * 0.25000
  // So VIN = measured value of G10_VIN_MEAS in volts * 4

  int vin_measured_steps = 0;
  analogReadResolution(12); // The ESP32-S3 processor in our LilyGo microprocessor board is 12bit resolution
  vin_measured_steps = analogRead(VIN_MEAS); // returns an int from 0-4095 mapped to 3.3v means 805.86uV/step.  
  if (vin_measured_steps > 1551) 
  { // 1551.137 steps = 5.000V.  2016.497 Steps = 6.50V.  Report red if less than 5.000
    // Good.  
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
  } else {
    // Bad
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
  }
  
  // Account for divider
  float vin = 0;
  vin = vin_measured_steps * 4 * 0.00080586;

  // Report voltage results
  tft.print(" ");
  tft.print("VIN = ");
  tft.printf("%2.3f", vin);
  tft.println("V");


  // I2C Bus Scan
  // Report Results to screen and serial
  int address;
  int error;


  // Check for 0x40 / PWM Controller
  address=64;
  Wire.beginTransmission(address);  //Data transmission to the specified device address starts.   
  error = Wire.endTransmission();
  if(error==0) 
  { 
    // Detected.  Print its address in hex
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
      tft.println("PCA9685 PWM Controller Detected: 0x40 ");
      Serial.println("PCA9685 PWM Controller Detected: 0x40");
  } else {
    // Not Detected
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
      tft.println("PCA9685 PWM Controller Not Detected: 0x40 ");
      Serial.println("PCA9685 PWM Controller Not Detected: 0x40");
  }
  delay(10);

  // Check for 0x6A / 6DOF IMU
  address = 106;
  Wire.beginTransmission(address);  //Data transmission to the specified device address starts.   
  error = Wire.endTransmission();
  if(error==0) 
  { 
    // Detected.  Print its address in hex
        tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
      tft.println("LSM6DSOX - 6DOF IMU Detected: 0x6A");
      Serial.println("LSM6DSOX - 6DOF IMU: 0x6A");
  } else {
    // Not Detected
        tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
      tft.println("LSM6DSOX - 6DOF IMU Not Detected: 0x6A ");
      Serial.println("LSM6DSOX - 6DOF IMU Not Detected: 0x6A");
  }
  delay(10);

  // Checking 0x10 PA1010D GPS
  // Need to check on Internal and External Bus and Report Accordingly
  address = 16;
  Wire.beginTransmission(address);  //Data transmission to the specified device address starts.   
  error = Wire.endTransmission();
  if(error==0) 
  { 
    // Detected on internal bus.  Print its address in hex
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set Text and Background Color
    tft.println("Adafruit PA1010D GPS Detected:  0x10 Int");
    Serial.println("Adafruit PA1010D GPS Detected: 0x10 Int");
  } else {
    // Not detected on internal bus, check external bus.
    Wire1.beginTransmission(address);  //Data transmission to the specified device address starts.   
    error = Wire1.endTransmission();
    if(error==0) 
    { 
      // Detected on external bus.  Print its address in hex
      tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
      tft.println("Adafruit PA1010D GPS Detected: 0x10 Ext");
      Serial.println("Adafruit PA1010D GPS Detected: 0x10 Ext");
    } else {
      // Not Detected
      tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
      tft.println("Adafruit PA1010D GPS Not Detected");
      Serial.println("Adafruit PA1010D GPS Not Detected");
    }
  }
  delay(10);

  // Checking 0x2A SparkFun OpenLog MicroSD
  // Need to check on Internal and External Bus and Report Accordingly
  tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
  address = 42;
  Wire.beginTransmission(address);  //Data transmission to the specified device address starts.   
  error = Wire.endTransmission();
  if(error==0) 
  { 
    // Detected on internal bus.  Print its address in hex
    tft.println("SparkFun OpenLog Detected:  0x2A Int");
    Serial.println("SparkFun OpenLog Detected:  0x2A Int");
  } else {
    // Not detected on internal bus, check external bus.
    Wire1.beginTransmission(address);  //Data transmission to the specified device address starts.   
    error = Wire1.endTransmission();
    if(error==0) 
    { 
      // Detected on external bus.  Print its address in hex
      tft.println("SparkFun OpenLog Detected:  0x2A Ext");
      Serial.println("SparkFun OpenLog Detected:  0x2A Ext");
    } else {
      // Not Detected
      tft.println("SparkFun OpenLog Not Detected.");
      Serial.println("SparkFun OpenLog Not Detected.");
    }
  }
  delay(10);


  // Steering / Throttle / Aux / Cal Channel Tests

  // Command PWM Chip to output test pulses
  configure_output_channel_pulse(0, 1500); // SERVO1 (Steer) -> on PWM Controller Output PWM0
  configure_output_channel_pulse(1, 1500); // SERVO2 (Throttle) -> on PWM Controller Output PWM1
  configure_output_channel_pulse(2, 1500); // SERVO3 (Transmission) -> on PWM Controller Output PWM2
  configure_output_channel_pulse(3, 1500); // SERVO4 (whatever) -> on PWM Controller Output PWM3
  configure_output_channel_pulse(4, 1500); // Cal Ch on PWM Controller Output PWM4
  delay(300);

  // Test Frequency measurement input from Steering
  // Pulses has the current measured pulse width for PWM0 (connected to RXA Steering)
  if (abs((int)Pulses - 1500) <= 100 )
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
    tft.print("Steering or Aux Channel Measured: ");
    tft.print((int)Pulses);
    tft.println("uS");
    Serial.print("Steering or Aux Channel Measured: ");
    Serial.println((int)Pulses);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
    tft.print("Steering or Aux Channel Measured: ");
    tft.println((int)Pulses);
    Serial.print("Steering or Aux Channel Measured: ");
    Serial.println((int)Pulses);
  }

  // Test Frequency measurement input from Throttle
  // Pulses2 has current measured pulse width for PWM1 (connected to RXT Throttle)
  if (abs((int)Pulses2 - 1500) <= 100 )
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
    tft.print("Throttle Channel Measured: ");
    tft.print((int)Pulses2);
    tft.println("uS");
    Serial.print("Throttle Channel Measured: ");
    Serial.println((int)Pulses2);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
    tft.print("Throttle Channel Measured: ");
    tft.println((int)Pulses2);
    Serial.print("Throttle Channel Measured: ");
    Serial.println((int)Pulses2);
  }

  // Test Frequency measurement input from Third Channel
  // Pulses3 has current measured pulse width for PWM2 (connected to RX3)
  if (abs((int)Pulses3 - 1500) <= 100 )
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
    tft.print("Channel 3 Measured: ");
    tft.print((int)Pulses3);
    tft.println("uS");
    Serial.print("Channel 3 Measured: ");
    Serial.println((int)Pulses3);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
    tft.print("Channel 3 Measured: ");
    tft.println((int)Pulses3);
    Serial.print("Channel 3 Measured: ");
    Serial.println((int)Pulses3);
  }

  // Pulses4 has the current measured pulse width for PWM2 (connected internally to CAL Input)
  if (abs((int)Pulses4 - 1500) <= 100 )
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set Text and Background Color
    tft.print("Cal Channel Measured: ");
    tft.print((int)Pulses4);
    tft.println("uS");
    Serial.print("Cal Measured: ");
    Serial.println((int)Pulses4);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set Text and Background Color
    tft.print("Cal Channel Measured: ");
    tft.print((int)Pulses4);
    tft.println("uS");
    Serial.print("Cal Channel Measured: ");
    Serial.println((int)Pulses4);
  }

  // Wait some time for user to see screen.  Then loop and do it all again.
  delay(2500);
  tft.fillScreen(TFT_BLACK); // Clear Screen

}