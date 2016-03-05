// Bricktronics Example: MotorSingle
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor.
//
// This example starts the motor at an intermediate speed,
// then speeds it up to full speed, and does the same in reverse.
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).
//
// Written in 2015 by Matthew Beckler and Adam Wolf for Wayne and Layne, LLC
// To the extent possible under law, the author(s) have dedicated all
//   copyright and related and neighboring rights to this software to the
//   public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along
//   with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>. 


// Include the Bricktronics Motor library and helper libraries
// Helper libraries can be downloaded from:
// https://www.pjrc.com/teensy/td_libs_Encoder.html
// https://github.com/br3ttb/Arduino-PID-Library/
//	Be sure to rename unzipped folder PID_v1
#include <Encoder.h>
#include <PID_v1.h>
#include <BricktronicsMotor.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call BricktronicsShield::begin() in the setup() function below.
// You also need to install the Adafruit MCP23017 library:
//	https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
// Select the motor port (MOTOR_1 or MOTOR_2) in the constructor below.
//
// Config 1 - CFG_WNL_BS
//#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <BricktronicsShield.h>
//BricktronicsMotor m(BricktronicsShield::MOTOR_1);
// Config end

// 2. With a Bricktronics Megashield - Include these lines below but do not
// call BricktronicsShield::begin() in the setup() function below. Select the
// desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
//
// Config 2 - CFG_WNL_BMS
//#include <BricktronicsMegashield.h>
//BricktronicsMotor m(BricktronicsMegashield::MOTOR_1);
// Config end

// 3. With a Bricktronics Motor Driver - No additional #includes needed,
// just update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, encoderPin1, encoderPin2
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (that is, it supports analogWrite)
//      Uno:       pins 3, 5, 6, 9, 10, and 11
//      Mega 2560: pins 2 - 13 and 44 - 46
// B. There are three ways to connect the encoder pins (labeled T1/T2 on the board).
// ** Best performance: Both signals are connected to true interrupt pins (listed below).
// ** Good performance: The FIRST signal (T1) is connected to an interrupt pin, the second signal is a regular pin. This is the mode used for the Bricktronics Shield/Megashield. For this mode it is CRITICAL that the true interrupt pin is used for T1 and not T2.
// ** Low performance: Both signals are connected to non-interrupt pins.
// Regardless of which performance mode used, you MUST list the pin T1 before T2 in
//   the constructor, otherwise the encoder will be connected backwards and the
//   PID algorithm will get all confused and freak out.
// Location of true interrupt pins:
//      Uno:       pins 2 and 3
//      Mega 2560: pins 2, 3, 21, 20, 19, and 18
//
// Config 3 - CFG_WNL_NS
//BricktronicsMotor m(3, 4, 10, 2, 5);
// Config end


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

  // Only call this line if you are using a Bricktronics Shield,
  // otherwise leave it commented-out.
  // Config 1 - CFG_WNL_BS
  //BricktronicsShield::begin();
  // Config end

  // Initialize the motor connections
  m.begin();
}

void loop() 
{
  Serial.println("Going forward.");
  m.setFixedDrive(75);
  delay(1000);
  
  m.setFixedDrive(255);
  delay(1000);

  Serial.println("Going in reverse.");
  m.setFixedDrive(-75);
  delay(1000);
  
  m.setFixedDrive(-255);
  delay(1000);
}

