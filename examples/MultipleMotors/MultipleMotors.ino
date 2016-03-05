// Bricktronics Example: MultipleMotors
// http://www.wayneandlayne.com/bricktronics
// This example uses multiple LEGO NXT motors
//
// This example shows how to use PID with multiple motors.
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
//      Be sure to rename unzipped folder PID_v1
#include <Encoder.h>
#include <PID_v1.h>
#include <BricktronicsMotor.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call BricktronicsShield::begin() in the setup() function below.
// You also need to install the Adafruit MCP23017 library:
//      https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
// Select the motor port (MOTOR_1 or MOTOR_2) in the constructor below.
//
// Config 1 - CFG_WNL_BS
//#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <BricktronicsShield.h>
//BricktronicsMotor m1(BricktronicsShield::MOTOR_1);
//BricktronicsMotor m2(BricktronicsShield::MOTOR_2);
// Config end

// 2. With a Bricktronics Megashield - Include these lines below but do not
// call BricktronicsShield::begin() in the setup() function below. Select the
// desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
//
// Config 2 - CFG_WNL_BMS
//#include <BricktronicsMegashield.h>
//BricktronicsMotor m1(BricktronicsMegashield::MOTOR_1);
//BricktronicsMotor m2(BricktronicsMegashield::MOTOR_2);
// Config end

// 3. With a Bricktronics Motor Driver - No additional #includes needed,
// just update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, encoderPin1, encoderPin2
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (supports analogWrite)
//      Uno:       pins 3, 5, 6, 9, 10, and 11
//      Mega 2560: pins 2 to 13 and 44 to 46
// B. There are three ways to connect the encoder pins (labeled T1/T2 on the board).
// ** Best performance: Both signals are connected to true interrupt pins (listed below).
// ** Good performance: The FIRST signal (T1) is connected to an interrupt pin, the second signa is a regular pin. This is the mode used for the Bricktronics Shield/Megashield. For this mode it is CRITICAL that the true interrupt pin is used for T1 and not T2.
// ** Low performance: Both signals are connected to non-interrupt pins.
// Regardless of which performance mode used, you MUST list the pin T1 before T2 in
//   the constructor, otherwise the encoder will be connected backwards and the
//   PID algorithm will get all confused and freak out.
// Location of true interrupt pins:
//      Uno:       pins 2 and 3
//      Mega 2560: pins 2, 3, 21, 20, 19, and 18
//
// Config 3 - CFG_WNL_NS
//BricktronicsMotor m1(4, 5, 10, 2, 8);
//BricktronicsMotor m2(6, 7, 11, 3, 9);
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
  m1.begin();
  m2.begin();
}

// To use PID control with multiple motors, we need to call the
// update() function for all motors periodically, so we can't just
// use the built-in goTo{Position,Angle}WaitFor*() functions.
// The easiest way is to make a super-update function that calls all
// the motors' update() functions at once.
void updateMotors()
{
    // Call all motors' update() functions.
    // If you add more motors or call them by different names,
    // be sure to update this function!
    m1.update();
    m2.update();
}

// we can also create a function similar to the single motor's delayUpdateMS
void delayUpdateMotors(uint32_t delayMS)
{
    unsigned long endTime = millis() + delayMS;
    while( millis() < endTime )
    {
        updateMotors();
    }
}

void loop() 
{
    Serial.print("Sending motors to +/- 180 ticks = +/- 90 degrees...");
    m1.goToPosition(180);
    m2.goToPosition(-180);
    delayUpdateMotors(2000);
    Serial.println("done");

    // If you want to wait until both motors have reached a position:
    m1.goToPosition(360);
    m2.goToPosition(-720);
    Serial.print("Waiting until m1 is at 360 and m2 is at -720...");
    // If you want to wait until either motor arrives, change && to || below:
    while( !m1.settledAtPosition(360) && !m2.settledAtPosition(-720) )
    {
        updateMotors();
    }
    Serial.println("done");
    delayUpdateMotors(2000);
}

