// Bricktronics Example: MotorPositionControlInterrupt
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor.
//
// This example demonstrates advanced motor position control, which uses the
// PID (proportional, integral, derivative) control algorithm to precisely
// drive the motor to the desired rotation position.
//
// This example uses the FlexiTimer2 library, which automatically
// calls our motor's update() function every 25 milliseconds. This allows us
// to do other things instead of managing the motor's update() calls. We can
// simply set the motor to a new desired position whenever we like, and the
// interrupt will automatically call the motor's update() function periodically.
//
// This example uses the FlexiTimer2 library to generate the interrupts, which
// breaks the analogWrite (PWM) output on the following pins:
//      Arduino UNO: Pins 3 and 11
//          On the Bricktronics Shield, this will only interfere with the
//          TIP120 transitor marked Q4 (you probably aren't using this.)
//      Arduino Mega: Pins 9 and 10
//          On the Bricktronics Megashield, this will only interfere with
//          motor 2, so don't use motor 2 with this FlexiTimer2 library.
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
// Helper libraries can be download from:
// https://www.pjrc.com/teensy/td_libs_Encoder.html
// https://github.com/br3ttb/Arduino-PID-Library/
//      Be sure to rename unzipped folder PID_v1
#include <Encoder.h>
#include <PID_v1.h>
#include <BricktronicsMotor.h>

// Install the FlexiTimer2 library from https://github.com/wimleers/flexitimer2
//      Be sure to rename the unzipped folder FlexiTimer2
#include <FlexiTimer2.h>


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

  // Set up the interrupt to occur every 25 ms
  FlexiTimer2::set(25, updateMotorInterrupt);
  FlexiTimer2::start();
}

// The FlexiTimer2 library (as configured in setup()) will call this
// function every 25 milliseconds. This function just calls m.update().
void updateMotorInterrupt(void)
{
    m.update();
    // If you have multiple motors, be sure to call all their update
    // functions here in the interrupt handler...
    // m2.update();
    // m3.update();
    // ...
}

void loop() 
{
  // The position control works by creating a desired rotation position (as
  // measured by the motor's position encoders), and then periodically calling
  // the m.update() function. The update function checks the motor's current
  // position, compares it to the desired position, and decides which way and
  // how fast to rotate the motor to reach that desired position. We need to
  // call m.update() periodically, and we've found that every 25-50ms works
  // pretty well, which is how we configured the FlexiTimer2 library in the
  // setup function above.

  // This statement doesn't actually move anything, yet.
  // It simply sets the motor's destination position (720 ticks per revolution).
  // 180 = one-quarter revolution in the "forward" direction
  Serial.println("Using goToPosition(180)...");
  m.goToPosition(180);

  // Since we have already set up the timer interrupt, we don't have to worry
  // about what we do in loop() here. The motor will correctly have its update()
  // function called at regular intervals.
  Serial.println("Calling delay(1000) works fine because the interrupt handles calling update().");
  delay(1000);


  Serial.println("Going to position 360...");
  m.goToPosition(360);

  Serial.println("Here is a long statement that will take a long time to transmit via serial...but our motor will keep working all the while!");

  // You can even try manually moving the motor during this long delay.
  // You'll find that the motor resists being moved, since the calls to update()
  // detect that the motor has been moved and works to restore its position to 360.

  Serial.println("Delaying for 10 seconds - You'll notice the motor is holding its position,");
  Serial.println("and will fight you if you try to manually move the motor.");
  delay(10000);
}

