// Bricktronics Example: MotorPositionControl
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor.
//
// This example demonstrates advanced motor position control, which uses the
// PID (proportional, integral, derivative) control algorithm to precisely
// drive the motor until it reaches the desired rotation position.
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
}

void loop() 
{
  Serial.println("");
  
  // The position control works by creating a desired rotation position (as
  // measured by the motor's position encoders), and then periodically calling
  // the m.update() function. The update function checks the motor's current
  // position, compares it to the desired position, and decides which way and
  // how fast to rotate the motor to reach that desired position. You need to
  // call m.update() periodically, and we've found that every 50ms works
  // pretty well. Since the internal PID library has a built-in rate limiting,
  // it is simplest to just call m.update() as often as you can, and it will
  // just work. You can adjust the internal update rate-limit by calling
  // m.pidSetUpdateFrequencyMS(50) to set the limit to every 50 milliseconds,
  // for example.

  // This statement doesn't actually move anything, yet.
  // It simply sets the motor's destination position (720 ticks per revolution).
  // 180 = one-quarter revolution in the "forward" direction
  m.goToPosition(180);
  Serial.print("Using goToPosition(180) with delayUpdateMS...");

  // To actually move the motor to the desired destination position, we need
  // to repeatedly call the update() function, which runs the PID algorithm
  // and decides how to move the motor in the next time interval.

  // If we have nothing else to do except wait for the motor to reach its
  // destination, we can use this shortcut function below, which repeatedly
  // calls the motor's update() function while waiting for the specified time
  // period (here, 1000 milliseconds). If the motor reaches the desired position
  // before 1000 ms, then we sit here until the time has expired.
  m.delayUpdateMS(1000);
  Serial.println("done");
  m.delayUpdateMS(2000);

  // The two functions above can be combined using this single function:
  Serial.print("Using goToPositionWaitForDelay(360)...");
  m.goToPositionWaitForDelay(360, 1000);
  Serial.println("done");
  m.delayUpdateMS(2000);

  // We can check the motor's current position:
  Serial.print("Current position: ");
  Serial.println(m.getPosition());

  // Now we want to move the motor to a different position.
  // This time, we want to only wait as long as necessary for the motor to
  // reach the new desired position. This is very similar to how you work with
  // motors in the NXT environment.
  Serial.print("Using goToPositionWaitForArrival(540)...");
  m.goToPositionWaitForArrival(540);
  Serial.println("done");
  m.delayUpdateMS(2000);


  // One thing to worry about, is what if our motor gets jammed or stuck,
  // we probably don't want to get stuck in the previous function forever,
  // waiting until the motor finally reaches the desired setpoint, right?
  // This function is the same as the previous function, but it will return
  // once the desired position is reached OR if the timeout expires. It returns
  // true if the position was reached, and returns false if the timeout expired.
  Serial.print("Using goToPositionWaitForArrivalOrTimeout(720)...");
  bool positionReached = m.goToPositionWaitForArrivalOrTimeout(720, 5000);
  if (positionReached)
  {
    Serial.println("reached position");
  }
  else
  {
    Serial.println("timeout");
  }
  m.delayUpdateMS(2000);


  // Now, let's pretend we have other things to do, and can't simply use
  // one of the waiting functions. We can do something like this:

  // Set the desired position
  m.goToPosition(0);

  // This is the time when we want to be done, 1 second from now.
  long endTimeOverall = millis() + 1000;
  Serial.print("Using manual loop with millis() back to position 0...");
  while (millis() < endTimeOverall)
  {
    // doSomething();
    // if (digitalRead(myPin) == HIGH) {
    //   handleButtonPress();
    // }
    // doSomethingElse();

    m.update();
  }
  Serial.println("done");
  m.delayUpdateMS(2000);


  // There are also a few functions for very basic motor control

  // Brake
  // Shorts the motor windings, which will quickly bring it to a stop.
  // This mode does not lock the motor in place electrically or mechanically.
  // You may also be interested in the hold() function below.
  m.setFixedDrive(255);
  Serial.println("Full speed ahead!");
  delay(100);
  Serial.println("Turning on dynamic brake...");
  m.brake();
  Serial.println("done - Notice that you can still turn the motor by hand.");
  delay(5000);

  // Coast
  // Disconnects the motor windings. Excess back-EMF will be shunted
  // through the motor driver's protection diodes and/or the body diodes
  // in the H-bridge. This will not actively slow-down the motor.
  m.setFixedDrive(-255);
  Serial.println("Full speed ahead!");
  delay(100);
  Serial.println("Turning on coast...");
  m.coast();
  Serial.println("done - Notice that you can still turn the motor by hand.");
  delay(5000);

  // Hold
  // Similar to brake(), but this function sets up a goToPosition() for the
  // current position, effectively locking the motor in place. That is, it
  // will resist any efforts to turn the motor, and will constantly try to
  // restore the motor to it's position when you call hold(). Just like
  // goToPosition(), you need to periodically call update().
  m.setFixedDrive(-255);
  Serial.println("Full speed ahead!");
  delay(100);
  Serial.println("Turning on hold...");
  m.hold();
  Serial.println("done - Notice that it's difficult to turn the motor by hand");
  Serial.println("and the motor restores its original hold position.");
  Serial.println("You do need to keep calling update() to make this work.");
  m.delayUpdateMS(5000);

  // Reset motor to position 0
  m.goToPositionWaitForDelay(0, 2000);
}

