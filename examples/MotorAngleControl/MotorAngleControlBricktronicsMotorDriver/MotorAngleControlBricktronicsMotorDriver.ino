// Bricktronics Example: MotorAngleControlBricktronicsMotorDriver
// http://www.wayneandlayne.com/bricktronics
//
// This example demonstrates advanced motor angle control, which uses the
// PID (proportional, integral, derivative) control algorithm to precisely
// drive the motor until it reaches the desired rotation angle.
//
// Hardware used:
// * Wayne and Layne Bricktronics Motor Driver
//   https://store.wayneandlayne.com/products/bricktronics-motor-driver.html
// * LEGO NXT or EV3 Motor
//
// Software libraries used:
// * Wayne and Layne BricktronicsMotor library
//   https://github.com/wayneandlayne/BricktronicsMotor
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).
//
// Written in 2016 by Matthew Beckler and Adam Wolf for Wayne and Layne, LLC
// To the extent possible under law, the author(s) have dedicated all
//   copyright and related and neighboring rights to this software to the
//   public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along
//   with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>. 


// Include the Bricktronics libraries
#include <BricktronicsMotor.h>


// Update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, encoderPin1, encoderPin2
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (that is, it supports analogWrite)
//      Uno:       pins 3, 5, 6, 9, 10, and 11
//      Mega 2560: pins 2 - 13 and 44 - 46
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
BricktronicsMotor m(3, 4, 10, 2, 5);


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

  // Initialize the motor connections
  m.begin();
}

void loop() 
{
  // The angle control works by creating a desired rotation angle (as
  // measured by the motor's position encoders), and then periodically calling
  // the m.update() function. The update function checks the motor's current
  // position, compares it to the desired position, and decides which way and
  // how fast to rotate the motor to reach that desired position. You need to
  // call m.update() periodically, and we've found that every 25-50ms works
  // pretty well. Since the internal PID library has a built-in rate limiting,
  // it is simplest to just call m.update() as often as you can, and it will
  // just work. You can adjust the internal update rate-limit by calling
  // m.pidSetUpdateFrequencyMS(50) to set the limit to every 50 milliseconds,
  // for example.

  // Angle control differs from position control in a few important ways. Angle
  // control only works with angles between 0 and 359 degrees, but properly
  // understands discontinuities and wrapping of angles. If you say "go to
  // angle 721" it will be the same as "go to angle 1". It also handles negative
  // angles: "go to angle -60" is equal to "go to angle 300". This is absolute
  // angle positioning, like if you were driving the hands on a clock's face.
  // If you want to say "go 45 degrees from the current position" you should try
  // m.goToAngle(m.getAngle() + 45);
  // The code is smart about going the "short way around". If you're at angle 10,
  // and want to go to angle 350, it will go 20 degrees counter-clockwise instead
  // of 340 degrees clockwise.

  // For the angle control, you can also specify a different multiplier
  // between motor encoder ticks and "output rotations", which defaults to 1.
  // Use this setting if your motor is connected to a gear train that
  // makes a different number of motor rotations per output rotation.
  // For example, if you have a 5:1 gear train between your motor and
  // the final output, then you can specify this value as 5.
  // Negative numbers should work just fine.
  m.setAngleOutputMultiplier(1);

  // Reset the current position to "angle 0"
  m.setAngle(0);

  // This statement doesn't actually move anything, yet.
  // It simply sets the motor's destination angle (360 degrees per revolution).
  m.goToAngle(30);

  // To actually move the motor to the desired destination angle, we need
  // to repeatedly call the update() function, which runs the PID algorithm
  // and decides how to move the motor in the next time interval.

  // If we have nothing else to do except wait for the motor to reach its
  // destination, we can use this shortcut function below, which repeatedly
  // calls the motor's update() function while waiting for the specified time
  // period (here, 1000 milliseconds). If the motor reaches the desired angle 
  // before 1000 ms, then we sit here until the time has expired.
  m.delayUpdateMS(1000);

  // The previous two statements can be combined into a single function call:
  //m.goToAngleWaitForDelay(30, 1000);

  // Now we want to move the motor to a different angle.
  // This time, we want to only wait as long as necessary for the motor to
  // reach the new desired angle. 
  m.goToAngleWaitForArrival(60);

  // One thing to worry about, is what if our motor gets jammed or stuck,
  // we probably don't want to get stuck in the previous function forever,
  // waiting until the motor finally reaches the desired setpoint, right?
  // This function is the same as the previous function, but it will return
  // once the desired angle is reached OR if the timeout expires. It returns
  // true if the angle was reached, and returns false if the timeout expired.
  m.goToAngleWaitForArrivalOrTimeout(90, 1000);
  
  // Let's go to a few more angles.
  // Notice that when we go from 330 to 30 it goes the "short way" instead of going back
  // the long way to 30.
  m.goToAngleWaitForArrivalOrTimeout(180, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(190, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(200, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(210, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(240, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(270, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(300, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(330, 1000);
  m.delayUpdateMS(1000);
  m.goToAngleWaitForArrivalOrTimeout(30, 1000);
  m.delayUpdateMS(1000);
  
  // Now, let's pretend we have other things to do, and can't simply use
  // one of the waiting functions. We can do something like this:

  // Set up our PID algorithm desired angle
  m.goToAngle(-90);
  // This is the time when we want to be done, here, 1 second from now.
  long endTimeOverall = millis() + 1000;
  while (millis() < endTimeOverall)
  {
    // Serial.println("Doing work (< 50ms each time) while motors are moving");
    //
    // doSomething();
    // if (digitalRead(myPin) == HIGH) {
    //   handleButtonPress();
    // }
    // doSomethingElse();

    m.update();
  }
  
  
  // 1080 mod 360 = 0, go back to the original position
  m.goToAngleWaitForArrivalOrTimeout(1080, 1000);
  m.delayUpdateMS(1000);
}

