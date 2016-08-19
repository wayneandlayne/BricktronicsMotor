// Bricktronics Example: MotorPositionControlBricktronicsMegashield
// http://www.wayneandlayne.com/bricktronics
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
// Hardware used:
// * Wayne and Layne Bricktronics Megashield
//   https://store.wayneandlayne.com/products/bricktronics-megashield-kit.html
// * LEGO NXT or EV3 Motor
//
// Software libraries used:
// * Wayne and Layne BricktronicsMegashield library
//   https://github.com/wayneandlayne/BricktronicsMegashield
// * Wayne and Layne BricktronicsMotor library
//   https://github.com/wayneandlayne/BricktronicsMotor
//
// Written in 2016 by Matthew Beckler and Adam Wolf for Wayne and Layne, LLC
// To the extent possible under law, the author(s) have dedicated all
//   copyright and related and neighboring rights to this software to the
//   public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along
//   with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>. 


// Include the Bricktronics libraries
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>


// Select the desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
BricktronicsMotor m(BricktronicsMegashield::MOTOR_1);


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

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

