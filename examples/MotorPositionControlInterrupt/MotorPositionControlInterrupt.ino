// Bricktronics Example: MotorPositionControlInterrupt
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor.
//
// This example demonstrates advanced motor position control, which uses the
// PID (proportional, integral, derivative) control algorithm to precisely
// drive the motor until it reaches the desired rotation position.
//
// This example uses the FlexiTimer2 library, which is used to automatically
// call our motor's update() function every 50 milliseconds. This allows us
// to do other things instead of managing the motor's update() calls. We can
// simply set the motor to a new desired position whenever we like, and the
// interrupt will automatically call the motor's update() function periodically.
// You can download this library from:
//      http://playground.arduino.cc/Main/FlexiTimer2
//
// This example uses Timer 2 to generate the interrupts, which breaks the
// PWM output on the following pins:
//      Arduino UNO: Pins 3 and 11
//          On the Bricktronics Shield, this will only interfere with the
//          TIP120 transitor marked Q4 (you probably aren't using this.)
//      Arduino Mega: Pins 9 and 10
//          On the Bricktronics Megashield, this will only interfere with
//          Motor 2, so don't use that motor with this FlexiTimer2 library.
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).


// Include the Bricktronics Motor library
#include <Motor.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call Bricktronics::begin() in the setup() function below. Select the
// motor port (BS_MOTOR_1 or BS_MOTOR_2) in the constructor below.
//
//#include <Wire.h>
//#include <Bricktronics2.h>
//Motor m = Motor(Bricktronics::BS_MOTOR_1);

// 2. With a Bricktronics Megashield - Include these lines below but do not
// call Bricktronics::being() in the setup() function below. Select the
// desired motor port (BMS_MOTOR_1 through BMS_MOTOR_6) in the constructor below.
//
//#include <Bricktronics2.h>
//Motor m = Motor(Bricktronics::BMS_MOTOR_1);

// 3. With a Bricktronics Motor Driver - No additional #includes needed,
// just update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, tachPinA, tachPinB
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (analogWrite)
// Uno:       pins 3, 5, 6, 9, 10, and 11
// Mega 2560: pins 2 to 13 and 44 to 46
// B. At least one of tachPinA/B needs to be an actual interrupt pin (not just
// a "pin change interrupt" pin).
// Uno:       pins 2 and 3
// Mega 2560: 2, 3, 21, 20, 19, and 18
//
//Motor m = Motor(3, 4, 10, 2, 5);


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

  // Only call this line if you are using a Bricktronics Shield,
  // otherwise leave it commented-out.
  //Bricktronics::begin();

  // Initialize the motor connections
  m.begin();

  // Set up the interrupt to occur every 50 ms
  FlexiTimer2::set(50, updateMotorInterrupt);
  FlexiTimer2::start();
}

// The FlexiTimer2 library (once configured in setup()) will call this
// function every 50 milliseconds. This function just calls m.update().
void updateMotorInterrupt(void)
{
    m.update();
}

void loop() 
{
  // The position control works by creating a desired rotation position (as
  // measured by the motor's position encoders), and then periodically calling
  // the m.update() function. The update function checks the motor's current
  // position, compares it to the desired position, and decides which way and
  // how fast to rotate the motor to reach that desired position. We need to
  // call m.update() periodically, and we've found that every 50ms works
  // pretty well, which is how we configured the FlexiTimer2 library in the
  // setup function above.

  // This statement doesn't actually move anything, yet.
  // It simply sets the motor's destination position (720 ticks per revolution).
  // 360 = one-half revolution in the "forward" direction
  m.goToPosition(360);

  // Since we have already set up the timer interrupt, we don't have to worry
  // about what we do in loop() here. The motor will correctly have its update()
  // function called at regular intervals.
  delay(random(1000));

  m.goToPosition(1080);

  Serial.println("Here is a long statement that will take a long time to transmit via serial...but our motor will keep working all the while!");

  delay(random(10000));

}

