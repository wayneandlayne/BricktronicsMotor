// Bricktronics Example: MotorPositionControlInterruptBricktronicsMegashield
// http://www.wayneandlayne.com/bricktronics
//
// This example demonstrates advanced motor position control, which uses the
// PID (proportional, integral, derivative) control algorithm to precisely
// drive the motor to the desired rotation position.
//
// Each microcontroller has one or more internal timer modules that can be
// configured to generate a software interrupt on a regular schedule. The
// Arduino system configures Timer0 to have an interrupt every millisecond,
// which is updates the variables used by the millis() function.
// This example adds another interrupt to Timer0 to occur every millisecond,
// which we can use to periodically call our motor's update() function every
// 50 milliseconds. By using Timer0 OC0A to generate this interrupt, it uses
// the same internal arduino chip hardware that would be used to generate a
// PWM output signal on these pins:
// * Uno: D6 - Only used as part of sensor port 3, which does not need PWM.
// * Mega: D13 - Not used by the Bricktronics Shield or Megashield.
// If you call analogWrite with D6 on Uno or D13 on Mega it will override
// this new Timer0 OC0A interrupt and cause problems. On non-AVR platforms
// like Teensy or ChipKit, things will probably work differently. Post in the
// W&L forums and we'll figure it out: https://discuss.wayneandlayne.com/
//
// Using an interrupt allows us to do other things in our main loop without
// managing the motor's update() calls. We can simply set the motor to a new
// desired position whenever we like, and the interrupt will automatically
// call the motor's update() function periodically.
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
// Written in 2017 by Matthew Beckler and Adam Wolf for Wayne and Layne, LLC
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

  // The Arduino system configures Timer0 to produce an interrupt every
  // millisecond, which is what makes millis() work.
  // The interrupts happen when the 8-bit timer overflows from 255 to 0,
  // and we can add a second interrupt to occur as the timer count passes
  // a specified compare value (OCR0A), which will also happen every one
  // millisecond.
  OCR0A = 0x7F;
  TIMSK0 |= _BV(OCIE0A);

  // TODO Timer0 does pwm for mega pins 4 and 13. Pin 4 is used for Motor 6,
  // so let's figure out which of compare A or B is used for pin 4, and use the
  // other compare for our 1khz interrupt. Unless analogWrite on pin 4 would mess up
  // both compare A and B settings?
}

// This function will be called every millisecond.
// It just calls update() for each motor.
ISR(TIMER0_COMPA_vect)
{
    static unsigned char count_ms = 0;
    if (++count_ms == 50)
    {
        m.update();
        // If you have multiple motors, be sure to call all their update
        // functions here in the interrupt handler...
        // m2.update();
        // m3.update();
        // ...
        count_ms = 0;
    }
}

void loop()
{
  // The position control works by creating a desired rotation position (as
  // measured by the motor's position encoders), and then periodically calling
  // the m.update() function. The update function checks the motor's current
  // position, compares it to the desired position, and decides which way and
  // how fast to rotate the motor to reach that desired position. We need to
  // call m.update() periodically so it can recompute the motor settings.

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

