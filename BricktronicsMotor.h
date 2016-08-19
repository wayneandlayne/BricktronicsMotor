/*
   BricktronicsMotor v1.2 - A software library for LEGO NXT motors.

   Copyright (C) 2015 Adam Wolf, Matthew Beckler, John Baichtal

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

   Wayne and Layne invests time and resources providing this open-source
   code, please support W&L and open-source hardware by purchasing products
   from https://store.wayneandlayne.com/ - Thanks!

   Wayne and Layne, LLC and our products are not connected to or endorsed by the LEGO Group.
   LEGO, Mindstorms, and NXT are trademarks of the LEGO Group.
*/


#ifndef BRICKTRONICSMOTOR_H
#define BRICKTRONICSMOTOR_H

// Arduino header files
#include <stdint.h>
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Library header files
#include "utility/Encoder.h"
#include "utility/PID_v1.h"
#include "utility/BricktronicsSettings.h"

// These are the default motor PID values for P, I, and D.
// Tested on an unloaded NXT 2.0 motor, you may want to adjust these
// PID constants based on whatever you have connect to your motor.
#define BRICKTRONICS_MOTOR_PID_KP                           2.64
#define BRICKTRONICS_MOTOR_PID_KI                           14.432
#define BRICKTRONICS_MOTOR_PID_KD                           0.1207317073

// We can have different motor modes
#define BRICKTRONICS_MOTOR_MODE_COAST                       0
#define BRICKTRONICS_MOTOR_MODE_BRAKE                       1
#define BRICKTRONICS_MOTOR_MODE_FIXED_DRIVE                 2
#define BRICKTRONICS_MOTOR_MODE_PID_POSITION                3
#define BRICKTRONICS_MOTOR_MODE_PID_SPEED                   4

// Sample time - Call update() as often as you can, but it will only update
// as often as this value. Can be updated by the user at runtime if desired.
#define BRICKTRONICS_MOTOR_PID_SAMPLE_TIME_MS               50

#define BRICKTRONICS_MOTOR_ANGLE_MULTIPLIER_DEFAULT         1
// Epsilon is used to evaluate if we are at a desired position (abs(getPosition() - desiredPosition) < epsilon)
#define BRICKTRONICS_MOTOR_EPSILON_DEFAULT                  5
// This constant is used to determine if the PID algorithm has settled down enough to stop calling update() and just call brake()
// Used to try and avoid overshoot by stopping PID updates too early.
#define BRICKTRONICS_MOTOR_PID_OUTPUT_SETTLED_THRESHOLD     30

class BricktronicsMotor
{
    public:
        // Constructor - Simple constructor accepts the five motor pins
        BricktronicsMotor(uint8_t enPin,
                          uint8_t dirPin,
                          uint8_t pwmPin,
                          uint8_t encoderPin1,
                          uint8_t encoderPin2):
            _enPin(enPin),
            _dirPin(dirPin),
            _pwmPin(pwmPin),
            _rawSpeed(0),
            _reversed(false),
            _pid(&_pidInput, &_pidOutput, &_pidSetpoint, BRICKTRONICS_MOTOR_PID_KP, BRICKTRONICS_MOTOR_PID_KI, BRICKTRONICS_MOTOR_PID_KD, DIRECT),
            _pidKp(BRICKTRONICS_MOTOR_PID_KP),
            _pidKi(BRICKTRONICS_MOTOR_PID_KI),
            _pidKd(BRICKTRONICS_MOTOR_PID_KD),
            _mode(BRICKTRONICS_MOTOR_MODE_COAST),
            _encoder(encoderPin1, encoderPin2),
            _angleMultiplier(BRICKTRONICS_MOTOR_ANGLE_MULTIPLIER_DEFAULT),
            _epsilon(BRICKTRONICS_MOTOR_EPSILON_DEFAULT),
            _pinMode(&::pinMode),
            _digitalWrite(&::digitalWrite),
            _digitalRead(&::digitalRead)
        {
            _pid.SetSampleTime(BRICKTRONICS_MOTOR_PID_SAMPLE_TIME_MS);
            _pid.SetOutputLimits(-255, +255);
        }

        // Constructor - Advanced constructor accepts a BricktronicsMotorSettings struct
        // to also override the low-level Arduino functions.
        BricktronicsMotor(const BricktronicsMotorSettings &settings):
            _enPin(settings.enPin),
            _dirPin(settings.dirPin),
            _pwmPin(settings.pwmPin),
            _rawSpeed(0),
            _reversed(settings.reversedMotorDrive), // See note below about why this is set to true for Bricktronics Shield
            _pid(&_pidInput, &_pidOutput, &_pidSetpoint, BRICKTRONICS_MOTOR_PID_KP, BRICKTRONICS_MOTOR_PID_KI, BRICKTRONICS_MOTOR_PID_KD, DIRECT),
            _pidKp(BRICKTRONICS_MOTOR_PID_KP),
            _pidKi(BRICKTRONICS_MOTOR_PID_KI),
            _pidKd(BRICKTRONICS_MOTOR_PID_KD),
            _mode(BRICKTRONICS_MOTOR_MODE_COAST),
            _encoder(settings.encoderPin1, settings.encoderPin2),
            _angleMultiplier(BRICKTRONICS_MOTOR_ANGLE_MULTIPLIER_DEFAULT),
            _epsilon(BRICKTRONICS_MOTOR_EPSILON_DEFAULT),
            _pinMode(settings.pinMode),
            _digitalWrite(settings.digitalWrite),
            _digitalRead(settings.digitalRead)
        {
            _pid.SetSampleTime(BRICKTRONICS_MOTOR_PID_SAMPLE_TIME_MS);
            _pid.SetOutputLimits(-255, +255);
        }

        // Set the dir/pwm/en pins as outputs and sets the motor to coast.
        void begin(void)
        {
            _pid.SetMode(AUTOMATIC);
            _pinMode(_dirPin, OUTPUT);
            _pinMode(_pwmPin, OUTPUT);
            _pinMode(_enPin, OUTPUT);
            coast();
        }

        // Disconnects the motor windings. Excess back-EMF will be shunted
        // through the motor driver's protection diodes and/or the body diodes
        // in the H-bridge. This will not actively slow-down the motor.
        // The L293D(D) chip used for Bricktronics v1 isn't great for
        // doing coasting, so disabling the drivers is the best we can do.
        void coast(void)
        {
            _mode = BRICKTRONICS_MOTOR_MODE_COAST;
            _digitalWrite(_dirPin, LOW);
            _digitalWrite(_pwmPin, LOW);
            _digitalWrite(_enPin, LOW);
        }

        // Shorts the motor windings, which will quickly bring it to a stop.
        // This mode does not lock the motor in place electrically or mechanically.
        // You may also be interested in the hold() function below.
        // The L293D(D) chip used for Bricktronics v1 isn't great for
        // doing braking, so enabling the drivers and shorting A/B is the best we can do.
        void brake(void)
        {
            _mode = BRICKTRONICS_MOTOR_MODE_BRAKE;
            _digitalWrite(_dirPin, LOW);
            _digitalWrite(_pwmPin, LOW);
            _digitalWrite(_enPin, HIGH);
        }

        // Similar to brake(), but this function sets up a goToPosition() for the
        // current position, effectively locking the motor in place. That is, it
        // will resist any efforts to turn the motor, and will constantly try to
        // restore the motor to the position it had when you called hold().
        // Just like with goToPosition(), you need to periodically call update().
        void hold(void)
        {
            goToPosition(getPosition());
        }

        // Read the encoder's current position.
        int32_t getPosition(void)
        {
            return(_encoder.read());
        }
        // Write the encoder's current position - This will mess up any control in progress!
        //     This only sets the number corresponding to the motor's current position.
        //     Usually you just want to reset the position to zero.
        void setPosition(int32_t pos)
        {
            _encoder.write(pos);
        }

        // Motors have some slop in their encoder output readings, so this function
        // can be used to make a "close enough?" check. The epsilon value can be get/set
        // using the functions below, and is used in the settledAtPosition check.
        // This function also checks to ensure that the PID algorithm has settled down enough
        // (that is, _pidOutput < BRICKTRONICS_MOTOR_PID_OUTPUT_SETTLED_THRESHOLD) that we
        // can just brake() without having to worry about coasting through the setpoint.
        bool settledAtPosition(int32_t position)
        {
            return(    (abs(getPosition() - position) < _epsilon)
                    && (abs(_pidOutput) < BRICKTRONICS_MOTOR_PID_OUTPUT_SETTLED_THRESHOLD) );
        }

        void setEpsilon(uint8_t epsilon)
        {
            _epsilon = epsilon;
        }
        uint8_t getEpsilon(void)
        {
            return( _epsilon );
        }


        // Some of the functions below need to periodically check on the
        // motor's operation and update how fast and/or which direction to
        // drive the motor. Use this update() function to do that. Call this
        // function as often as you can, since it will only actually update as
        // often as the frequency setpoint (defaults to 50ms), which can be
        // updated below.
        void update(void)
        {
            switch( _mode )
            {
                case BRICKTRONICS_MOTOR_MODE_PID_POSITION:
                    _pidInput = _encoder.read();
                    _pid.Compute();
                    _rawSetSpeed(_pidOutput);
                    /*
                    Serial.print("_pidOutput: ");
                    Serial.print(_pidOutput);
                    Serial.print(", pos: ");
                    Serial.println(_pidInput);
                    */
                    break;

                case BRICKTRONICS_MOTOR_MODE_PID_SPEED:
                    // TODO create implementation of speed control
                    break;

                default:
                    // None of the other motor modes need periodic updating.
                    break;
            }
        }

        // This function periodically calls update() until delayMS
        // milliseconds have elapsed. Useful if you have nothing else to do.
        void delayUpdateMS(uint32_t delayMS)
        {
            unsigned long endTime = millis() + delayMS;
            while (millis() < endTime)
            {
                update();
                // We could put a delay(5) here, but the PID library already has a 
                // "sample time" parameter to only run so frequent, you know?
            }
        }


        // PID related functions
        // Update the maximum frequency at which the PID algorithm will actually update.
        void pidSetUpdateFrequencyMS(int timeMS)
        {
            _pid.SetSampleTime(timeMS);
        }

        // Print out the PID values to the serial port
        void pidPrintValues(void)
        {
            Serial.print("SET:");
            Serial.println(_pidSetpoint);
            Serial.print("INP:");
            Serial.println(_pidInput);
            Serial.print("OUT:");
            Serial.println(_pidOutput);
        }

        // Functions for getting and setting the PID tuning parameters
        double pidGetKp(void)
        {
            return _pidKp;
        }
        double pidGetKi(void)
        {
            return _pidKi;
        }
        double pidGetKd(void)
        {
            return _pidKd;
        }

        void pidSetTunings(double Kp, double Ki, double Kd)
        {
            _pidKp = Kp;
            _pidKi = Ki;
            _pidKd = Kd;
            pidUpdateTunings();
        }

        void pidUpdateTunings(void)
        {
            _pid.SetTunings(_pidKp, _pidKi, _pidKd);
        }

        void pidSetKp(double Kp)
        {
            _pidKp = Kp;
            pidUpdateTunings();
        }
        void pidSetKi(double Ki)
        {
            _pidKi = Ki;
            pidUpdateTunings();
        }
        void pidSetKd(double Kd)
        {
            _pidKd = Kd;
            pidUpdateTunings();
        }


        // Raw, uncontrolled speed settings
        // There is no monitoring or control of the speed here,
        // just set a fixed drive strength between -255 and +255 (0 = brake).
        void setFixedDrive(int16_t s)
        {
            _mode = BRICKTRONICS_MOTOR_MODE_FIXED_DRIVE;
            _rawSpeed = s;
            _rawSetSpeed(_rawSpeed);
        }

        // Retrieves the previously-set fixed drive speed
        int16_t getFixedDrive(void)
        {
            return _rawSpeed;
        }


        // Position control functions
        void goToPosition(int32_t position)
        {
            // Swith our internal PID into position mode
            _mode = BRICKTRONICS_MOTOR_MODE_PID_POSITION;
            _pidSetpoint = position;
        }

        // Go to the specified position using PID, but wait for the specified number of milliseconds
        void goToPositionWaitForDelay(int32_t position, uint32_t delayMS)
        {
            goToPosition(position);
            delayUpdateMS(delayMS);
        }

        // Go to the specified position using PID, but wait until the motor arrives
        void goToPositionWaitForArrival(int32_t position)
        {
            goToPosition(position);
            while( !settledAtPosition( position ) )
            {
                update();
            }
        }

        // Same as above, but return after timeoutMS milliseconds in case it gets stuck
        // Returns true if we made it to position, false if we had a timeout
        bool goToPositionWaitForArrivalOrTimeout(int32_t position, uint32_t timeoutMS)
        {
            goToPosition(position);
            timeoutMS += millis(); // future time when we timeout
            while( !settledAtPosition( position ) && ( millis() < timeoutMS ) )
            {
                update();
            }
            if( millis() >= timeoutMS )
            {
                return false;
            }
            return true;
        }

        // Angle control functions - 0 - 359, handles discontinuity nicely.
        // Can specify any angle, positive or negative. If you say 
        // "go to angle 721" it will be the same as "go to angle 1".
        // Similarly, "go to angle -60" will be "go to angle 300".
        // If you want "go 45 degrees clockwise from here", try using
        // m.goToAngle(m.getAngle() + 45);
        // The fancy angle math is in _getDestPositionFromAngle, these functions just
        // use the result of that function with their corresponding goToPosition*.
        void goToAngle(int32_t angle)
        {
            goToPosition(_getDestPositionFromAngle(angle));
        }

        // Go to the specified angle using PID, but wait for the specified number of milliseconds
        void goToAngleWaitForDelay(int32_t angle, uint32_t delayMS)
        {
            goToPositionWaitForDelay(_getDestPositionFromAngle(angle), delayMS);
        }

        // Go to the specified angle using PID, but wait until the motor arrives
        void goToAngleWaitForArrival(int32_t angle)
        {
            goToPositionWaitForArrival(_getDestPositionFromAngle(angle));
        }

        // Same as above, but return after timeoutMS milliseconds in case it gets stuck
        // Returns true if we made it to the desired position, false if we had a timeout
        bool goToAngleWaitForArrivalOrTimeout(int32_t angle, uint32_t timeoutMS)
        {
            return goToPositionWaitForArrivalOrTimeout(_getDestPositionFromAngle(angle), timeoutMS);
        }

        // Returns the current angle (0-359)
        uint16_t getAngle(void)
        {
            return( ( getPosition() / _angleMultiplier ) % 360 );
        }

        // Updates the encoder position to be the specified angle
        void setAngle(int32_t angle)
        {
            setPosition( ( angle % 360 ) * _angleMultiplier );
        }

        // For the angle control, the user can specify a different multiplier
        // between motor encoder ticks and "output rotations", defaults to 1.
        // Use this setting if your motor is connected to a gear train that
        // makes a different number of motor rotations per output rotation.
        // For example, if you have a 5:1 gear train between your motor and
        // the final output, then you can specify this value as 5.
        // Negative numbers should work just fine.
        // TODO Using integer angles means we can't do sub-degree positioning,
        // which only really becomes noticable if we scale-up the output by a lot.
        void setAngleOutputMultiplier(int8_t multiplier)
        {
            // Since the LEGO NXT motor encoders have 720 ticks per 360 degrees,
            // we have to double the user's specified multiplier.
            _angleMultiplier = multiplier << 1;
        }



    //private:
        // We really don't like to hide things inside private,
        // but if we did, these would be the private items.
        uint8_t _enPin;
        uint8_t _dirPin;
        uint8_t _pwmPin;

        uint8_t _mode;
        uint16_t _rawSpeed;

        // PID variables
        PID _pid;
        double _pidSetpoint, _pidInput, _pidOutput;
        double _pidKp, _pidKi, _pidKd;

        // Tracks the position of the motor
        Encoder _encoder;

        // Check out the comments above for setAngleOutputMultiplier()
        int8_t _angleMultiplier;

        // Raw, uncontrolled speed settings
        // There is no monitoring or control of the speed here,
        // just set a fixed drive strength between -255 and +255.
        // Be sure to check out coast(), brake(), and hold().
        void _rawSetSpeed(int16_t s)
        {
            if( _reversed )
            {
                s = -s;
            }

            if( s < 0 )
            {
                _digitalWrite(_dirPin, HIGH);
                analogWrite(_pwmPin, 255 + s);
            }
            else
            {
                _digitalWrite(_dirPin, LOW);
                analogWrite(_pwmPin, s);
            }

            // Enable drivers
            _digitalWrite(_enPin, HIGH);
        }

        // If you reverse the speed/direction pins, the motor runs backwards.
        // Use this value to switch how your speed settings are applied.
        // See _rawSetSpeed above.
        // The Bricktronics Shield has the speed/direction signals reversed
        // from the canonical naming used on the Bricktronics Motor Driver and
        // Bricktronics Megashield, so the Bricktronics Shield constructor
        // sets this to true.
        bool _reversed;


        // Uses the current angle to determine the desired destination
        // position of the motor. Used in the goToAngle* functions.
        int32_t _getDestPositionFromAngle(int32_t angle)
        {
            int16_t delta = (angle % 360) - getAngle();

            while( delta > 180 )
            {
                delta -= 360;
            }
            while( delta < -180 )
            {
                delta += 360;
            }

            // Now, delta is between -180 and +180
            return( getPosition() + ( delta * _angleMultiplier ) );
        }

        // When checking if the motor has reached a certain position,
        // there is likely to be a small amount of "slop", and it would be
        // unreasonable to stall forever trying to get to position 180 when
        // we are "only" at position 179. This value can be read/set using
        // the functions above, and is used in position check like this:
        // abs(getPosition() - checkPosition) > _epsilon
        uint8_t _epsilon;

        // For the Bricktronics Shield, which has an I2C I/O expander chip, we need a way to
        // override some common Arduino functions. We use function pointers here to handle this.
        // For the non-Bricktronics Shield cases, the simple constructor above provides the built-in functions.
        void (*_pinMode)(uint8_t, uint8_t);
        void (*_digitalWrite)(uint8_t, uint8_t);
        int (*_digitalRead)(uint8_t);
};

#endif // #ifdef BRICKTRONICSMOTOR_H

