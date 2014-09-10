/*
    Bricktronics library for LEGO NXT motors.

    Copyright (C) 2014 Adam Wolf, Matthew Beckler, John Baichtal

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

#include "BricktronicsMotor.h"

// This is the simplified constructor that allows you to specify only the
// five motor pins.
BricktronicsMotor::BricktronicsMotor(uint8_t enPin, uint8_t dirPin, uint8_t pwmPin, uint8_t encoderPin1, uint8_t encoderPin2):
             _enPin(enPin),
             _dirPin(dirPin),
             _pwmPin(pwmPin),
             _enabled(false),
             _rawSpeed(0),
             _pid(&_pidInput, &_pidOutput, &_pidSetpoint, BRICKTRONICS_MOTOR_PID_KP, BRICKTRONICS_MOTOR_PID_KI, BRICKTRONICS_MOTOR_PID_KD, REVERSE),
             _pidMode(BRICKTRONICS_MOTOR_PID_MODE_DISABLED),
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

// This is the complicated constructor that allows for overriding the
// low-level Arduino functions.
BricktronicsMotor::BricktronicsMotor(const BricktronicsMotorSettings &settings):
             _enPin(settings.enPin),
             _dirPin(settings.dirPin),
             _pwmPin(settings.pwmPin),
             _enabled(false),
             _rawSpeed(0),
             _pid(&_pidInput, &_pidOutput, &_pidSetpoint, BRICKTRONICS_MOTOR_PID_KP, BRICKTRONICS_MOTOR_PID_KI, BRICKTRONICS_MOTOR_PID_KD, REVERSE),
             _pidMode(BRICKTRONICS_MOTOR_PID_MODE_DISABLED),
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


int32_t BricktronicsMotor::getPosition(void)
{
    //delay(1);
    //int32_t x = _encoder.read();
    //Serial.println(x, HEX);
    //x = _encoder.read();
    //Serial.println(x, HEX);
    //return x;

    return _encoder.read();

    //return _encoder.encoder.position;
}

void BricktronicsMotor::setPosition(int32_t pos)
{
    _encoder.write(pos);
}



void BricktronicsMotor::begin(void)
{
    _pid.SetMode(AUTOMATIC);
    _enabled = true;
    stop();
    _pinMode(_dirPin, OUTPUT);
    _pinMode(_pwmPin, OUTPUT);
    _pinMode(_enPin, OUTPUT);
}

void BricktronicsMotor::enable(void)
{
    begin();
}

void BricktronicsMotor::disable(void)
{
    _enabled = false;
    _pinMode(_dirPin, INPUT);
    _pinMode(_pwmPin, INPUT);
    _pinMode(_enPin, INPUT);
}

void BricktronicsMotor::stop(void)
{
    _digitalWrite(_enPin, LOW);
    _digitalWrite(_dirPin, LOW);
    _digitalWrite(_pwmPin, LOW);
}


// RAW UNCONTROLLED SPEED FUNCTION
void BricktronicsMotor::rawSetSpeed(int16_t s)
{
    _rawSpeed = s;
    if (s == 0)
    {
        stop();
    }
    else if (s < 0)
    {
        _digitalWrite(_dirPin, HIGH);
        analogWrite(_pwmPin, 255 + s);
        _digitalWrite(_enPin, HIGH);
    }
    else
    {
        _digitalWrite(_dirPin, LOW);
        analogWrite(_pwmPin, s);
        _digitalWrite(_enPin, HIGH);
    }
}

int16_t BricktronicsMotor::rawGetSpeed(void)
{
    return _rawSpeed;
}


void BricktronicsMotor::goToPosition(int32_t position)
{
    // Swith our internal PID into position mode
    _pidMode = BRICKTRONICS_MOTOR_PID_MODE_POSITION;
    _pidSetpoint = position;
}

void BricktronicsMotor::goToPositionWait(int32_t position)
{
    goToPosition(position);
    while (!settledAtPosition(position))
    {
        update();
    }
    stop();
}

bool BricktronicsMotor::goToPositionWaitTimeout(int32_t position, uint32_t timeoutMS)
{
    goToPosition(position);
    timeoutMS += millis(); // future time when we timeout
    while ( !settledAtPosition(position) && (millis() < timeoutMS) )
    {
        update();
    }
    stop();
    if (millis() >= timeoutMS)
    {
        return false;
    }
    return true;
}

bool BricktronicsMotor::settledAtPosition(int32_t position)
{
    return (    (abs(getPosition() - position) < _epsilon)
             && (abs(_pidOutput) < BRICKTRONICS_MOTOR_PID_OUTPUT_SETTLED_THRESHOLD) );
}

void BricktronicsMotor::setAngleOutputMultiplier(int8_t multiplier)
{
    // Since the LEGO NXT motor encoders have 720 ticks per 360 degrees,
    // we have to double the user's specified multiplier.
    _angleMultiplier = multiplier << 1;
}

int32_t BricktronicsMotor::_getDestPositionFromAngle(int32_t angle)
{
    int16_t delta = (angle % 360) - getAngle();
    Serial.print("getAngle: ");
    Serial.println(getAngle());
    Serial.print("angle: ");
    Serial.println(angle);
    Serial.print("delta pre: ");
    Serial.println(delta);

    while (delta > 180)
    {
        delta -= 360;
    }
    while (delta < -180)
    {
        delta += 360;
    }
    Serial.print("delta post: ");
    Serial.println(delta);

    // Now, delta is between -180 and +180
    Serial.print("getPosition: ");
    Serial.println(getPosition());
    Serial.print("delta * _angleMultiplier: ");
    Serial.println(delta * _angleMultiplier);
    int32_t position = getPosition() + (delta * _angleMultiplier);
    Serial.print("position: ");
    Serial.println(position);
    return position;
}

void BricktronicsMotor::goToAngle(int32_t angle)
{
    goToPosition(_getDestPositionFromAngle(angle));
}

void BricktronicsMotor::goToAngleWait(int32_t angle)
{
    goToPositionWait(_getDestPositionFromAngle(angle));
}

bool BricktronicsMotor::goToAngleWaitTimeout(int32_t angle, uint32_t timeoutMS)
{
    return goToPositionWaitTimeout(_getDestPositionFromAngle(angle), timeoutMS);
}

uint16_t BricktronicsMotor::getAngle(void)
{
  return ( (getPosition() / _angleMultiplier) % 360 );
}

void BricktronicsMotor::setAngle(int32_t angle)
{
    setPosition((angle % 360) * _angleMultiplier);
}



void BricktronicsMotor::update(void)
{
    switch (_pidMode)
    {
        case BRICKTRONICS_MOTOR_PID_MODE_POSITION:
            _pidInput = _encoder.read();
            _pid.Compute();
            rawSetSpeed(_pidOutput);
            //Serial.print("pos: ");
            //Serial.println(_pidInput);
            //Serial.print("out: ");
            //Serial.println(_pidOutput);
            break;

        case BRICKTRONICS_MOTOR_PID_MODE_SPEED:
            // TODO how can we determine the current speed if this is being called frequently
            break;

        default: // includes BRICKTRONICS_MOTOR_PID_MODE_DISABLED
            break;
    }
}

void BricktronicsMotor::delayUpdateMS(int delayMS)
{
    unsigned long endTime = millis() + delayMS;
    while (millis() < endTime)
    {
        update();
        // We could put a delay(5) here, but the PID library already has a 
        // "sample time" parameter to only run so frequent, you know?
    }
}

void BricktronicsMotor::pidSetUpdateFrequencyMS(int timeMS)
{
    _pid.SetSampleTime(timeMS);
}

void BricktronicsMotor::pidPrintValues(void)
{
    Serial.print("SET:");
    Serial.println(_pidSetpoint);
    Serial.print("INP:");
    Serial.println(_pidInput);
    Serial.print("OUT:");
    Serial.println(_pidOutput);
}

double BricktronicsMotor::pidGetKp(void)
{
    return _pid.GetKp();
}

double BricktronicsMotor::pidGetKi(void)
{
    return _pid.GetKi();
}

double BricktronicsMotor::pidGetKd(void)
{
    return _pid.GetKd();
}

void BricktronicsMotor::pidSetKp(double Kp)
{
    _pid.SetTunings(Kp, _pid.GetKi(), _pid.GetKd());
}

void BricktronicsMotor::pidSetKi(double Ki)
{
    _pid.SetTunings(_pid.GetKp(), Ki, _pid.GetKd());
}

void BricktronicsMotor::pidSetKd(double Kd)
{
    _pid.SetTunings(_pid.GetKp(), _pid.GetKi(), Kd);
}

void BricktronicsMotor::pidSetTunings(double Kp, double Ki, double Kd)
{
    _pid.SetTunings(Kp, Ki, Kd);
}

