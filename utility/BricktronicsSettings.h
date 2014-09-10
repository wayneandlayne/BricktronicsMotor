/*
    Struct definitions for motor and sensor settings.
    Provides a compact way to transfer motor and sensor settings
    from the Bricktronics class to the motor and sensor class constructors.

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

#ifndef BRICKTRONICSSETTINGS_H
#define BRICKTRONICSSETTINGS_H

// Arduino header files
#include <inttypes.h>

typedef struct BricktronicsMotorSettings
{
   uint8_t enPin;
   uint8_t dirPin;
   uint8_t pwmPin;
   uint8_t encoderPin1;
   uint8_t encoderPin2;
   void (*pinMode)(uint8_t, uint8_t);
   void (*digitalWrite)(uint8_t, uint8_t);
   int (*digitalRead)(uint8_t);
} BricktronicsMotorSettings;

typedef struct BricktronicsSensorSettings
{
   uint8_t ANA;
   uint8_t DA;
   uint8_t DB;
   void (*pinMode)(uint8_t, uint8_t);
   void (*digitalWrite)(uint8_t, uint8_t);
   int (*digitalRead)(uint8_t);
} BricktronicsSensorSettings;

#endif // #ifndef BRICKTRONICSSETTINGS_H

