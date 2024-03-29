/****
    CloudArchitect
    Copyright (C) 2020  JRBridge Ltd
    Version 0.0.1

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.    
***/

#ifndef __JRLIN_HELPER_H
#define __JRLIN_HELPER_H


#include "Arduino.h"

class JRLINHelper {
  public:
    // For Lin 1.X "start" should = 0, for Lin 2.X "start" should be the addr byte.
    static uint8_t dataChecksum(const uint8_t* message, char nBytes, uint16_t start = 0);
    static uint8_t addrParity(uint8_t addr);
};

#endif
