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

#include "JRLinHelper.h"

/***
 *  JRLin Helper Class 
 * 
 */

/* Lin defines its checksum as an inverted 8 bit sum with carry */
uint8_t JRLINHelper::dataChecksum(const uint8_t* message, char nBytes, uint16_t sum)
{
  while (nBytes-- > 0) sum += *(message++);

  // Add the carry
  while (sum >> 8) // In case adding the carry causes another carry
    sum = (sum & 255) + (sum >> 8);
    
  return (~sum);
}

/* Create the Lin ID parity */
#define BIT(data,shift) ((addr&(1<<shift))>>shift)

uint8_t JRLINHelper::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr, 0) ^ BIT(addr, 1) ^ BIT(addr, 2) ^ BIT(addr, 4);
  uint8_t p1 = ~(BIT(addr, 1) ^ BIT(addr, 3) ^ BIT(addr, 4) ^ BIT(addr, 5));

  return (p0 | (p1 << 1)) << 6;
}
