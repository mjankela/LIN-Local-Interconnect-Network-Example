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

#include "Arduino.h"
#include "JRLinMaster.h"
#include "JRLinHelper.h"

JRLINMaster::JRLINMaster()
{
  _speed = 0;
  _ptrDbgSerial = NULL; //pointer to debug serial

  _framelist = NULL; //our schedule list
  _txPin = 1; //digital pin used for the break signal

  _serialOn = false; // UART is active?

  _tbit  = 0;
  _nominalFrameTime  = 0;
  _timeout = 0;

  _brk_end = 0;
  _brk_begin = 0;

  _linMasterStateMachine = lm_unknown; // init the local state machine
}

/***
    State Machine Configuration Settings
*/
void JRLINMaster::setFrameList(JRLINFrameBuffer *framelist, int fcount) {
  _framelist = framelist;
  _fcount = fcount;
}

void JRLINMaster::begin(uint32_t speed, uint8_t break_pin , AltSoftSerial *dbgSerial) // could be inlined for performance
{
  _speed = speed;
  _txPin = break_pin;
  _ptrDbgSerial = dbgSerial;

  /**
      Calculate Static Timeout Values
  */
  _tbit  = 100000 / (uint32_t)_speed;                             // Not quite in uSec, I'm saving an extra 10 to change a 1.4 (40%) to 14 below...;
  _nominalFrameTime  = ((34 * _tbit) + 90 * _tbit);     // 90 = 10*max # payload bytes + checksum (9)
  _timeout = LIN_TIMEOUT_IN_FRAMES * 14 * _nominalFrameTime;  // 14 is the specced addtl 40% space above normal*10 -- the extra 10 is just pulled out of the 1000000 needed to convert to uSec (so that there are no decimal #s), ms would be divide by ten!

  _brk_end = (1000000UL / ((uint32_t)_speed)); //52ms - 19200
  _brk_begin = _brk_end * LIN_BREAK_DURATION; //52ms*Duration - 19200

  _linMasterStateMachine = lm_sync; // init the local state machine

  Serial.begin(_speed);
  _serialOn = true;

#if DEBUG_MODE_MASTER >= 1
  if (_ptrDbgSerial) _ptrDbgSerial->println("Lin statemachine started..." );
#endif
#if DEBUG_MODE_MASTER >= 2
  if (_ptrDbgSerial) {
    _ptrDbgSerial->println("Configuration:" );
    _ptrDbgSerial->print("_tbit: " );
    _ptrDbgSerial->println(_tbit);
    _ptrDbgSerial->print("_nominalFrameTime: " );
    _ptrDbgSerial->println(_nominalFrameTime );
    _ptrDbgSerial->print("_timeout: " );
    _ptrDbgSerial->println(_timeout );
    _ptrDbgSerial->print("_brk_end: " );
    _ptrDbgSerial->println(_brk_end);
    _ptrDbgSerial->print("_brk_begin: " );
    _ptrDbgSerial->println(_brk_begin );
  }
#endif
}

/**
    LIN Master Communication methods
*/

// Generate a BREAK signal (a low signal for longer than a byte) across the serial line,
// triggered over a separate pin request ! UART cannot sent this request
void JRLINMaster::serialBreak(void)
{
  if (_serialOn) {
    Serial.end();
    _serialOn = false;
  }

  pinMode(_txPin, OUTPUT);
  digitalWrite(_txPin, LOW);  // Send BREAK
  delayMicroseconds(_brk_begin);

  digitalWrite(_txPin, HIGH);  // BREAK delimiter
  delayMicroseconds(_brk_end);

  Serial.begin(_speed);
  _serialOn = true;
}

void JRLINMaster::master_send(JRLINFrameBuffer *ptrFrame) {
  if (ptrFrame == NULL) return;
  static uint8_t c = -1;
    
  serialBreak();       // Generate the low signal that exceeds 1 char.
  Serial.write(0x55);  // Sync byte
  Serial.write(ptrFrame->calc_addr);  // ID byte
}


void JRLINMaster::slave_send(JRLINFrameBuffer *ptrFrame, uint8_t proto) {
  if (ptrFrame == NULL) return;
  
//  uint8_t addrbyte = (ptrFrame->addr & 0x3f) | JRLINHelper::addrParity(ptrFrame->addr);  //should be already calculated !
  uint8_t cksum = JRLINHelper::dataChecksum(ptrFrame->data, ptrFrame->len, (proto == 1) ? 0 : ptrFrame->calc_addr); // should be already calculated

  Serial.write(ptrFrame->data, ptrFrame->len); // data bytes
  Serial.write(cksum);  // checksum
}


LINMasterRead JRLINMaster::master_receive( JRLINFrameBuffer *ptrFrame, unsigned long frame_start, unsigned long max_frame_duration) {

  // Check if we are still within our allowed time slot  
  if ((micros() - frame_start) > max_frame_duration) {
    _linMasterStateMachine = lm_sync;
    Serial.flush();
    return lm_ret_frame_timeout; // cannot fall through ! otherwise the serial code will trigger
  }
  
  if (!Serial.available()) return lm_ret_processing;
  if (ptrFrame == NULL) return lm_ret_frame_buffer_not_found; //error ! move it before the time check?

#if DEBUG_MODE_MASTER >= 2
  if (_ptrDbgSerial != NULL)_ptrDbgSerial->println("[M]");
#endif

  static uint8_t c = -1;
  while (Serial.available()) {
    c = Serial.read();

    switch ( _linMasterStateMachine) {
      case lm_sync:
        _linMasterStateMachine = lm_id;
        break;
      case  lm_id:
        _linMasterStateMachine = lm_get_data;
        _charcounter = 0;
        break;
      case lm_get_data:
        ptrFrame->data[_charcounter] = c;

#if DEBUG_MODE_MASTER >= 2
        if (_ptrDbgSerial != NULL)_ptrDbgSerial->println(c, HEX);
#endif

        if (_charcounter == (ptrFrame->len - 1))
        {
          _linMasterStateMachine = lm_get_chksm;
        } else {
          _linMasterStateMachine = lm_get_data;
        }
        _charcounter++;

        break;

      case lm_get_chksm:
        ptrFrame->exp_cksm = c;
        ptrFrame->cksm = JRLINHelper::dataChecksum(ptrFrame->data, ptrFrame->len, ptrFrame->calc_addr);

        Serial.flush();
        _linMasterStateMachine = lm_sync;

        ptrFrame->updated = ++ptrFrame->updated;
        ptrFrame->timestamp = millis();

        if (ptrFrame->exp_cksm == ptrFrame->cksm) {
          return lm_ret_frame_valid;
        } else {
          return lm_ret_wrong_checksum;
        }

        break;
      default:
        break;
    }
  } // while

  return lm_ret_processing;
}
