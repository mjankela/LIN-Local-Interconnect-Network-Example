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

#ifndef __JRLIN_SLAVE_H
#define __JRLIN_SLACE_H

#include "Arduino.h"
#include "JRLinFrameBuffer.h"

#define DEBUG_MODE_SLAVE (0) // 0 no debug, 1 standard debug, 2 extended debug


#define LIN_BREAK_DURATION    (15)   // Number of bits in the break.
#define LIN_TIMEOUT_IN_FRAMES (2)    // Wait this many max frame times before declaring a read timeout. In microseconds!

// READBACK, ALIGN datatypes, Align FrameBuffer for Slave and Master!, 
// Check Comments in code!, Turn on off the Master Node!, Check the Return variable in the SlaveReturn

enum LINStates {
  unknown //shall be never reached
  , off
  , detected_break
  , wait_sync_byte
  , get_id
  , get_data
  , get_chksm
  , reply_to_id_empty_and_terminate
  , empty_and_terminate
  , empty_and_terminate_wrongchecksum
  , empty_and_terminate_timeout
  , empty_and_terminate_no_id_match
};

enum SlaveStates {
  frameProcessing
  , frameValid
  , frameSlotTimeout
  , frameWrongChecksum
  , frameNoIdMatch
  , frameIDReplied
};

enum BREAKState {
  waiting
  , pindown_indicated
  , valid
};

class JRLINSlave {
  protected:
    AltSoftSerial *_ptrDbgSerial;
    
    bool _serialOn;  // UART is on or off
    uint32_t _speed; // UART speed

    uint8_t _buffer_count; //hide as static ? Need to count the currently parsed UART buffer

    LINStates _linStateMachine;

    JRLINFrameBuffer *_framelist; //pointer to the schedule list to process
    JRLINFrameBuffer *_ptrCurrentFrame;
    uint16_t _fcount; //framebuffer size

    void _terminate();
    void _slave_send(uint8_t addr, const uint8_t* message, uint8_t nBytes, uint8_t proto = 2);

  public:
    JRLINSlave();
    volatile BREAKState breakSymbol; //externally set through the interrupt routine

    /** Configuration API */
    void config(uint32_t speed, AltSoftSerial *dbgSerial = NULL); // merge config with begin! also set the debug console to begin!
    void setFrameList(JRLINFrameBuffer *framelist, uint16_t fcount);
    
    /** Control API */
    void begin();
    SlaveStates slave_receive(unsigned long frame_start, unsigned long max_frame_duration);  // only public API

};

#endif
