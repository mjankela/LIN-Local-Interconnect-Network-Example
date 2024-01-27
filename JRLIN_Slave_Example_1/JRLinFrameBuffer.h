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

#ifndef __JRLIN_FRAME_BUFFER_H
#define __JRLIN_FRAME_BUFFER_H

#include <AltSoftSerial.h>
#include "Arduino.h"

/**
 Union, 0 pure master request, 1 send with buffer content , 2 receive into buffer
 Rename to JRLIN_MESSAGE, or JRLIN_FRAME_DATA, JRLIN_FRAME_BUFFER, MSG_TYPE
 on/off on sending, receiving
*/

enum JRLINFrameBufferType {
    fb_id_receive // receive into buffer, 
  , fb_id_send    // send with content, 
  , fb_id_only    // only trigger id
};


// not identical to the master frame buffer!
class JRLINFrameBuffer {
  public:
    JRLINFrameBuffer();
    JRLINFrameBufferType   frametype; 

    uint8_t   addr;
    uint8_t   calc_addr;

    // needed for receive & sending frames
    uint8_t   len;
    uint8_t   data[8];
    uint8_t   cksm;
    uint8_t   exp_cksm;
    uint8_t   updated;
    unsigned long timestamp;

    void init( uint8_t buf_addr, JRLINFrameBufferType buf_frametype, uint8_t buf_len);
    static void debug_buffer_print(AltSoftSerial *_dbgSerial, JRLINFrameBuffer *buf);    //static ?
};


#endif
