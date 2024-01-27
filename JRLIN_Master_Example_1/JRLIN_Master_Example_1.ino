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

#include "JRLinMaster.h"
#include "JRLinFrameBuffer.h"
#include "JRLinHelper.h"
#include <AltSoftSerial.h>

#define DEBUG_MODE_MAIN (0)

/**PINS */
#define FAULT_PIN (8)
#define CS_PIN (7) // not recommended to be hardwired in schematic; operation mode of transceiver needs to be in synch with the micro-controller; hardwiring could lead to false LIN traffic during startup
#define ONBOARD_LED (13)
#define BUTTON_PIN (4)

/* Schedule size euqals 500ms; 5ms per slot; 100 slots*/
#define SCHEDULE_MAX_SLOTS (100)
#define SCHEDULE_TBASE_MILLIS (5)  //milliseconds, x slots = cycle time

#define SCHEDULE_ENTRIES (1)
#define SLOT_TIMEOUT_MICROS (25000) //25000 microsec = 25 ms

AltSoftSerial altSerial;

JRLINMaster lin;
JRLINFrameBuffer framelist[SCHEDULE_ENTRIES]; //reserve memory for 3 frames
static JRLINFrameBuffer *ptrCurrentFrame = NULL;

// timer to aboard waiting for a frame
volatile unsigned long frame_start = 0;

// used for creating a schedule grid
unsigned long delayStart = 0; // the time the delay started
bool timerRunning = false; // true if still waiting for delay to finish
uint8_t slotCount = -1;

enum MasterStates {
  sendId      //request an id interaction
  , receiveId // collect the nodes reply
} masterState;

void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  pinMode(FAULT_PIN, OUTPUT);
  digitalWrite(FAULT_PIN, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW); //dark

  /**
     Init Debug Serial
  */
  altSerial.begin(19200); // needs to be turned on in advance during runtime!

#if DEBUG_MODE_MAIN >= 1
  altSerial.println("HW UART LIN Master Node");
#endif

  /**
     Configure frame list schedule; don't forget to adjust SCHEDULE_ENTRIES
  */

  framelist[0].init(0x30, fb_id_send, 4, 10);
  framelist[0].data[0] = 0xFF;
  framelist[0].data[1] = 0x00;
  framelist[0].data[2] = 0xFF;
  framelist[0].data[3] = 0x00;
  framelist[0].len = 4;

  lin.setFrameList(framelist, SCHEDULE_ENTRIES);
  ptrCurrentFrame = NULL;

  /**
     Configure timeouts
  */
  delayStart = millis();   // start delay
  timerRunning = true; // start or stop the time checking

  /**
     Configure State Machine and Schedule table
  */
  masterState = sendId;

  // Start the LIN machine
  lin.begin(9600, 1, &altSerial);
}

void loop()
{
  if (masterState == sendId) {
    // if lin machine is idle and does not need to pick up messages
    // check if it is time to send a message
    if (timerRunning && ((millis() - delayStart) >= SCHEDULE_TBASE_MILLIS)) {
      delayStart += SCHEDULE_TBASE_MILLIS; // this prevents drift in the delays

      ++slotCount;
      if (slotCount >= SCHEDULE_MAX_SLOTS) slotCount = 0; // slot schedule size

      // pick the msg to be send in the current slot
      ptrCurrentFrame = NULL;
      for (int i = 0; i < SCHEDULE_ENTRIES; i++) { //framelist size
        if (framelist[i].slot == slotCount) {
          ptrCurrentFrame = &framelist[i];
          break;
        }
      } // for filter

      if (ptrCurrentFrame != NULL) {
        /**
           MASTER SENDS ID
        */
        lin.master_send(ptrCurrentFrame);
        Serial.flush();

        if (ptrCurrentFrame->frametype == fb_id_receive) {
          // receive reply!
          //  Serial.flush(); //empty the UART
          frame_start = micros();
          masterState = receiveId; //instant reply
        } else if (ptrCurrentFrame->frametype == fb_id_send) {
          /**
            MASTER SENDS DATA PAYLOAD
          */
          lin.slave_send(ptrCurrentFrame);
          Serial.flush();
        } else {
          // don't do anything
        }
      } // != NULL, frame to be sent in the slot was found
    } else {
      // nothing to send; no work on the LIN bus has to be done
      // use resources for regular application computation
      compute();
    }
  } // send ID


  if (masterState == receiveId) { //jump instantly to the reply generation if required
    if (ptrCurrentFrame != NULL) {

      static LINMasterRead ret = lm_ret_unknown;
      /**
        MASTER RECEIVES DATA PAYLOAD
      */
      ret = lin.master_receive(ptrCurrentFrame, frame_start, SLOT_TIMEOUT_MICROS ); //FILL UP THE RIGHT BUFFER!

      switch (ret) {
        case lm_ret_processing:
          // keep processing
          break;

        case lm_ret_frame_valid:
          masterState = sendId;
#if DEBUG_MODE_MAIN >= 1
          JRLINFrameBuffer::debug_buffer_print(&altSerial, ptrCurrentFrame);
#endif
#if DEBUG_MODE_MAIN >= 2
          altSerial.println("V");
#endif
          break;

        case lm_ret_frame_timeout:
          masterState = sendId;
          break;

        case lm_ret_wrong_checksum:
          masterState = sendId;

#if DEBUG_MODE_MAIN >= 2
          altSerial.println("!CHK");
#endif
          break;
        default:
          masterState = sendId; // keep on going
          break;
      } // switch
    } // if ptrCurrentFrame != NULL
    else {
      // throw warning, pointer == NULL!
#if DEBUG_MODE_MAIN >= 2
      altSerial.println("Warning: ptrCurrentFrame == NULL!");
#endif
      masterState = sendId; // keep on going
    }
  } //received ID
}

void compute() {
  // if no work is needed on the LIN bus, do some application computation instead
  int sensorVal = digitalRead(BUTTON_PIN);
  if (sensorVal == HIGH) {
    digitalWrite(13, LOW);
    framelist[0].data[3] = 0x0;
  } else {
    digitalWrite(13, HIGH);
    framelist[0].data[3] = 0x1;
  }
}
