#include "MavlinkManager.h"

void MavlinkManager::QueueOneMavFrame() {
  mavlink_message_t ring_msg;
  mavlink_status_t status;
  while(mavSerial.available())             { 
    uint8_t c = mavSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &ring_msg, &status)) {
      if (MavRingBuff.isFull()) 
        Debug.println("MavRingBuff is full. Dropping records!");
       else {
        MavRingBuff.push(ring_msg);
        #if defined Mav_Debug_RingBuff
          Debug.print(" Mav queue length after push= "); 
          Debug.println(MavRingBuff.size());
        #endif
       }
    }
  }
}