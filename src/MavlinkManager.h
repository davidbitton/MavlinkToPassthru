
#ifndef MAVLINKMANAGER_H_
#define MAVLINKMANAGER_H_

#include <CircularBuffer.h>
#include <ardupilotmega/mavlink.h>

#include "config.h"

class MavlinkManager {
private:
  CircularBuffer<mavlink_message_t, 10> MavRingBuff;
  mavlink_message_t msg;

public:
  MavlinkManager();
  void QueueOneMavFrame();
  void DecodeOneMavFrame();
  void Aux_ReceiveAndForward();
};

#endif