
#ifndef FRSKYSPORT_H_
#define FRSKYSPORT_H_

#include <CircularBuffer.h>

class FrSkySport {
private:
  typedef struct {
    uint8_t severity;
    char text[50];
    uint8_t txtlth;
    bool simple;
  } ST_type;

  ST_type ST_record;
  CircularBuffer<ST_type, 10> MsgRingBuff;

public:
  FrSkySport(void);
  void Init();
  
  bool BufferIsFull();
};

#endif