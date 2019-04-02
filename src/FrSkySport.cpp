#include "FrSkySport.h"

FrSkySport::FrSkySport(void) {

}

void FrSkySport::Init() {

}

bool FrSkySport::BufferIsFull() {
    return MsgRingBuff.isFull();
}