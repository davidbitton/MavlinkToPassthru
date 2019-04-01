#include <stdint.h>
#include <Arduino.h>

#ifndef FRSKYSPORTPASSTHRU_H_
#define FRSKYSPORTPASSTHRU_H_

#include "vars.h"

void FrSkySPort_Init(void);
void ReadSPort(void);
uint32_t bit32Extract(uint32_t dword, uint8_t displ, uint8_t lth);
void FrSkySPort_Process();
void SendRssiF101();
void SendLat800();
void SendLon800();
void SendStatusTextChunk5000();
void SendAP_Status5001();
void Send_GPS_Status5002();
void Send_Bat1_5003();
void Send_Home_5004();
void Send_VelYaw_5005();
void Send_Atti_5006();
void SendParameters5007();
void Send_Bat2_5008();
void CheckByteStuffAndSend(uint8_t byte);
uint32_t createMask(uint8_t lo, uint8_t hi);
uint32_t Abs(int32_t num);
uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);
int16_t Add360(int16_t arg1, int16_t arg2);

//***************************************************
uint8_t PX4FlightModeNum(uint8_t main, uint8_t sub) {
    switch (main) {

    case 1:
        return 1; // MANUAL
    case 2:
        return 2; // ALTITUDE
    case 3:
        return 3; // POSCTL
    case 4:

        switch (sub) {
        case 1:
            return 4; // AUTO READY
        case 2:
            return 5; // AUTO TAKEOFF
        case 3:
            return 6; // AUTO LOITER
        case 4:
            return 7; // AUTO MISSION
        case 5:
            return 8; // AUTO RTL
        case 6:
            return 9; // AUTO LAND
        case 7:
            return 10; //  AUTO RTGS
        case 8:
            return 11; // AUTO FOLLOW ME
        case 9:
            return 12; //  AUTO PRECLAND
        default:
            return 13; //  AUTO UNKNOWN
        }

    case 5:
        return 14; //  ACRO
    case 6:
        return 15; //  OFFBOARD
    case 7:
        return 16; //  STABILIZED
    case 8:
        return 17; //  RATTITUDE
    case 9:
        return 18; //  SIMPLE
    default:
        return 19; //  UNKNOWN
    }
}


#endif