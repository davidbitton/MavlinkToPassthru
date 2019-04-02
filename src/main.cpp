#include <Arduino.h>

#include "FrSkySport.h"
#include "MavlinkManager.h"
#include "config.h"

#if defined(TEENSY31)
#define MavStatusLed 13
#elif defined(ARDUINO_BLUEPILL_F103C6)
#define PROCESSOR_BLUE_PILL 1
#define MavStatusLed PC13
#if defined Aux_Port_Enabled
#error Blue Pill board does not have enough UARTS for Auxilliary port. Un-comment #define Aux_Port_Enabled.
#endif
#if (SPort_Serial == 3)
#error Blue Pill board does not have Serial3. This configuration is not possible.
#endif
#elif defined(ARDUINO_MAPLE_MINI_ORIGIN)
#define MavStatusLed 33 // PB1
#endif

uint8_t MavLedState = LOW;
uint16_t BufStatusLed = 12;
uint8_t BufLedState = LOW;

bool homGood = false;
bool mavGood = false;

uint16_t hb_count = 0;

uint32_t hb_millis = 0;
uint32_t acc_millis = 0;
uint32_t rds_millis = 0;
uint32_t em_millis = 0;
uint32_t mav_led_millis = 0;
uint32_t sp_millis = 0;

FrSkySport frSkySport;
MavlinkManager mavlinkManager;

// //****************** Ring Buffers *************************
// typedef struct {
//   uint8_t severity;
//   char text[50];
//   uint8_t txtlth;
//   bool simple;
// } ST_type;

// ST_type ST_record;
// CircularBuffer<ST_type, 10> MsgRingBuff;
// CircularBuffer<mavlink_message_t, 10> MavRingBuff;
// // ******************************************

void BlinkMavLed(uint32_t period) {
    uint32_t cMillis = millis();
    if (cMillis - mav_led_millis >= period) { // blink period
        mav_led_millis = cMillis;
        if (MavLedState == LOW) {
            MavLedState = HIGH;
        } else {
            MavLedState = LOW;
        }
    }
}

void ServiceMavStatusLed() {
    if (mavGood) {
        MavLedState = HIGH;
        digitalWrite(MavStatusLed, MavLedState);
    } else {
        BlinkMavLed(500);
    }
    digitalWrite(MavStatusLed, MavLedState);
}

void ServiceBufStatusLed() {
    if (/*MsgRingBuff*/ frSkySport.BufferIsFull()) {
        BufLedState = HIGH;
    } else
        BufLedState = LOW;
    digitalWrite(BufStatusLed, BufLedState);
}

void ServiceStatusLeds() {
    ServiceMavStatusLed();
    ServiceBufStatusLed();
}

// ******************************************
void setup() {

    frSkySport.Init();
    mavSerial.begin(mavBaud);

#if defined Aux_Port_Enabled
    auxSerial.begin(auxBaud);
#endif

    Debug.begin(115200);

    mavGood = false;
    homGood = false;
    hb_count = 0;
    hb_millis = millis();
    acc_millis = millis();
    rds_millis = millis();
    em_millis = millis();

    delay(2500);
    Debug.println("Starting .... ");

    Debug.print("Target Board is ");
#if (Target_Board == 0) // Teensy3x
    Debug.println("Teensy 3.x");
#elif (Target_Board == 1) // Blue Pill
    Debug.println("Blue Pill STM32F103C");
#elif (Target_Board == 2) //  Maple Mini
    Debug.println("Maple Mini STM32F103C");
#endif

#ifdef Ground_Mode
    Debug.println("Ground Mode");
#endif
#ifdef Air_Mode
    Debug.println("Air Mode");
#endif
#ifdef Relay_Mode
    Debug.println("Relay Mode");
#endif

#if defined Aux_Port_Enabled
    Debug.print("Auxilliary Mavlink port enabled");
#ifdef auxDuplex
    Debug.println(" for two-way telemetry to/from Flight Control computer");
#else
    Debug.println(" for one-way telemetry down from Flight Control computer");
#endif
#endif

#if (Battery_mAh_Source == 1)
    Debug.println("Battery_mAh_Source = 1 - Get battery capacities from the FC");
#elif (Battery_mAh_Source == 2)
    Debug.println(
        "Battery_mAh_Source = 2 - Define battery capacities in this firmware");
#elif (Battery_mAh_Source == 3)
    Debug.println(
        "Battery_mAh_Source = 3 - Define battery capacities in the LUA script");
#else
#error You must define at least one Battery_mAh_Source. Please correct.
#endif

#if (SPort_Serial == 1)
    Debug.println("Using Serial_1 for S.Port");
#else
    Debug.println("Using Serial_3 for S.Port");
#endif

#if defined QLRS
    Debug.println("QLRS variant");
#endif

    pinMode(MavStatusLed, OUTPUT);
    pinMode(BufStatusLed, OUTPUT);
}

// ******************************************
// ******************************************
void loop() {

#ifdef Data_Streams_Enabled
    if (mavGood) { // If we have a link request data streams from MavLink every 5s
        if (millis() - rds_millis > 5000) {
            rds_millis = millis();
            Debug.println("Requesting data streams");
            RequestDataStreams(); // ensure Teensy Tx connected to Taranis RX  (When
                                  // SRx not enumerated)
        }
    }
#endif

    if (mavGood &&
        (millis() - hb_millis) > 3000) { // if no heartbeat from APM in 3s then
                                         // assume mav not connected
        mavGood = false;
        Debug.println("Heartbeat timed out! Mavlink not connected");
        hb_count = 0;
    }
#if (Battery_mAh_Source == 1) // Get battery capacity from the FC
    // Request battery capacity params, and when they have safely arrived switch
    // txsw_pin (6) high
    if (mavGood) {
        if (!ap_bat_paramsReq) {
            Request_Param_Read(
                356); // Request Bat1 capacity   do this twice in case of lost frame
            Request_Param_Read(356);
            Request_Param_Read(364); // Request Bat2 capacity
            Request_Param_Read(364);
            Debug.println("Battery capacities requested");
            ap_bat_paramsReq = true;
        } else {
            if (ap_bat_paramsRead && (!parm_msg_shown)) {
                parm_msg_shown = true;
                Debug.println("Battery params successfully read");
            }
        }
    }
#endif

#ifdef Mav_List_Params
    if (mavGood && (!ap_paramsList)) {
        Request_Param_List();
        ap_paramsList = true;
    }
#endif

    if (mavSerial.available())
        mavlinkManager
            .QueueOneMavFrame(); // Add one Mavlink frame to the ring buffer

    mavlinkManager.DecodeOneMavFrame(); // Decode a Mavlink frame from the ring buffer if there
                                        // is one

    Aux_ReceiveAndForward(); // Service aux incoming if enabled

#ifdef Ground_Mode
    if (mavGood && ((millis() - em_millis) > 10)) {
        Emulate_ReadSPort(); // Emulate the sensor IDs received from XRS receiver on
                             // SPort
        em_millis = millis();
    }
#endif

#if defined Air_Mode || defined Relay_Mode
    if (mavGood && ((millis() - sp_millis) >
                    1)) { // fine tune the polling period, zero for Blue Pill
        ReadSPort();      // Receive round-robin of sensor IDs received from XRS receiver
        sp_millis = millis();
    }
#endif

    ServiceStatusLeds();
}