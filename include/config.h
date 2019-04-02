#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

//************************************* Please select your options here before compiling **************************
//#define PX4_Flight_stack   //  If your flight stack is PX4 and not APM, un-comment this line
// Choose one (only) of these target boards
//#define Target_Board 0 // Teensy 3.x              Un-comment this line if you are using a Teensy 3.x
//#define Target_Board   1      // Blue Pill STM32F103C    OR un-comment this line if you are using a Blue Pill STM32F103C
//#define Target_Board   2      // Maple_Mini STM32F103C   OR un-comment this line if you are using a Maple_Mini STM32F103C

// Choose one (only) of these three modes
//#define Ground_Mode          // Converter between Taranis and LRS tranceiver (like Orange)
//#define Air_Mode             // Converter between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
#define Relay_Mode // Converter between LRS tranceiver (like Orange) and FrSky receiver (like XRS) in relay box on the ground

//#define Battery_mAh_Source  1  // Get battery mAh from the FC - note both RX and TX lines must be connected
//#define Battery_mAh_Source  2  // Define bat1_capacity and bat2_capacity below and use those
#define Battery_mAh_Source 3 // Define battery mAh in the LUA script on the Taranis/Horus - Recommended

const uint16_t bat1_capacity = 5200;
const uint16_t bat2_capacity = 0;

#define SPort_Serial 1 // The default is Serial 1, but 3 is possible if we don't want aux port

//#define Aux_Port_Enabled    // For BlueTooth or other auxilliary serial passthrough
//#define QLRS                // Un-comment this line only if you are using the QLRS telemetry system


#define Debug Serial
#define mavBaud 57600
#define mavSerial Serial2

#endif