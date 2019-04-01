#include <stdint.h>
#include <CircularBuffer.h>
#include <ardupilotmega/mavlink.h>

#ifndef VARS_H_
#define VARS_H_

#include "Location.h"
#include "Battery.h"
#include "config.h"

//*** LEDS ********************************************************************************************************
//uint16_t MavStatusLed = 13;
uint8_t MavLedState = LOW;
uint16_t BufStatusLed = 12;
uint8_t BufLedState = LOW;

//#if (Target_Board == 0) // Teensy3x
#if defined(TEENSY31)
#define MavStatusLed 13
//#elif (Target_Board == 1) // Blue Pill
#elif defined(ARDUINO_BLUEPILL_F103C6)
#define PROCESSOR_BLUE_PILL 1
#define MavStatusLed PC13
#if defined Aux_Port_Enabled
#error Blue Pill board does not have enough UARTS for Auxilliary port. Un-comment #define Aux_Port_Enabled.
#endif
#if (SPort_Serial == 3)
#error Blue Pill board does not have Serial3. This configuration is not possible.
#endif
//#elif (Target_Board == 2) //  Maple Mini
#elif defined(ARDUINO_MAPLE_MINI_ORIGIN)
#define MavStatusLed 33 // PB1
#endif

#define Debug Serial // USB
#define frBaud 57600 // Use 57600
#define mavSerial Serial2
#define mavBaud 57600

#if (SPort_Serial == 1)
#define frSerial Serial1 // S.Port
#elif (SPort_Serial == 3)
#define frSerial Serial3 // S.Port
#else
#error SPort_Serial can only be 1 or 3. Please correct.
#endif

#if defined Aux_Port_Enabled
#if (SPort_Serial == 3)
#error Aux port and SPort both configured for Serial3. Please correct.
#else
#define auxSerial Serial3 // Mavlink telemetry to and from auxilliary adapter
#define auxBaud 57600     // Use 57600
#define auxDuplex         // Pass aux <-> FC traffic up and down, else only down from FC
#endif
#endif

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t hb_count = 0;

bool ap_bat_paramsReq = false;
bool ap_bat_paramsRead = false;
bool parm_msg_shown = false;
bool ap_paramsList = false;
uint8_t paramsID = 0;

bool homGood = false;
bool mavGood = false;
bool rssiGood = false;

uint32_t hb_millis = 0;
uint32_t rds_millis = 0;
uint32_t acc_millis = 0;
uint32_t em_millis = 0;
uint32_t sp_millis = 0;
uint32_t mav_led_millis = 0;

uint32_t now_millis = 0;
uint32_t prev_millis = 0;

uint32_t lat800_millis = 0;
uint32_t lon800_millis = 0;
uint32_t ST5000_millis = 0;
uint32_t AP5001_millis = 0;
uint32_t GPS5002_millis = 0;
uint32_t Bat1_5003_millis = 0;
uint32_t Home_5004_millis = 0;
uint32_t VelYaw5005_millis = 0;
uint32_t Atti5006_millis = 0;
uint32_t Param5007_millis = 0;
uint32_t Bat2_5008_millis = 0;
uint32_t RC_5009_millis = 0;
uint32_t Hud_5010_millis = 0;
uint32_t rssi_F101_millis = 0;

float lon1, lat1, lon2, lat2, alt1, alt2;

// 3D Location vectors

volatile /*struct*/ Location hom = {
    0, 0, 0, 0}; // home location

volatile /*struct*/ Location cur = {
    0, 0, 0, 0}; // current location

/*struct*/ Battery bat1 = {
    0, 0, 0, 0, 0, 0, 0, true};

/*struct*/ Battery bat2 = {
    0, 0, 0, 0, 0, 0, 0, true};

//****************** Ring Buffers *************************
typedef struct {
    uint8_t severity;
    char text[50];
    uint8_t txtlth;
    bool simple;
} ST_type;

ST_type ST_record;

CircularBuffer<ST_type, 10> MsgRingBuff;

CircularBuffer<mavlink_message_t, 10> MavRingBuff;

// ******************************************

// uint8_t fr_chunk_pntr = 0; // chunk pointer
// uint16_t fr_bat2_mAh;
// uint8_t fr_gps_status;     // part a

mavlink_message_t msg;
uint8_t len;
// Mavlink Messages

// Mavlink Header
uint8_t system_id;
uint8_t component_id;
uint8_t target_component;
uint8_t mvType;

// Message #0  HEARTHBEAT
uint8_t ap_type_tmp = 0; // hold the type until we know HB not from GCS or Tracket
uint8_t ap_type = 0;
uint8_t ap_autopilot = 0;
uint8_t ap_base_mode = 0;
uint32_t ap_custom_mode = 0;
uint8_t ap_system_status = 0;
uint8_t ap_mavlink_version = 0;
bool px4_flight_stack = false;
uint8_t px4_main_mode = 0;
uint8_t px4_sub_mode = 0;

// Message # 1  SYS_STATUS
uint16_t ap_voltage_battery1 = 0; // 1000 = 1V
int16_t ap_current_battery1 = 0;  //  10 = 1A
uint8_t ap_ccell_count1 = 0;

// Message #20 PARAM_REQUEST_READ
// target_system  System ID
uint8_t target_system;   //   System ID
char req_param_id[16];   //  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
int16_t req_param_index; //  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)

// Message #20 PARAM_REQUEST_READ
//  Generic Mavlink Header defined above
// use #22 PARAM_VALUE variables below
// ap_param_index . Send -1 to use the param ID field as identifier (else the param id will be ignored)
float ap_bat1_capacity;
float ap_bat2_capacity;

// Message #21 PARAM_REQUEST_LIST
//  Generic Mavlink Header defined above

// Message #22 PARAM_VALUE
char ap_param_id[16];
float ap_param_value;
uint8_t ap_param_type;
uint16_t ap_param_count; //  Total number of onboard parameters
uint16_t ap_param_index; //  Index of this onboard parameter

// Message #24  GPS_RAW_INT
uint8_t ap_fixtype = 3;     // 0= No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed, 7=Static, 8=PPP
uint8_t ap_sat_visible = 0; // numbers of visible satelites
int32_t ap_latitude = 0;    // 7 assumed decimal places
int32_t ap_longitude = 0;   // 7 assumed decimal places
int32_t ap_amsl24 = 0;      // 1000 = 1m
uint16_t ap_eph;            // GPS HDOP horizontal dilution of position (unitless)
uint16_t ap_epv;            // GPS VDOP vertical dilution of position (unitless)
uint16_t ap_vel;            // GPS ground speed (m/s * 100) cm/s
uint16_t ap_cog;            // Course over ground in degrees * 100, 0.0..359.99 degrees
// mav2
int32_t ap_alt_ellipsoid; // mm    Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
uint32_t ap_h_acc;        // mm    Position uncertainty. Positive for up.
uint32_t ap_v_acc;        // mm    Altitude uncertainty. Positive for up.
uint32_t ap_vel_acc;      // mm    Speed uncertainty. Positive for up.
uint32_t ap_hdg_acc;      // degE5   Heading / track uncertainty

// Message #27 RAW IMU
int32_t ap_accX = 0;
int32_t ap_accY = 0;
int32_t ap_accZ = 0;

// Message #29 SCALED_PRESSURE
uint32_t ap_time_boot_ms; // Timestamp (milliseconds since system boot)
float ap_press_abs;       // Absolute pressure (hectopascal)
float ap_press_diff;      // Differential pressure 1 (hectopascal)
int16_t ap_temperature;   // Temperature measurement (0.01 degrees celsius)

// Message ATTITUDE ( #30 )
float ap_roll;       // Roll angle (rad, -pi..+pi)
float ap_pitch;      // Pitch angle (rad, -pi..+pi)
float ap_yaw;        // Yaw angle (rad, -pi..+pi)
float ap_rollspeed;  // Roll angular speed (rad/s)
float ap_pitchspeed; // Pitch angular speed (rad/s)
float ap_yawspeed;   // Yaw angular speed (rad/s)

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap_lat;      // Latitude, expressed as degrees * 1E7
int32_t ap_lon;      // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33;   // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag;   // Altitude above ground (millimeters)
int16_t ap_vx;       // Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;       // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;       // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_gps_hdg; // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

// Message #65 RC_Channels
bool ap_rc_flag = false; // true when rc record received
uint8_t ap_chcnt;
uint16_t ap_chan_raw[17]; // 16 channels, [0] ignored use [1] thru [16] for simplicity

//uint16_t ap_chan16_raw;        // Used for RSSI uS 1000=0%  2000=100%
uint8_t rssi; // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #74 VFR_HUD
float ap_hud_air_spd;
float ap_hud_grd_spd;
int16_t ap_hud_hdg;
uint16_t ap_hud_throt;
float ap_hud_bar_alt;
float ap_hud_climb;

// Message  #125 POWER_STATUS
uint16_t ap_Vcc;    // 5V rail voltage in millivolts
uint16_t ap_Vservo; // servo rail voltage in millivolts
uint16_t ap_flags;  // power supply status flags (see MAV_POWER_STATUS enum)
/*
 * MAV_POWER_STATUS
Power supply status flags (bitmask)
1   MAV_POWER_STATUS_BRICK_VALID  main brick power supply valid
2   MAV_POWER_STATUS_SERVO_VALID  main servo power supply valid for FMU
4   MAV_POWER_STATUS_USB_CONNECTED  USB power is connected
8   MAV_POWER_STATUS_PERIPH_OVERCURRENT peripheral supply is in over-current state
16  MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT hi-power peripheral supply is in over-current state
32  MAV_POWER_STATUS_CHANGED  Power status has changed since boot
 */

// Message  #147 BATTERY_STATUS
uint8_t ap_battery_id;
uint8_t ap_battery_function;
uint8_t ap_bat_type;
int16_t ap_bat_temperature;  // centi-degrees celsius
uint16_t ap_voltages[10];    // cell voltages in millivolts
int16_t ap_current_battery;  // in 10*milliamperes (1 = 10 milliampere)
int32_t ap_current_consumed; // mAh
int32_t ap_energy_consumed;  // HectoJoules (intergrated U*I*dt) (1 = 100 Joule)
int8_t ap_battery_remaining; // (0%: 0, 100%: 100)
int32_t ap_time_remaining;   // in seconds
uint8_t ap_charge_state;

// Message #166 RADIO
uint8_t ap_rssi;      // local signal strength
uint8_t ap_remrssi;   // remote signal strength
uint8_t ap_txbuf;     // how full the tx buffer is as a percentage
uint8_t ap_noise;     // background noise level
uint8_t ap_remnoise;  // remote background noise level
uint16_t ap_rxerrors; // receive errors
uint16_t ap_fixed;    // count of error corrected packets

// Message #181 BATTERY2
uint16_t ap_voltage_battery2 = 0; // 1000 = 1V
int16_t ap_current_battery2 = 0;  //  10 = 1A
uint8_t ap_cell_count2 = 0;

// Message #253 STATUSTEXT
uint8_t ap_severity;
char ap_text[60];
uint8_t ap_txtlth;
bool ap_simple = 0;

// //***************************************************************
// // FrSky Passthrough Variables
uint32_t fr_payload;

// 0x800 GPS
uint8_t ms2bits;
uint32_t fr_lat = 0;
uint32_t fr_lon = 0;

// 0x5000 Text Msg
uint32_t fr_textmsg;
char fr_text[60];
uint8_t fr_severity;
uint8_t fr_txtlth;
char fr_chunk[4];
uint8_t fr_chunk_pntr = 0; // chunk pointer

// 0x5001 AP Status
uint8_t fr_flight_mode;
uint8_t fr_simple;

uint8_t fr_land_complete;
uint8_t fr_armed;
uint8_t fr_bat_fs;
uint8_t fr_ekf_fs;

// 0x5002 GPS Status
uint8_t fr_numsats;
uint8_t fr_gps_status;     // part a
uint8_t fr_gps_adv_status; // part b
uint8_t fr_hdop;
uint32_t fr_amsl;

uint8_t neg;

//0x5003 Batt
uint16_t fr_bat1_volts;
uint16_t fr_bat1_amps;
uint16_t fr_bat1_mAh;

// 0x5004 Home
uint16_t fr_home_dist;
int16_t fr_home_angle; // degrees
int16_t fr_home_arrow; // 0 = heading pointing to home, unit = 3 degrees
int16_t fr_home_alt;

short fr_pwr;

// 0x5005 Velocity and yaw
uint32_t fr_velyaw;
float fr_vy;  // climb in decimeters/s
float fr_vx;  // groundspeed in decimeters/s
float fr_yaw; // heading units of 0.2 degrees

// 0x5006 Attitude and range
uint16_t fr_roll;
uint16_t fr_pitch;
uint16_t fr_range;

// 0x5007 Parameters
uint8_t fr_param_id;
uint32_t fr_param_val;
uint32_t fr_frame_type;
uint32_t fr_bat1_capacity;
uint32_t fr_bat2_capacity;
bool fr_paramsSent = false;

//0x5008 Batt
float fr_bat2_volts;
float fr_bat2_amps;
uint16_t fr_bat2_mAh;

//0x5009 RC channels       // 4 ch per frame
uint8_t fr_chcnt;
int8_t fr_rc[5]; // [0] ignored use [1] thu [4] for simplicity

//0x5010 HUD
float fr_air_spd;  // dm/s
uint16_t fr_throt; // 0 to 100%
float fr_bar_alt;  // metres

//0xF103
uint32_t fr_rssi;

#endif