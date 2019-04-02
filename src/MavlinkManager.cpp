#include "MavlinkManager.h"
#include "utility.h"

void MavlinkManager::QueueOneMavFrame() {
    mavlink_message_t ring_msg;
    mavlink_status_t status;
    while (mavSerial.available()) {
        uint8_t c = mavSerial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &ring_msg, &status)) {
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

void MavlinkManager::DecodeOneMavFrame() {

    if (!MavRingBuff.isEmpty()) {

        msg = (MavRingBuff.shift()); // Get a mavlink message from front of queue

#if defined Mav_Debug_RingBuff
        Debug.print("Mavlink ring buffer msg: ");
        PrintMavBuffer(&msg);
        Debug.print(" Mav queue length after shift= ");
        Debug.println(MavRingBuff.size());
#endif

#if defined Aux_Port_Enabled
        len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef Aux_Port_Debug
        Debug.println("auxSerial passed down from FC:");
        PrintMavBuffer(&msg);
#endif
        auxSerial.write(buf, len);
#endif

        // Debug.print(" msgid="); Debug.println(msg.msgid);

        switch (msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT:                                  // #0   http://mavlink.org/messages/common
            uint8_t ap_type_tmp = mavlink_msg_heartbeat_get_type(&msg); // Alex - don't contaminate the ap-type variable
            if (ap_type_tmp == 5 || ap_type_tmp == 6 || ap_type_tmp == 27)
                break;
            // Ignore heartbeats from GCS (6) or Ant Trackers(5) or ADSB (27)
            uint8_t ap_type = ap_type_tmp;
            uint8_t ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
            uint8_t ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
            uint32_t ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);

            uint8_t px4_main_mode = bit32Extract(ap_custom_mode, 16, 8);
            uint8_t px4_sub_mode = bit32Extract(ap_custom_mode, 24, 8);
            //px4_flight_stack = (ap_autopilot == MAV_AUTOPILOT_PX4);

            // ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
            // ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
            //hb_millis = millis();

            if ((ap_base_mode >> 7) && (!homGood))
                MarkHome(); // If motors armed for the first time, then mark this spot as home

#if defined Mav_Debug_All || defined Mav_Debug_Heartbeat
            Debug.print("Mavlink in #0 Heartbeat: ");
            Debug.print("ap_type=");
            Debug.print(ap_type);
            Debug.print("  ap_autopilot=");
            Debug.print(ap_autopilot);
            Debug.print("  ap_base_mode=");
            Debug.print(ap_base_mode);
            Debug.print(" ap_custom_mode=");
            Debug.print(ap_custom_mode);
            Debug.print("  ap_system_status=");
            Debug.print(/* ap_system_status */ mavlink_msg_heartbeat_get_system_status(&msg));
            Debug.print("  ap_mavlink_version=");
            Debug.print(/* ap_mavlink_version */ mavlink_msg_heartbeat_get_mavlink_version(&msg));

            if (/* px4_flight_stack */ (ap_autopilot == MAV_AUTOPILOT_PX4)) {
                Debug.print(" px4_main_mode=");
                Debug.print(px4_main_mode);
                Debug.print(" px4_sub_mode=");
                Debug.print(px4_sub_mode);
                Debug.print(" ");
                Debug.print(PX4FlightModeName(px4_main_mode, px4_sub_mode));
            }

            Debug.println();

#endif

            if (!mavGood) {
                hb_count++;
                Debug.print("hb_count=");
                Debug.print(hb_count);
                Debug.println("");

                if (hb_count >= 3) { // If  3 heartbeats from MavLink then we are connected
                    mavGood = true;
                    Debug.println("mavgood=true");
                    hb_count = 0;
                }
            }
            break;
        case MAVLINK_MSG_ID_SYS_STATUS: // #1
            if (!mavGood)
                break;
            ap_voltage_battery1 = Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&msg));    // 1000 = 1V  i.e mV
            ap_current_battery1 = Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&msg)); //  100 = 1A, i.e dA
            if (ap_voltage_battery1 > 21000)
                ap_ccell_count1 = 6;
            else if (ap_voltage_battery1 > 16800 && ap_ccell_count1 != 6)
                ap_ccell_count1 = 5;
            else if (ap_voltage_battery1 > 12600 && ap_ccell_count1 != 5)
                ap_ccell_count1 = 4;
            else if (ap_voltage_battery1 > 8400 && ap_ccell_count1 != 4)
                ap_ccell_count1 = 3;
            else if (ap_voltage_battery1 > 4200 && ap_ccell_count1 != 3)
                ap_ccell_count1 = 2;
            else
                ap_ccell_count1 = 0;
#if defined Mav_Debug_All || defined Mav_Debug_SysStatus
            Debug.print("Mavlink in #1 Sys_Status: ");
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery1 / 1000, 3); // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery1 / 100, 1); // now A

            Debug.print("  mAh=");
            Debug.print(bat1.mAh, 6);
            Debug.print("  Total mAh=");
            Debug.print(bat1.tot_mAh, 3); // Consumed so far, calculated in Average module

            Debug.print("  Bat1 cell count= ");
            Debug.println(ap_ccell_count1);
#endif
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: // #20 - OUTGOING TO UAV
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: // #21 - OUTGOING TO UAV
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_PARAM_VALUE: // #22
            if (!mavGood)
                break;
            len = mavlink_msg_param_value_get_param_id(&msg, ap_param_id);
            ap_param_value = mavlink_msg_param_value_get_param_value(&msg);
            ap_param_count = mavlink_msg_param_value_get_param_count(&msg);
            ap_param_index = mavlink_msg_param_value_get_param_index(&msg);

            switch (ap_param_index) { // if#define Battery_mAh_Source !=1 these will never arrive
            case 356:                 // Bat1 Capacity
                ap_bat1_capacity = ap_param_value;
#if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink in #22 Param_Value: ");
                Debug.print("bat1 capacity=");
                Debug.println(ap_bat1_capacity);
#endif
                break;
            case 364: // Bat2 Capacity
                ap_bat2_capacity = ap_param_value;
                ap_bat_paramsRead = true;
#if defined Mav_Debug_All || defined Debug_Batteries
                Debug.print("Mavlink in #22 Param_Value: ");
                Debug.print("bat2 capacity=");
                Debug.println(ap_bat2_capacity);
#endif
                break;
            }

#if defined Mav_Debug_All || defined Mav_Debug_Params
            Debug.print("Mavlink in #22 Param_Value: ");
            Debug.print("param_id=");
            Debug.print(ap_param_id);
            Debug.print("  param_value=");
            Debug.print(ap_param_value, 4);
            Debug.print("  param_count=");
            Debug.print(ap_param_count);
            Debug.print("  param_index=");
            Debug.println(ap_param_index);
#endif
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT: // #24
            if (!mavGood)
                break;
            ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
            ap_sat_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg); // number of visible satellites
            if (ap_fixtype > 2) {
                ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
                ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg); // 1m =1000
                ap_eph = mavlink_msg_gps_raw_int_get_eph(&msg);    // GPS HDOP
                ap_epv = mavlink_msg_gps_raw_int_get_epv(&msg);    // GPS VDOP
                ap_vel = mavlink_msg_gps_raw_int_get_vel(&msg);    // GPS ground speed (m/s * 100)
                ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg);    // Course over ground (NOT heading) in degrees * 100
            }
#if defined Mav_Debug_All || defined Mav_Debug_GPS_Raw
            Debug.print("Mavlink in #24 GPS_RAW_INT: ");
            Debug.print("ap_fixtype=");
            Debug.print(ap_fixtype);
            if (ap_fixtype == 0)
                Debug.print(" No GPS");
            else if (ap_fixtype == 1)
                Debug.print(" No Fix");
            else if (ap_fixtype == 2)
                Debug.print(" 2D Fix");
            else if (ap_fixtype == 3)
                Debug.print(" 3D Fix");
            else if (ap_fixtype == 4)
                Debug.print(" DGPS/SBAS aided");
            else if (ap_fixtype == 5)
                Debug.print(" RTK Float");
            else if (ap_fixtype == 6)
                Debug.print(" RTK Fixed");
            else if (ap_fixtype == 7)
                Debug.print(" Static fixed");
            else if (ap_fixtype == 8)
                Debug.print(" PPP");
            else
                Debug.print(" Unknown");

            Debug.print("  sats visible=");
            Debug.print(ap_sat_visible);
            Debug.print("  latitude=");
            Debug.print((float)(ap_latitude) / 1E7, 7);
            Debug.print("  longitude=");
            Debug.print((float)(ap_longitude) / 1E7, 7);
            Debug.print("  gps alt amsl=");
            Debug.print((float)(ap_amsl24) / 1E3, 1);
            Debug.print("  eph (hdop)=");
            Debug.print(ap_eph); // HDOP
            Debug.print("  epv (vdop)=");
            Debug.print(ap_epv);
            Debug.print("  vel=");
            Debug.print((float)ap_vel / 100, 3); // GPS ground speed (m/s)
            Debug.print("  cog=");
            Debug.println((float)ap_cog / 100, 1); // Course over ground in degrees
#endif
            break;
        case MAVLINK_MSG_ID_RAW_IMU: // #27
            if (!mavGood)
                break;
            ap_accX = mavlink_msg_raw_imu_get_xacc(&msg);
            ap_accY = mavlink_msg_raw_imu_get_yacc(&msg);
            ap_accZ = mavlink_msg_raw_imu_get_zacc(&msg);
#if defined Mav_Debug_All || defined Mav_Debug_Raw_IMU
            Debug.print("Mavlink in #27 Raw_IMU: ");
            Debug.print("accX=");
            Debug.print((float)ap_accX / 1000);
            Debug.print("  accY=");
            Debug.print((float)ap_accY / 1000);
            Debug.print("  accZ=");
            Debug.println((float)ap_accZ / 1000);
#endif
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE: // #29
            if (!mavGood)
                break;
            ap_press_abs = mavlink_msg_scaled_pressure_get_press_abs(&msg);
            ap_temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
#if defined Mav_Debug_All || defined Mav_Debug_Scaled_Pressure
            Debug.print("Mavlink in #29 Scaled_Pressure: ");
            Debug.print("  press_abs=");
            Debug.print(ap_press_abs, 1);
            Debug.print("hPa  press_diff=");
            Debug.print(ap_press_diff, 3);
            Debug.print("hPa  temperature=");
            Debug.print((float)(ap_temperature) / 100, 1);
            Debug.println("C");
#endif
            break;
        case MAVLINK_MSG_ID_ATTITUDE: // #30
            if (!mavGood)
                break;

            ap_roll = mavlink_msg_attitude_get_roll(&msg);             // Roll angle (rad, -pi..+pi)
            ap_pitch = mavlink_msg_attitude_get_pitch(&msg);           // Pitch angle (rad, -pi..+pi)
            ap_yaw = mavlink_msg_attitude_get_yaw(&msg);               // Yaw angle (rad, -pi..+pi)
            ap_rollspeed = mavlink_msg_attitude_get_rollspeed(&msg);   // Roll angular speed (rad/s)
            ap_pitchspeed = mavlink_msg_attitude_get_pitchspeed(&msg); // Pitch angular speed (rad/s)
            ap_yawspeed = mavlink_msg_attitude_get_yawspeed(&msg);     // Yaw angular speed (rad/s)

            ap_roll = RadToDeg(ap_roll); // Now degrees
            ap_pitch = RadToDeg(ap_pitch);
            ap_yaw = RadToDeg(ap_yaw);

#if defined Mav_Debug_All || defined Mav_Debug_Attitude
            Debug.print("Mavlink in #30 Attitude: ");
            Debug.print(" ap_roll degs=");
            Debug.print(ap_roll, 1);
            Debug.print(" ap_pitch degs=");
            Debug.print(ap_pitch, 1);
            Debug.print(" ap_yaw degs=");
            Debug.println(ap_yaw, 1);
#endif

            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // #33
            if ((!mavGood) || (ap_fixtype < 3))
                break;
            ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
            ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
            ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // Altitude above mean sea level (millimeters)
            ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
            ap_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
            ap_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
            ap_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
            ap_gps_hdg = mavlink_msg_global_position_int_get_hdg(&msg);         // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

            cur.lat = (float)ap_lat / 1E7;
            cur.lon = (float)ap_lon / 1E7;
            cur.alt = ap_amsl33 / 1E3;
            cur.hdg = ap_gps_hdg / 100;

#if defined Mav_Debug_All || defined Mav_Debug_GPS_Int
            Debug.print("Mavlink in #33 GPS Int: ");
            Debug.print(" ap_lat=");
            Debug.print((float)ap_lat / 1E7, 6);
            Debug.print(" ap_lon=");
            Debug.print((float)ap_lon / 1E7, 6);
            Debug.print(" ap_amsl=");
            Debug.print((float)ap_amsl33 / 1E3, 0);
            Debug.print(" ap_alt_ag=");
            Debug.print((float)ap_alt_ag / 1E3, 1);
            Debug.print(" ap_vx=");
            Debug.print((float)ap_vx / 100, 2);
            Debug.print(" ap_vy=");
            Debug.print((float)ap_vy / 100, 2);
            Debug.print(" ap_vz=");
            Debug.print((float)ap_vz / 100, 2);
            Debug.print(" ap_gps_hdg=");
            Debug.println((float)ap_gps_hdg / 100, 1);
#endif

            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: // #35
            if (!mavGood)
                break;
#if defined QLRS
            rssiGood = true; //  We have received at least one rssi packet from air mavlink
            ap_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
            ap_rc_flag = true;
#endif
#if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
            Debug.print("Mavlink in #35 RC_Channels_Raw: ");
            Debug.print("  Receive RSSI=");
            Debug.println(ap_rssi / 2.54);
#endif
            break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: // #36
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_MISSION_CURRENT: // #42
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS: // #65
            if (!mavGood)
                break;
            ap_chcnt = mavlink_msg_rc_channels_get_chancount(&msg);
            ap_chan_raw[1] = mavlink_msg_rc_channels_get_chan1_raw(&msg);
            ap_chan_raw[2] = mavlink_msg_rc_channels_get_chan2_raw(&msg);
            ap_chan_raw[3] = mavlink_msg_rc_channels_get_chan3_raw(&msg);
            ap_chan_raw[4] = mavlink_msg_rc_channels_get_chan4_raw(&msg);
            ap_chan_raw[5] = mavlink_msg_rc_channels_get_chan5_raw(&msg);
            ap_chan_raw[6] = mavlink_msg_rc_channels_get_chan6_raw(&msg);
            ap_chan_raw[7] = mavlink_msg_rc_channels_get_chan7_raw(&msg);
            ap_chan_raw[8] = mavlink_msg_rc_channels_get_chan8_raw(&msg);
            ap_chan_raw[9] = mavlink_msg_rc_channels_get_chan9_raw(&msg);
            ap_chan_raw[10] = mavlink_msg_rc_channels_get_chan10_raw(&msg);
            ap_chan_raw[11] = mavlink_msg_rc_channels_get_chan11_raw(&msg);
            ap_chan_raw[12] = mavlink_msg_rc_channels_get_chan12_raw(&msg);
            ap_chan_raw[13] = mavlink_msg_rc_channels_get_chan13_raw(&msg);
            ap_chan_raw[14] = mavlink_msg_rc_channels_get_chan14_raw(&msg);
            ap_chan_raw[15] = mavlink_msg_rc_channels_get_chan15_raw(&msg);
            ap_chan_raw[16] = mavlink_msg_rc_channels_get_chan16_raw(&msg);
#ifndef QLRS
            ap_rssi = mavlink_msg_rc_channels_get_rssi(&msg); // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown
            ap_rc_flag = true;                                // tell fr routine we have an rc records
            rssiGood = true;                                  //  We have received at least one rssi packet from air mavlink
#endif
#if defined Mav_Debug_All || defined Mav_Debug_Rssi || defined Mav_Debug_RC
            Debug.print("Mavlink in #65 RC_Channels: ");
            Debug.print("Channel count= ");
            Debug.print(ap_chcnt);
            Debug.print(" values: ");
            for (int i = 1; i <= ap_chcnt; i++) {
                Debug.print(" ");
                Debug.print(i);
                Debug.print("=");
                Debug.print(ap_chan_raw[i]);
            }
            Debug.print("  Receive RSSI=");
            Debug.println(ap_rssi / 2.54);
#endif
            break;
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: // #66 - OUTGOING TO UAV
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_VFR_HUD: //  #74
            if (!mavGood)
                break;
            ap_hud_air_spd = mavlink_msg_vfr_hud_get_airspeed(&msg);
            ap_hud_grd_spd = mavlink_msg_vfr_hud_get_groundspeed(&msg); //  in m/s
            ap_hud_hdg = mavlink_msg_vfr_hud_get_heading(&msg);         //  in degrees
            ap_hud_throt = mavlink_msg_vfr_hud_get_throttle(&msg);      //  integer percent
            ap_hud_bar_alt = mavlink_msg_vfr_hud_get_alt(&msg);         //  m
            ap_hud_climb = mavlink_msg_vfr_hud_get_climb(&msg);         //  m/s

#if defined Mav_Debug_All || defined Mav_Debug_Hud
            Debug.print("Mavlink in #74 VFR_HUD: ");
            Debug.print("Airspeed= ");
            Debug.print(ap_hud_air_spd, 2); // m/s
            Debug.print("  Groundspeed= ");
            Debug.print(ap_hud_grd_spd, 2); // m/s
            Debug.print("  Heading= ");
            Debug.print(ap_hud_hdg); // deg
            Debug.print("  Throttle %= ");
            Debug.print(ap_hud_throt); // %
            Debug.print("  Baro alt= ");
            Debug.print(ap_hud_bar_alt, 0); // m
            Debug.print("  Climb rate= ");
            Debug.println(ap_hud_climb); // m/s
#endif
            break;
        case MAVLINK_MSG_ID_SCALED_IMU2: // #116   http://mavlink.org/messages/common
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_POWER_STATUS: // #125   http://mavlink.org/messages/common
            if (!mavGood)
                break;
            ap_Vcc = mavlink_msg_power_status_get_Vcc(&msg);       // 5V rail voltage in millivolts
            ap_Vservo = mavlink_msg_power_status_get_Vservo(&msg); // servo rail voltage in millivolts
            ap_flags = mavlink_msg_power_status_get_flags(&msg);   // power supply status flags (see MAV_POWER_STATUS enum)
#ifdef Mav_Debug_All
            Debug.print("Mavlink in #125 Power Status: ");
            Debug.print("Vcc= ");
            Debug.print(ap_Vcc);
            Debug.print("  Vservo= ");
            Debug.print(ap_Vservo);
            Debug.print("  flags= ");
            Debug.println(ap_flags);
#endif
            break;
        case MAVLINK_MSG_ID_BATTERY_STATUS: // #147   http://mavlink.org/messages/common
            if (!mavGood)
                break;
            ap_battery_id = mavlink_msg_battery_status_get_id(&msg);
            ap_current_battery = mavlink_msg_battery_status_get_current_battery(&msg);     // in 10*milliamperes (1 = 10 milliampere)
            ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&msg);   // mAh
            ap_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&msg); // (0%: 0, 100%: 100)

            if (ap_battery_id == 0) { // Battery 1
                fr_bat1_mAh = ap_current_consumed;
            } else if (ap_battery_id == 1) { // Battery 2
                fr_bat2_mAh = ap_current_consumed;
            }

#if defined Mav_Debug_All || defined Debug_Batteries
            Debug.print("Mavlink in #147 Battery Status: ");
            Debug.print(" bat id= ");
            Debug.print(ap_battery_id);
            Debug.print(" bat current mA= ");
            Debug.print(ap_current_battery * 10);
            Debug.print(" ap_current_consumed mAh= ");
            Debug.print(ap_current_consumed);
            if (ap_battery_id == 0) {
                Debug.print(" my di/dt mAh= ");
                Debug.println(Total_mAh1(), 0);
            } else {
                Debug.print(" my di/dt mAh= ");
                Debug.println(Total_mAh2(), 0);
            }
            //  Debug.print(" bat % remaining= ");  Debug.println(ap_time_remaining);
#endif

            break;
        case MAVLINK_MSG_ID_SENSOR_OFFSETS: // #150   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_MEMINFO: // #152   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_RADIO: // #166   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            ap_rssi = mavlink_msg_radio_get_rssi(&msg);         // local signal strength
            ap_remrssi = mavlink_msg_radio_get_remrssi(&msg);   // remote signal strength
            ap_txbuf = mavlink_msg_radio_get_txbuf(&msg);       // how full the tx buffer is as a percentage
            ap_noise = mavlink_msg_radio_get_noise(&msg);       // remote background noise level
            ap_remnoise = mavlink_msg_radio_get_remnoise(&msg); // receive errors
            ap_rxerrors = mavlink_msg_radio_get_rxerrors(&msg); // count of error corrected packets
            ap_fixed = mavlink_msg_radio_get_fixed(&msg);
#ifdef Mav_Debug_All
            Debug.print("Mavlink in #166 Radio: ");
            Debug.print("rssi=");
            Debug.print(ap_rssi);
            Debug.print("remrssi=");
            Debug.print(ap_remrssi);
            Debug.print("txbuf=");
            Debug.print(ap_txbuf);
            Debug.print("noise=");
            Debug.print(ap_noise);
            Debug.print("remnoise=");
            Debug.print(ap_remnoise);
            Debug.print("rxerrors=");
            Debug.print(ap_rxerrors);
            Debug.print("fixed=");
            Debug.println(ap_fixed);
#endif
            break;
        case MAVLINK_MSG_ID_AHRS2: // #178   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_BATTERY2: // #181   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            ap_voltage_battery2 = Get_Volt_Average2(mavlink_msg_battery2_get_voltage(&msg));            // 1000 = 1V
            ap_current_battery2 = Get_Current_Average2(mavlink_msg_battery2_get_current_battery(&msg)); //  100 = 1A
            if (ap_voltage_battery2 > 21000)
                ap_cell_count2 = 6;
            else if (ap_voltage_battery2 > 16800 && ap_cell_count2 != 6)
                ap_cell_count2 = 5;
            else if (ap_voltage_battery2 > 12600 && ap_cell_count2 != 5)
                ap_cell_count2 = 4;
            else if (ap_voltage_battery2 > 8400 && ap_cell_count2 != 4)
                ap_cell_count2 = 3;
            else if (ap_voltage_battery2 > 4200 && ap_cell_count2 != 3)
                ap_cell_count2 = 2;
            else
                ap_cell_count2 = 0;
#if defined Mav_Debug_All || defined Mav_Debug_Batteriestery2
            Debug.print("Mavlink in #181 Battery2: ");
            Debug.print(" Bat volts=");
            Debug.print((float)ap_voltage_battery2 / 1000, 3); // now V
            Debug.print("  Bat amps=");
            Debug.print((float)ap_current_battery2 / 100, 1); // now A

            Debug.print("  mAh=");
            Debug.print(bat2.mAh, 6);
            Debug.print("  Total mAh=");
            Debug.print(bat2.tot_mAh, 3);

            Debug.print("  Bat cell count= ");
            Debug.println(ap_cell_count2);
#endif
            break;

        case MAVLINK_MSG_ID_AHRS3: // #182   http://mavlink.org/messages/ardupilotmega
            if (!mavGood)
                break;
            break;
        case MAVLINK_MSG_ID_STATUSTEXT: // #253
            ap_severity = mavlink_msg_statustext_get_severity(&msg);
            len = mavlink_msg_statustext_get_text(&msg, ap_text);

            for (int i = 0; i <= len; i++) {                                                                  // Get real len
                if ((ap_text[i] == 32 || ap_text[i] == 0) && (ap_text[i + 1] == 32 || ap_text[i + 1] == 0)) { // find first consecutive double-space
                    len = i;
                    break;
                }
            }
            ap_text[len + 1] = 0x00;
            ap_text[len + 2] = 0x00; // mark the end of text chunk +
            ap_text[len + 3] = 0x00;
            ap_text[len + 4] = 0x00;

            ap_txtlth = len + 1;

            //   fr_chunk_pntr = 0;

            if (strcmp(ap_text, "SIMPLE mode on") == 0)
                ap_simple = true;
            else if (strcmp(ap_text, "SIMPLE mode off") == 0)
                ap_simple = false;

            if (MsgRingBuff.isFull()) {
                Debug.println("MsgRingBuff is full!");
            } else {
                ST_record.severity = ap_severity;
                memcpy(ST_record.text, ap_text, ap_txtlth + 4); // length + rest of last chunk at least
                                                                //     memcpy(&ST_record.text[0], &ap_text[0], ap_txtlth);
                ST_record.txtlth = ap_txtlth;
                ST_record.simple = ap_simple;
                MsgRingBuff.push(ST_record);
            }

#if defined Mav_Debug_All || defined Mav_Debug_Text
            Debug.print("Mavlink in #253 Statustext pushed onto MsgRingBuff: ");
            Debug.print("Queue length= ");
            Debug.print(MsgRingBuff.size());
            Debug.print(" Msg lth=");
            Debug.print(len);
            Debug.print(" Severity=");
            Debug.print(ap_severity);
            Debug.print(" ");
            Debug.print(MavSeverity(ap_severity));
            Debug.print("  Text= ");
            Debug.print(" |");
            Debug.print(ap_text);
            Debug.print("| ");
            Debug.print("  ap_simple ");
            Debug.println(ap_simple);
#endif
            break;
        default:
            if (!mavGood)
                break;
#ifdef Mav_Debug_All
//  Debug.print("Mavlink in: ");
//  Debug.print("Unknown Message ID #");
//  Debug.print(msg.msgid);
// Debug.println(" Ignored");
#endif

            break;
        }
    }
}

void MavlinkManager::Aux_ReceiveAndForward() { // up to FC, optional
#if defined Aux_Port_Enabled && defined auxDuplex
    mavlink_message_t msg;
    mavlink_status_t status;

    while (auxSerial.available()) {
        uint8_t c = auxSerial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

#ifdef Aux_Port_Debug
            Debug.println("auxSerial passed up to FC:");
            PrintMavBuffer(&msg);
#endif

            len = mavlink_msg_to_send_buffer(buf, &msg);
            mavSerial.write(buf, len);
        }
    }
#endif
}