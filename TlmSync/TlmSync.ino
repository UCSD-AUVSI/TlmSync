/**
 * This program is the Telemtry Synchronization Code for the 2014 AUVSI
 * competition.
 * 
 * 
 */
 
// Includes
#include <GCS_MAVLink.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Defines
#define _MavLinkSerial  Serial2 // MavLink Rx Interface
#define _OBCSerial      Serial  // OBC Tx/Rx Interface
#define SHOOT_PERIOD    1       // Shooting Period (in seconds)
#define LED             13      // LED Pin
#define	SHOOT_PIN		23		// Trigger Pin
#define G_ROLL_PIN		22		// Analog Input for Gimbal Roll
#define G_PITCH_PIN		21		// Analog Input for Gimbal Pitch

// Message Variable Structures
// Message #0  HEARTHBEAT
uint8_t     ap_type = 0;
uint8_t     ap_autopilot = 0;
uint8_t     ap_base_mode = 0;
uint32_t    ap_custom_mode = 0;
uint8_t     ap_system_status = 0;
uint8_t     ap_mavlink_version = 0;

// Message #24  GPS_RAW_INT
uint8_t     ap_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t     ap_sat_visible = 0;           // numbers of visible satelites
int32_t     ap_latitude = 0;              // 585522540;
int32_t     ap_longitude = 0;            // 162344467;
int32_t     ap_gps_altitude = 0;        // 1000 = 1m
uint64_t    ap_time_usec = 0;

// Message #30 ATTITUDE
float       ac_roll = 0;
float       ac_pitch = 0;

// Message #33 GLOBAL_POSITION_INT
int16_t     ac_hdg = 0;
int32_t     ac_lat = 0;
int32_t     ac_lon = 0;
int32_t     ac_alt = 0;
int32_t     ac_ralt = 0;
int16_t     ac_vx = 0;
int16_t     ac_vy = 0;
int16_t     ac_vz = 0;

// Gimbal Angles
int16_t     g_roll = 0;
int16_t     g_pitch = 0;

// *******************************************
uint8_t     MavLink_Connected;
uint16_t    hb_count;

unsigned long MavLink_Connected_timer;

String inputString = "";

mavlink_message_t msg;

IntervalTimer triggerTimer;
uint32_t flags = 0;

// ******************************************
void setup()  {
    _MavLinkSerial.begin(57600);
    _OBCSerial.begin(57600);
    MavLink_Connected = 0;
    MavLink_Connected_timer=millis();
    hb_count = 0;


    pinMode(LED,OUTPUT);
    pinMode(SHOOT_PIN, OUTPUT);
    pinMode(G_ROLL_PIN, INPUT);
    pinMode(G_PITCH_PIN, INPUT);
    pinMode(12,OUTPUT);
    pinMode(14,INPUT);
    pinMode(0, INPUT);
    pinMode(1, OUTPUT);
}

void triggerShoot(){
    digitalWrite(SHOOT_PIN, flags & (1 << 0));
    if(flags & (1 << 0) && MavLink_Connected){
        // get analog
        g_roll = analogRead(G_ROLL_PIN);
        g_pitch = analogRead(G_PITCH_PIN);
		// Translate Analog
        // Convert ap_time_usec to be based off last 4th hour instead of epoch
		_OBCSerial.print((uint32_t)(ap_time_usec % (uint32_t)4000000000));
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_roll, 3);  // radians
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_pitch, 3); // radians
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_hdg * 0.000174532952, 3);  //radians
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_lat / 10000000.0, 7);  // degrees
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_lon / 10000000.0, 7);  // degrees
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_alt * 0.0003048, 1);   // feet
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_ralt * 0.0003048, 1);   // feet
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_vx / 100.0, 3);
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_vy / 100.0, 3);
        _OBCSerial.write('\t');
        _OBCSerial.print(ac_vz / 100.0, 3);
        _OBCSerial.write('\n');
    }
    flags ^= (1 << 0);
}

// ******************************************
void loop() {
    uint16_t len;

    if((millis() - MavLink_Connected_timer) > 1500)  {   // if no HEARTBEAT from APM  in 1.5s then we are not connected
        MavLink_Connected = 0;
        hb_count = 0;
    }

    _MavLink_receive();                   // Check MavLink communication

    // Check for confirm start trigger
    while(_OBCSerial.available()){
        uint8_t inputChar = (uint8_t)_OBCSerial.read();
        inputString += inputChar;
        if(inputChar == '\n'){	// Require newline message deliminator
            if(inputString.equals("1\n")){	// Message is "1"
                // Initialize trigger IRQ
				triggerTimer.begin(triggerShoot, 500000 * SHOOT_PERIOD);
            }
            if(inputString.equals("0\n")){	// Message is "0"
                triggerTimer.end();	// Disable trigger
            }
            inputString = "";	// Clear message buffer
        }
    }
}

void _MavLink_receive() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while(_MavLinkSerial.available()){
        uint8_t inputChar = _MavLinkSerial.read();
        if(mavlink_parse_char(MAVLINK_COMM_0, inputChar, &msg, &status)){
            switch(msg.msgid){
                case MAVLINK_MSG_ID_HEARTBEAT:  // 0
                    ap_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
                    ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    MavLink_Connected_timer = millis();
                    hb_count++;
                    if((hb_count++) > 10){        // If  received > 10 heartbeats from MavLink then we are connected
                        MavLink_Connected = 1;
                        hb_count = 0;
                        digitalWrite(LED, HIGH);      // LED will be ON when connected to MavLink, else it will slowly blink
                    }
                    break;
                case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
                    ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
                    ap_sat_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
                    gps_status = (ap_sat_visible * 10) + ap_fixtype;
                    ap_time_usec = mavlink_msg_gps_raw_int_get_time_usec(&msg);
                    if(ap_fixtype == 3){
                        ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                        ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
                        ap_gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);    // 1m =1000
                    }
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:   //30
                    ac_roll = mavlink_msg_attitude_get_roll(&msg);
                    ac_pitch = mavlink_msg_attitude_get_pitch(&msg);
                    break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //33
                    ac_lat = mavlink_msg_global_position_int_get_lat(&msg);
                    ac_lon = mavlink_msg_global_position_int_get_lon(&msg);
                    ac_alt = mavlink_msg_global_position_int_get_alt(&msg);
                    ac_ralt = mavlink_msg_global_position_int_get_relative_alt(&msg);
                    ac_vx = mavlink_msg_global_position_int_get_vx(&msg);
                    ac_vy = mavlink_msg_global_position_int_get_vy(&msg);
                    ac_vz = mavlink_msg_global_position_int_get_vz(&msg);
                    ac_hdg = mavlink_msg_global_position_int_get_hdg(&msg);
                    break;
                default:
                    break;
            }
        }
    }
}
