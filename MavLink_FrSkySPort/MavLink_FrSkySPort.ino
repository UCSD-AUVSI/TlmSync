#include <avr/io.h>
#include <avr/interrupt.h>
/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Teensy 3.1:
SPort S --> TX1
SPort + --> Vin
SPort  - --> GND

APM Telemetry DF13-5  Pin 2 --> RX2
APM Telemetry DF13-5  Pin 3 --> TX2
APM Telemetry DF13-5  Pin 5 --> GND

Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V )


This is the data we send to FrSky, you can change this to have your own
set of data
******************************************************
Data transmitted to FrSky Taranis:
Cell           ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S )
Cells         ( Voltage from LiPo [V] )
A2             ( Analog voltage from input A0 on Teensy 3.1 )
Alt             ( Altitude from baro.  [m] )
GAlt          ( Altitude from GPS   [m])
HDG         ( Compass heading  [deg])
Rpm         ( Throttle when ARMED [%] )
AccX         ( AccX m/s ? )
AccY         ( AccY m/s ? )
AccZ         ( AccZ m/s ? )
VSpd        ( Vertical speed [m/s] )
Speed      ( Ground speed from GPS,  [km/h] )
T1            ( GPS status = ap_sat_visible*10) + ap_fixtype )
T2            ( ARMED=1, DISARMED=0 )
Vfas          ( same as Cells )
Longitud
Latitud
Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position

******************************************************

*/


/* CHANGELOG
 * 18 May 2014
 * * Added code for camera trigger, using the Periodic Interrupt Timer
 */
#include <GCS_MAVLink.h>

#define _MavLinkSerial      Serial2
#define _OBCSerial          Serial
#define START                   1
#define MSG_RATE            10              // Hertz
#define SHOOT_PERIOD        1   // seconds

// ******************************************
// Message #0  HEARTHBEAT
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t  ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message # 1  SYS_STATUS
uint16_t  ap_voltage_battery = 0;    // 1000 = 1V
int16_t    ap_current_battery = 0;    //  10 = 1A

// Message #24  GPS_RAW_INT
uint8_t    ap_fixtype = 3;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;           // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition.
int32_t    ap_latitude = 0;              // 585522540;
int32_t    ap_longitude = 0;            // 162344467;
int32_t    ap_gps_altitude = 0;        // 1000 = 1m
uint64_t     ap_time_usec = 0;


// Message #74 VFR_HUD
int32_t    ap_airspeed = 0;
uint32_t  ap_groundspeed = 0;
uint32_t  ap_heading = 0;
uint16_t  ap_throttle = 0;

// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    ap_bar_altitude = 0;    // 100 = 1m
int32_t    ap_climb_rate=0;        // 100= 1m/s

// Message #27 RAW IMU
int32_t   ap_accX = 0;
int32_t   ap_accY = 0;
int32_t   ap_accZ = 0;

// Message #30 ATTITUDE
float ac_roll = 0;
float ac_pitch = 0;


// Message #33 GLOBAL_POSITION_INT
int16_t ac_hdg = 0;
int32_t ac_lat = 0;
int32_t ac_lon = 0;
int32_t ac_alt = 0;
int32_t ac_ralt = 0;
int16_t ac_vx = 0;
int16_t ac_vy = 0;
int16_t ac_vz = 0;

// Gimbal Angles
int16_t g_roll = 0;
int16_t g_pitch = 0;

// ******************************************
// These are special for FrSky
int32_t     vfas = 0;                // 100 = 1,0V
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype
// ex. 83 = 8 sattelites visible, 3D lock
uint8_t   ap_cell_count = 0;

// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;

unsigned long MavLink_Connected_timer;
unsigned long acc_timer;

String inputString = "";

int LED = 13;

mavlink_message_t msg;

IntervalTimer triggerTimer;
uint32_t flags = 0;

// ******************************************
void setup()  {
    _MavLinkSerial.begin(57600);
    _OBCSerial.begin(57600);
    MavLink_Connected = 0;
    MavLink_Connected_timer=millis();
    acc_timer=millis();
    hb_count = 0;


    pinMode(LED,OUTPUT);
    pinMode(12,OUTPUT);

    pinMode(14,INPUT);
    analogReference(DEFAULT);

    pinMode(23, OUTPUT);

    pinMode(22, INPUT);
    pinMode(21, INPUT);
    pinMode(0, INPUT);
    pinMode(1, OUTPUT);
    //triggerTimer.begin(triggerShoot, 500000);
    Serial.println("Teensy Board v1.0b1");
}

void triggerShoot(){
    digitalWrite(23, flags & (1 << 0));
    if(flags & (1 << 0) && MavLink_Connected){
        // get analog
        g_roll = analogRead(22);
        g_pitch = analogRead(21);
        _OBCSerial.print((uint32_t)(ap_time_usec%(uint32_t)4000000000));
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
        //_OBCSerial.write('\t');

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
        if(inputChar == '\n'){
            if(inputString.equals("1\n")){
                triggerTimer.begin(triggerShoot, 500000 * SHOOT_PERIOD);
            }
            if(inputString.equals("0\n")){
                triggerTimer.end();
            }
            inputString = "";
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
                case MAVLINK_MSG_ID_SYS_STATUS:   // 1
                    ap_voltage_battery = Get_Volt_Average(mavlink_msg_sys_status_get_voltage_battery(&msg));  // 1 = 1mV
                    ap_current_battery = Get_Current_Average(mavlink_msg_sys_status_get_current_battery(&msg));     // 1=10mA
                    if(ap_voltage_battery > 21000){
                        ap_cell_count = 6;
                    }else if(ap_voltage_battery > 16800 && ap_cell_count != 6){
                        ap_cell_count = 5;
                    }else if(ap_voltage_battery > 12600 && ap_cell_count != 5){
                        ap_cell_count = 4;
                    }else if(ap_voltage_battery > 8400 && ap_cell_count != 4){
                        ap_cell_count = 3;
                    }else if(ap_voltage_battery > 4200 && ap_cell_count != 3){
                        ap_cell_count = 2;
                    }else{
                        ap_cell_count = 0;
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
                case MAVLINK_MSG_ID_RAW_IMU:   // 27
                    ap_accX = mavlink_msg_raw_imu_get_xacc(&msg) / 10;                //
                    ap_accY = mavlink_msg_raw_imu_get_yacc(&msg) / 10;
                    ap_accZ = mavlink_msg_raw_imu_get_zacc(&msg) / 10;
                    break;
                case MAVLINK_MSG_ID_VFR_HUD:   //  74
                    ap_airspeed = 0;
                    ap_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
                    ap_heading = mavlink_msg_vfr_hud_get_heading(&msg);     // 100 = 100 deg
                    ap_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);        //  100 = 100%
                    ap_bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg) * 100;        //  m
                    ap_climb_rate=mavlink_msg_vfr_hud_get_climb(&msg) * 100;        //  m/s
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



