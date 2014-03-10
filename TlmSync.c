#include "../mavlink/include/common/common.h"
#define SYNC_FLAG 0
#define NEWLINE 0x0A
#define TIME  0
#define LAT   4
#define LON   8
#define ALT   12
#define HDG   16
#define CROLL 18
#define CPITC 20
#define AROLL 22
#define APITC 26
#define VLX   30
#define VLY   32
#define VLZ   34
#define MSG_END 36
#define NOP ;

///////////////////////////////////TYPEDEFS/////////////////////////////////////
typedef struct __telem_msg_t{
  uint32_t time_boot_ms;  // Timestamp (millisecons since system boot)
  int32_t lat;            // Latitude, expressed as * 1E7
  int32_t lon;            // Longitude, expressed as * 1E7
  int32_t alt;            // Altitude in meters, expressed as * 1000 (millimeters), above MSL
  uint16_t hdg;           // Compass heading in degrees * 100, 0.00 to 359.99.  If unknown, set to: UINT16_MAX
  int16_t cam_roll;       // Camera roll angle in degrees * 10, -30.0 to 30.0 degrees.
  int16_t cam_pitch;      // Camera pitch angle in degres * 10, -30.0 to 30.0 degrees.
  float air_roll;         // Airframe roll angle in radians, -pi to +pi
  float air_pitch;        // Airframe pitch angle in radians, -pi to +pi
  int16_t vx;             // Ground x speed (Latitude), expressed as m/s * 100
  int16_t vy;             // Ground y speed (Longitude), expressed as m/s * 100
  int16_t vz;             // Vertical speed, expressed as m/s * 100
 } telem_msg_t;
///////////////////////////////////TYPEDEFS/////////////////////////////////////

//////////////////////////////////VARIABLES/////////////////////////////////////
static int flags = 0;                   // flag bitfield
static int packet_drops = 0;            // number of dropped packets
static mavlink_message_t mav_msg;       // current MAVlink message
static mavlink_status_t status;         // MAVlink status
static telem_msg_t telem_msg;           // Telemtry message
static uint8_t usart_buffer[256];       // Circular buffer for MAVlink
static uint8_t buffer_start_index = 0;  // Circular buffer beginning index
static uint8_t buffer_end_index = 0;    // Circular buffer ending index
static uint8_t *msg_buffer;         // Buffer for telemetry send
static uint16_t msg_buffer_index = 0;   // index for buffer transmit
static uint16_t msg_end_index = 0;      // last message byte
//////////////////////////////////VARIABLES/////////////////////////////////////

ISR(TIMER1_COMPA_vect){	//This is the timing interrupt for the shoot pulse. 2 Hz
  PORTE ^= (1 << 4);	//Toggle shoot pin
  flags ^= (1 << SYNC_FLAG);  //Trigger data sync
}

ISR(USART0_RX_vect){
  usart_buffer[buffer_end_index++] = UDR0;  // Get USART byte
}

ISR(USART1_TX_vect){
  if(msg_buffer_index <= msg_end_index){ // If there are still indices to be transmitted
    UDR1 = msg_buffer[msg_buffer_index++];
  }else{
    msg_buffer_index = 0; // Clear buffer
    msg_end_index = 0;
    UCSR1B &= ~(1 << TXEN1);  // Disable transmit
  }
}

void setup(){
  // setup msg_buffer
  *msg_buffer = malloc(512);

  // USART Register Set
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN);  //Enable interrupt and receive
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);  //Set 8 bit frame
  UBRR0H = (9600 >> 8); // set 9600 baud
  UBRR0L = (9600 & 0xff);
  
  // USART1 Regsiter Set
  UCSR1A = 0;
  UCSR1B |= (1 << TXCIE1); //Enable interrupt
  UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10);  // set 8 bit frame
  UBRR1H = (9600 >> 8); // set 9600 baud
  UBRR1L = (9600 & 0xff);
  
  //V_ref is 0, no need to set
  ACDSRA |= 0x01; //Prescaler set to 2
  ADCSRA |= (1 << ADEN);//enable ADC
  ADMUX |= (1 << REFS0);  // Set internal 5V reference
  //setup timer
  TCCR1B |= (1 << CS12);  //Set 256 prescaler
  OCR1AH = (31250 >> 8);  //Half second into compare register
  OCR1AL = (31250 & 0xff);
  TIMSK1 |= (1 << OCIE1A);  //Activate interrupt
}

void loop(){
  if(flags & (1 << SYNC_FLAG)){
    //Get sync data
    //Get ADC data
    ADMUX &= ~(1 << MUX0);   // Set ADC 0
    ADCSRA |= (1 << ADSC);  // Start ADC conversion

    //Get Mav Data
    while(buffer_start_index != buffer_end_index){  // when there is something in the buffer
      if(mavlink_parse_char(MAVLINK_COMM_0, usart_buffer[buffer_start_index++], &mav_msg, &status){
        switch(msg.msgid){  // check message id
          case GLOBAL_POSITION_INT:
            telem_msg.time_boot_ms = ((mavlink_msg_global_position_t) msg).time_boot_ms;
            telem_msg.lat = ((mavlink_msg_global_position_t) msg).lat;
            telem_msg.lon = ((mavlink_msg_global_position_t) msg).lon;
            telem_msg.alt = ((mavlink_msg_global_position_t) msg).alt;
            telem_msg.hdg = ((mavlink_msg_global_position_t) msg).hdg;
            telem_msg.vx = ((mavlink_msg_global_position_t) msg).vx;
            telem_msg.vy = ((mavlink_msg_global_position_t) msg).vy;
            telem_msg.vz = ((mavlink_msg_global_position_t) msg).vz;            
            break;
          case ATTITUDE:
            telem_msg.air_roll = ((mavlink_msg_global_position_t) msg).roll;
            telem_msg.air_pitch = ((mavlink_msg_global_position_t) msg).pitch;
            break;
          default:
            break;
        }
      }
    }
    
    // Update Packet Drops counter
    packet_drops += status.packet_rx_drop_count;
    
    //finish ADC data (don't waste the 13 us)
    telem_msg.cam_roll = ADCL;
    telem_msg.cam_roll += ADCH << 8;
    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);  // Start ADC conversion
    while(!(ADCSRA & (1 << ADIF))){
      NOP // Blocking to wait for ADC conversion
    }
    telem_msg.cam_pitch = ADCL;
    telem_msg.cam_pitch += ADCH << 8;
        
    //Push data
    *(msg_buffer + TIME)  = telem_msg.time_boot_ms;
    *(msg_buffer + LAT)   = telem_msg.lat;
    *(msg_buffer + LON)   = telem_msg.lon;
    *(msg_buffer + ALT)   = telem_msg.alt;
    *(msg_buffer + HDG)   = telem_msg.hdg;
    *(msg_buffer + VLX)   = telem_msg.vx;
    *(msg_buffer + VLY)   = telem_msg.vy;
    *(msg_buffer + VLZ)   = telem_msg.vz;
    *(msg_buffer + AROLL) = telem_msg.air_roll;
    *(msg_buffer + APITC) = telem_msg.air_pitch;
    *(msg_buffer + CROLL) = telem_msg.cam_roll;
    *(msg_buffer + CPITC) = telem_msg.cam_pitch;
    *(msg_buffer + MSG_END) = NEWLINE;  // add end character
    msg_end_index = MSG_END;  // update message length
    
    UDR1 = *(msg_buffer + msg_buffer_index++);  // load first byte
    UCSR1B |= (1 << TXEN1); // enable transmit
  }
}
