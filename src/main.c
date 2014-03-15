////////////////////////////////////INCLUDES////////////////////////////////////
#include "/Libraries/mavlink/include/common/common.h"
#include <stm32f30x.h>
#include <stm32f30x_gpio.h>
////////////////////////////////////INCLUDES////////////////////////////////////

/////////////////////////////////////DEFINES////////////////////////////////////
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
#define APTIC 26
#define VLX   30
#define VLY   32
#define VLZ   34
#define MSG_END 36
#define NOP ;
/////////////////////////////////////DEFINES////////////////////////////////////

////////////////////////////////////TYPEDEFS////////////////////////////////////
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
////////////////////////////////////TYPEDEFS////////////////////////////////////

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

int main(void){
  //SETUP
  // setup msg_buffer
  *msg_buffer = malloc(512);

  // USART1 Register Set
  // Enable Rx interrupt, Rx enable, 8N1 @ 112500
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // Enable USART1 clk
  // set PA10 (Rx)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable GPIOA clk
  // Pin, Mode, Speed, OType, PuPd
  GPIO_InitTypeDef GPIOInitStruct = {GPIO_Pin_10, GPIO_Mode_AF, GPIO_Speed_50MHz\
      , GPIO_OType_PP, GPIO_PuPd_UP};
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
  GPIO_Init(GPIOA, GPIOInitStruct);
  // Set 112500 baud, 8N1, Rx only, no HFC
  USART_InitTypeDef USARTInitStruct = {112500, USART_WordLength_8b, \
      USART_StopBits_1, USART_Parity_No, USART_Mode_Rx, \
      USART_HardwareFlowControl_None};
  USART_Init(USART1, USARTInitStruct);
  // Enable Rx Interrupt
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  
  // USART2 Regsiter Set
  // Enable RxTx interrupt, RxTx enable, 8N1 @ 112500
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART2, ENABLE);  // Enable USART2 clk
  // output PB3 (Tx), PB4 (Rx)
  RCC_AHGPeriphClockCmd(RCC_AHBPeriphGPIOA, ENABLE);  // Enable GPIOB clk
  // Pin, Mode, SPeed, OType, PuPd
  // Set PB4
  GPIOInitStruct = {GPIO_Pin_4, GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP,\
      GPIO_PuPd_UP};
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_7);
  GPIO_Init(GPIOB, GPIOInitStruct);
  // Set PB3
  GPIOInitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_7);
  GPIO_Init(GPIOB, GPIOInitStruct);
  // Set USART using old stuff
  USARTInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // set both Rx/Tx
  USART_Init(USART2, USARTInitStruct);
  // Enable Rx Interrupt
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  // Setup ADC1
  // Set clock for high speed
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
  // Supply RCC clk to ADC
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12);
  // Setup GPIO in analog pins 
  // Use PF0 and PF1 for roll and pitch
  GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIOInitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOF, GPIOInitStruct);
  GPIOInitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOF, GPIOInitStruct);

  //setup timer 2Hz OCR
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1, ENABLE);  // enable TIM1 clk

}


