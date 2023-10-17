/*transmitter code example, transmits an ascending number every TIME_GAP milliseconds in NO_ACK mode*/
/*static payload length of 1 byte, 1Mbps datarate, -6 dbm rf transmit power, channel 32 of 125 chanels*/
#include <nrf24l01.h>

#include <stdlib.h>
#include <avr/io.h>	
#include <avr/interrupt.h> 
#include <ioE28.h>
#include <USARTE28.h>
#include <i2c.h>
#include <lsm303agr.h>
#include <ADC.h>
#include <math.h>

#define PI 3.14159265

const uint8_t bits = 15;
const uint16_t full_range_max = 2000;	// milli g

/* FUNCTIONS */
void timer1_init(uint16_t timeout);
void transmit_data(uint16_t speed, uint16_t roll, uint16_t pitch, uint8_t operation);

/* GLOBAL VARIABLES */
volatile uint8_t timerFlag = 0; 

/* CODE */
int main(void){

  sei();               // Reset Ports for Interrupt
  //timer1_init(15625);  // timer start with 1 s
  timer1_init(3906);  // timer start with 0.5 s
  USART_Init();        // Start Serial comm
	i2cInit();					 // Start ITC comm module
	lsm303_AccelInit();	 // Initalize Accelerometer
  ADC_Init();          // Initalize ADC
  nrf24_device(TRANSMITTER, RESET);     //initializing nrf24l01+ as a transmitter using one simple function

  int16_t accel_x, accel_y, accel_z;		// Initialize values for accerlometer outputs
	lsm303AccelData_s accel_raw;			    // Initalize value for accel raw data
  uint16_t potval;                      // Initalize value for ADC (10k pot)
  uint16_t theta_turn;
  uint16_t theta_fwd_rev;
  uint8_t controls;

  printf("Begin Sensor Reading and Transmission!\n\r");

  while(1){

    if (timerFlag){
      timerFlag = 0;

      potval = ADC_getValue();          // Read sensor data
      lsm303_AccelReadRaw(&accel_raw);

      accel_x = (float) ( ((int32_t) accel_raw.x * full_range_max) >> bits );
			accel_y = (float) ( ((int32_t) accel_raw.y * full_range_max) >> bits );
			accel_z = (float) ( ((int32_t) accel_raw.z * full_range_max) >> bits );

			theta_turn = round(atan2(accel_x, accel_z) * (180/PI))+180; 
      theta_fwd_rev = round(atan2(accel_y, accel_z) * (180/PI))+180;

      uint8_t ignition = 0b1;

      controls = (ignition << 7) | 0b0000000;

      printf("Pot Value: %d \tTurn Angle: %d \tFwd Angle: %d \tIngition: %d\n\r", potval, theta_turn, theta_fwd_rev, ignition);

      // Prepare transmission 
      transmit_data(potval, theta_turn, theta_fwd_rev, controls);

      }
  }
  return 0;
} 

ISR(TIMER1_COMPA_vect) {
  timerFlag = 1;          						        // change flag
}

void timer1_init(uint16_t timeout) {
  TCCR1B |= (1 << WGM12);						          // Set mode to CTC
  TIMSK1 |= (1 << OCIE1A);						        // Enable timer interrupt
  OCR1A = timeout;						  		          // Load timeout value
  TCCR1B |= ((1 << CS10) | (1 << CS12));		  // Set prescaler to 1024
}


void transmit_data(uint16_t speed, uint16_t roll, uint16_t pitch, uint8_t operation){ // Loads into uint8_t integer array and loads TX buffer, then transmits
  if (speed>9999){speed=9999;}  // Failsafes
  if (roll>9999){roll=9999;}  
  
  uint8_t data[7];

  if (speed > 99){
    data[0] = speed/100; // 1000s & 100s place
    speed %= 100;
  }  
  else {data[0] = 0;}
  data[1] = speed; // 10s & 1s place
 
  if (roll > 99){
    data[2] = roll/100; // 1000s & 100s place
    roll %= 100;
  }  
  else {data[2] = 0;}
  data[3] = roll; // 10s & 1s place

  if (pitch > 99){
    data[4] = pitch/100; // 1000s & 100s place
    pitch %= 100;
  }  
  else {data[4] = 0;}
  data[5] = pitch; // 10s & 1s place

  data[6] = operation;
 
  while(nrf24_transmit(data, 7, NO_ACK_MODE) == TRANSMIT_FAIL){printf("Transmit fail!\n\r");}   // Wait until payload is loaded into TX buffer

  while(nrf24_transmit_status() == TRANSMIT_IN_PROGRESS);      //poll the transmit status, make sure the payload is sent

}