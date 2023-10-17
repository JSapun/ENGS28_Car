/*receiver code example, prints the received payload to the Serial monitor in HEX format*/
/*static payload length of 1 byte, 1Mbps datarate, -6 dbm rf transmit power, channel 32 of 125 chanels*/
#include <nRF24L01.h>

#include <stdlib.h>
#include <avr/io.h>	
#include <avr/interrupt.h> 
#include <ioE28.h>
#include <USARTE28.h>
#include <tb6612_T1.h>
#include <math.h>

const uint8_t ignition_mask = 0b10000000;
const uint8_t bits = 15;
const uint16_t full_range_max = 2000;	// milli g

/* FUNCTIONS */
void timer1_init(uint16_t timeout);

/* GLOBAL VARIABLES */
volatile uint8_t timerFlag = 0; 

/* CODE */
int main(void){

  sei();                          // Reset Ports for Interrupt
  timer1_init(15625);             // timer start with 1 s
  USART_Init();                   // Start Serial comm
  motors_init(); 
  nrf24_device(RECEIVER, RESET);                           //initializing nrf24l01+ as a receiver device with one simple function call
  nrf24_prx_static_payload_width(7, NUMBER_OF_DP_DEFAULT); // Initialize 4 recieve datapipes (only required for receive) to send int array
  
  uint8_t data[7];
 
  printf("Begin Recieving and Controlling Car!\n\r");


  
  while(1){
    //if (timerFlag){
      //timerFlag = 0;


      while(nrf24_receive(data, 7) == RECEIVE_FIFO_EMPTY);    //poll and receive in one simple function call
 
      uint16_t speed = (data[0]*100) + data[1];   // Data is organized into 2 bytes to easily transfer values up to 255255, altough we only need 1000
      uint16_t roll = (data[2]*100) + data[3];     
      uint16_t pitch = (data[4]*100) + data[5];     
      uint8_t operation = data[6];                // Includes all extra controls like ignition, headlights, tailights, etc
      
      uint8_t ignition = (operation & ignition_mask);

  
      if (!ignition){
        motors_mode(BRAKE);
        motors_mode(STOP);
        printf("Engine Stopped\n\r");
        continue;
      }
      ignition = 1; // Set to 1 to make thing simpler

      uint16_t motor1_value = 0;
			uint16_t motor2_value = 0;
      uint16_t turn = 0;

      if (roll > 200){          // Right turn
        turn = (((360 - roll)*300)/64) + 250;       
        motor1_value = speed;           // Regular speed to left wheel
        motor2_value = ((int32_t) speed*turn)/1000;      // Percentage of turn to right wheel --> turn right
      }
      else if (roll < 160){                    // Left turn
        turn = ((300*roll)/64) + 250;
        motor1_value = ((int32_t) speed*turn)/1000;  // Regular speed to right wheel
        motor2_value = speed;         // Percentage of turn to left wheel --> turn left
      }
      else {  // Not turning range
        motor1_value = speed;
        motor2_value = speed;
      }


      if ((pitch < 160) && (speed > 250)){          // FWD
        motors_mode(FWD);
      }
      else if ((pitch > 200) && (speed > 250)){     // REV
        motors_mode(REV);
      }
      else {  
        motors_mode(BRAKE);
        motors_mode(STOP);
      }

      printf("turn: %d \troll: %d \tspeed: %d \tmotor1: %d \tmotor2: %d\n\r", turn, roll, speed, motor1_value, motor2_value);

      motor1_speed(motor1_value);        // Set the speed desired, will not operate in stop mode
      motor2_speed(motor2_value);

   // }
    
    //printf("Speed: %d \tRoll: %d \tPitch: %d \tIgnition: %d\n\r", speed, roll, pitch, ignition);
  }
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
