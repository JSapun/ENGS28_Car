/*transmitter code example, transmits an ascending number every TIME_GAP milliseconds in NO_ACK mode*/
/*static payload length of 1 byte, 1Mbps datarate, -6 dbm rf transmit power, channel 32 of 125 chanels*/
#include <nrf24l01.h>
#include <ioE28.h>
#include <USARTE28.h>
 
#define TIME_GAP    500

int main(void){
  
  USART_Init();   				              // Initalize USUART COM for Screen
  nrf24_device(TRANSMITTER, RESET);     //initializing nrf24l01+ as a transmitter using one simple function

  uint8_t data[4];

  printf("Transmitter loading...\n\r");

  while(1){
    _delay_ms(TIME_GAP); 

    data[0] = 00;
    data[1] = 34;
    data[2] = 12;
    data[3] = 34;
 
    while(nrf24_transmit(&data, 4, NO_ACK_MODE) == TRANSMIT_FAIL){      //wait until payload is loaded into TX buffer
      printf("Transmit fail!\n\r");
    }
    
    while(nrf24_transmit_status() == TRANSMIT_IN_PROGRESS);      //poll the transmit status, make sure the payload is sent

    printf("Speed: %d%d, Roll: %d%d\n\r", data[0],data[1],data[2],data[3]);
  }
} 