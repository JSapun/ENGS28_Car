/*receiver code example, prints the received payload to the Serial monitor in HEX format*/
/*static payload length of 1 byte, 1Mbps datarate, -6 dbm rf transmit power, channel 32 of 125 chanels*/
#include <nRF24L01.h>
#include <ioE28.h>
#include <USARTE28.h>

int main(void){
  
  USART_Init();   				            // Initalize USUART COM for Screen
  nrf24_device(RECEIVER, RESET);      // Initializing nrf24l01+ as a receiver device with one simple function call
  nrf24_prx_static_payload_width(4, NUMBER_OF_DP_DEFAULT); // Initialize 4 recieve datapipes (only required for receive) to send int array

  uint8_t data[4];

  printf("Receiver loading...\n\r");

  while(1){

    while(nrf24_receive(&data, 4) == RECEIVE_FIFO_EMPTY);    //poll and receive in one simple function call

    printf("Speed: %d%d, Roll: %d%d\n\r", data[0],data[1],data[2],data[3]); 

    for (uint8_t i=0; i<4; i++){data[i]=0;}    // Reset Values
  }
}   