/*
 * Justin Sapun
 * Low Level implementation for the nRF24L01 transciever.
*/

/*macros for SPI, CE and CSN pin configuration, change pins # according to mcu*/
#define MOSI_PIN    PORTB3
#define MISO_PIN    PORTB4
#define SCK_PIN     PORTB5
#define SS_PIN      PORTB2
#define NRF24_CSN   PORTB0            /*to enable SPI on nrf24, active LOW*/
#define NRF24_CE    PORTD7             /*active HIGH, activate chip in RX or TX mode*/

/*start of low level functions, specific to the mcu and compiler*/

/*delay in miliseconds*/
void delay_function(uint32_t duration_ms)
{
  _delay_ms(duration_ms);
}

/*contains all SPI configuations, such as pins and control registers*/
/*SPI control: master, interrupts disabled, clock polarity low when idle, clock phase falling edge, clock up tp 1 MHz*/
void SPI_Initializer()
{
  
  DDRB |= (1 << SS_PIN);    // Set as output
  DDRB |= (1 << MOSI_PIN);
  DDRB |= (1 << MISO_PIN);
  DDRB |= (1 << SCK_PIN);

  SPCR = 0X51;                      /*master, interrupt disabled, spi enabled, clock polarity low when idle, clock phase falling edge, 1 MHz clock*/
}

/*contains all CSN and CE pins gpio configurations, including setting them as gpio outputs and turning SPI off and CE '1'*/
void pinout_Initializer()
{
  DDRD |= (1 << NRF24_CE);  // Set as output
  DDRB |= (1 << NRF24_CSN);

  PORTB |= (1 << NRF24_CSN); // Set bit ---??? not sure if we set or clear
  //digitalWrite(NRF24_CSN, SPI_OFF);       /*nrf24l01 is not accepting commands*/
  
  nrf24_CE(HIGH);                         /*no need to change this line*/
}

/*CSN pin manipulation to high or low (SPI on or off)*/
void nrf24_SPI(uint8_t input)
{
  if (input == SPI_ON) // if SPI is 0
    PORTB &= ~(1 << NRF24_CSN); // clear
  else
    PORTB |= (1 << NRF24_CSN); // set
}

/*1 byte SPI shift register send and receive routine*/
uint8_t SPI_send_command(uint8_t command)
{
  SPDR = command;
  while ((SPSR & (1 << SPIF)) == 0) {}
  return SPDR;
}

/*CE pin maniplation to high or low*/
void nrf24_CE(uint8_t input)
{
  if (input == CE_ON) // if CE is 1
    PORTD |= (1 << NRF24_CE); // set
  else
    PORTD &= ~(1 << NRF24_CE); // clear
}