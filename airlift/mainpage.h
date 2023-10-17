/** \mainpage
 *
 * \section Introduction
 * 
 * We are using the Adafruit Airlift Shield for our IoT exploration.  Details about it can
 * be found at https://www.adafruit.com/product/4285.
 *
 * Adafruit provides a library for use in the Arduino environment which enables access to the
 * WiFi functionality of the ESP32 chip contained on the shield.  Professor Bonfert-Taylor has
 * re-written the library for use with the Arduino Uno and our bare-metal approach.
 *
 * We have provided some sample code to help you connect to Adafruit's MQTT service
 * (https://io.adafruit.com).  Note: For simplicity, we are using HTTP to connect to the service.
 * HTTP is the standard web query protocol.  It is relatively easy to manually create and
 * process HTTP queries and responses.
 *
 * Our sample code aims to remove much of the tedium associated with establishing and maintaining
 * wireless connections to commercial services.  In doing so, it limits the available services a
 * bit.
 *
 * If you find the sample code lacking, dig a bit deeper into the driver and modify it to suit
 * your needs.  You can always go back to the original if you mess things up!
 *
 * \section generalTemplate General Template
 *
 * Here is a general outline of the steps required to interact with data using a feed at
 * https://io.adafruit.com/.
 *
 * -# Define the feed you are connecting to and initialize any variables you need.
 * -# Initialize any peripherals you are using \ref SPIinit must be called.  Others
 *     such as `USART_Init` can also be called
 * -# Connect to the network.
 * -# Receive Input
 *    - Consult local sensors _or_
 *    - Query the remote feed
 * -# React to the input
 *    - Enable a local actuator _or_
 *    - Send data to a remote feed
 * -# Delay a bit so as to not overwhelm the service provider
 *
 * \section sampleCode Sample Code
 *
 * There are 5 sample programs provided with this library.  Their names provide pretty
 * good insight into what they do.  The first two just connect to the Airlift Shield.  The
 * others interact with the remote service.
 *
 * -# airlift/Airlift_blinky.c: Blink the RGB LED on the shield.
 * -# airlift/Airlift_TestWifi.c: Connect to the network specified in the secrets file.
 *
 * -# airlift/AdafruitIOTestGetDataWithLibrary.c: Retrieve some data from the feed using an HTTP GET request
 * -# airlift/AdafruitIOTestPostDataWithLibrary.c: Send some data to the feed using an HTTP POST request
 * -# airlift/AdafruitIOTestLEDOnOffWithLibrary.c: Toggle an LED based on the current value of the feed
 *
 * Let's take a look in some detail at \ref AdafruitIOTestLEDOnOffWithLibrary.c
 *
 * The program starts off with the normal preamble.
 *
 * \snippet AdafruitIOTestLEDOnOffWithLibrary.c Preamble
 *
 * The credentials section comes next.  It simply brings your information from the mySecrets.h file into the
 * main loop.
 *
 * \snippet AdafruitIOTestLEDOnOffWithLibrary.c Credentials
 *
 * In main, we initialize the name of the feed, any additional variables we'll need and the peripherals we are
 * using.
 *
 * \snippet AdafruitIOTestLEDOnOffWithLibrary.c Initialize
 *
 * Once everything is initialized, we need to connect to the local network.
 *
 *
 * \snippet AdafruitIOTestLEDOnOffWithLibrary.c Netconnect
 *
 * In the main forever loop, we attempt to connect to the desired feed.  If that succeeds,
 * we query it for the most recent value and up our actuator based on the result.  Note: the
 * LED on the Airlift Shield is actually 3 LEDs (Red, Green, and Blue) all in the same package.
 *
 * One of the driver functions (\ref ESP32setLEDs) allows you to set the color of the LED.
 *
 * \snippet AdafruitIOTestLEDOnOffWithLibrary.c MainLoop
 *
 *
 * \defgroup Wifi Wifi Related Functions
 *
 * These functions all pertain to connecting to (or maintaining a connection with) a WiFi network.
 *
 * They are very high level and perform a fair amount of work under the hood.  Most functions in
 * this group make use of the ESP32 specific driver functions described in \ref ESPFunctions.
 *
 * \defgroup ESPFunctions ESP32 driver functions.
 *
 * These functions communicate over SPI with the ESP32 co-processor located on the Airlift shield.
 *
 *
 * \defgroup Html HTML Functions
 *
 * These are some utility function to parse the returned HTML response.
 *
 * \defgroup Http HTTP Functions
 *
 * These functions create and process HTTP requests for an established client.
 *
 *
 */
 
