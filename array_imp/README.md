# Data Array Transmit Implementation

With these two files, you can easily understand how to send 8-bit data arrays with the nrf24l01 transcievers. Once downloaded, hook up each Arduino to a 
seperate com port (or the same if that's easier). Then, you can designate one Arduino as a transmitter, and one as a reciever. If using seperate com ports, 
please note the changes necessary in the makefile. From there, you can run "make" and "make flash" for each Arduino. Finally, you can moniter the serial outputs
for each corresponding Arduino.
