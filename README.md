# Walter unified communications library

## Introduction 

Walter is a multi-radio IoT module that combines an Espressif ESP32-S3 together with a Sequans 
Communications GM02SP. This means that a single Walter module offers connectivity over LTE Cat-M1,
LTE Cat-NB1, LTE Cat-NB2, WiFi, Bluetooth Low energy and also has a GNSS receiver that works with
the GPS and GLONASS satellite constellations.

The GM02SP modem offers an AT command interface over a UART connection. It is a quite cumbersome
task to implement an efficient AT command line interface. Therefore DPTechnics has created the 
`WalterModem` library, an efficient wrapper around the AT commands. While applications can easily
be developed on top of `WalterModem` they can be relatively verbose and it is still up to the
application developer to handle the various cellular error conditions. 

The `WalterComm` unified communications library is designed to offer a more high level approach to
network programming the Walter module. The functionality offered by this library consists of:

 - Transparent switching between WLAN, LTE-M and NB-IoT
 - Transparent radio specific error handling and retry mechanisms
 - Transparant protocol multiplexing
 - Easy certificate management
