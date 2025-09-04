# DESIGN / ARCHITECTURE explenation

## structure

unified comms exist out of 4 major components

1) unifiedController (manages connections and drivers)
2) driver interfaces
3) driver implemenations
4) BSP (board support files, board declaration)

## creating a new driver

unified comms works on a system of drivers.
each driver has to inherit from a driver interface: `cellularDriver`,`wifiDriver` etc...

By doing this we can support an endless amount of driver implementations and be future proof.

### NOTE

each driver must have its own logging TAG:

```CPP
    constexpr const char* DRIVER_GM02S = "SequansGM02S";
```

## BSP (board support files)

Board support files define the drivers and name of each board definition.

A BSP must contain the folloing:

1) driver hardware configurations
2) driver instances
3) a BSP() macro instanciation

Look to [walter.hpp](./src/bsp/walter.hpp) for an example.

## Connecting a driver

driver connection happens in `2 major steps`.

1) connection to the network (driver specific)
2) IP address aquisition. (IP_EVENT)

when these two conditions are true a driver can be considered connected.
As soon as one of these conditions is no longer true the driver is no longer connected and the unifiedcontroller must be notified.
This can be done useing the launchDisconnected event function.
Then to modem will reattempt to connect a driver or find a connected driver.

## TODO

1) add unit tests
2) test reconnect/ failover from wifi to gm02s (problem lies with GM02S)
3) create proper examples
4) add clang-format file and enforce using CI/CD
5) implement modem simulator for easier testing
6) rewrite the readme.
7) PR/ISSUE templates.
8) add possibilty to disable driver.

