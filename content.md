---
title: 'Getting Started with Opta and Finder 7M'
description: "Learn how to read 7M registers using the Modbus protocol on Opta."
author: 'Federico Di Gregorio'
libraries:
  - name: 'ArduinoRS485'
    url: https://www.arduino.cc/reference/en/libraries/arduinors485
  - name: 'ArduinoModbus'
    url: https://www.arduino.cc/reference/en/libraries/arduinomodbus
difficulty: intermediate
tags:
  - Getting-started
  - ModbusRTU
  - RS-485
  - Finder 7M
software:
  - ide-v1
  - ide-v2
  - arduino-cli
  - web-editor
hardware:
  - hardware/07.opta/opta-family/opta
---

## Overview

Among the protocols supported by the Opta, we find Modbus RTU. In this tutorial
we will learn how to implement Modbus RTU communication over RS-485 between the
Opta and a Finder 7M energy meter.In particular, we are going to learn how to
correctly convert values read from the 7M registers to valid floating point
values and upload them to the cloud.

## Goals

* Learn how to establish RS-485 interface connectivity between the Opta and
  Finder 7M devices.
* Learn how to use the Modbus RTU communication protocol to read 7M registers.
* Learn how to convert encoded values to floating point numbers.
* Learn how to upload 7M energy readings to the Arduino Cloud.

## Required Hardware and Software

### Hardware Requirements

* Opta PLC with RS-485 support (x1).
* Finder 7M energy meter (x1).
* 12VDC/1A DIN rail power supply (x1).
* USB-C® cable (x1).
* Wire with either specification for RS-485 connection (x3):
  * STP/UTP 24-18AWG (Unterminated) 100-130Ω rated
  * STP/UTP 22-16AWG (Terminated) 100-130Ω rated

### Software Requirements

* [Arduino IDE 1.8.10+](https://www.arduino.cc/en/software), [Arduino IDE
2.0+](https://www.arduino.cc/en/software) or [Arduino Web
Editor](https://create.arduino.cc/editor).
* If you choose an offline Arduino IDE, you must install the `ArduinoRS485` and
`ArduinoModbus` libraries. You can install them using the Library Manager of
the Arduino IDE.
* The [Arduino Cloud](https://create.arduino.cc/iot/things) will be required to
  store the 7M energy readings via Wi-Fi® connectivity using the sketch
  provided in the following section. The Ethernet connection is also available
  as a connectivity option to leverage Arduino Cloud applications. The Arduino
  Cloud account is free and is needed to access its features.
* [Example code](assets/Opta7MExample.zip).

## Finder 7M and the Modbus Protocol

Finder 7M energy meters provide access to a series of *input registers*
(read-only registers) via the Modbus RTU communications protocol over RS-485
serial connection.

As documented in the [7M Modbus communication protocol
document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf)
any measure accessible from the 7M display is also available on Modbus as a
series of 16-bit reads: for example the energy counter E1 is available as a
32-bit value obtained by combining the read of the two 16-bit registers located
at offsets 406 and 407. Note that all offsets are *register* offsets, not
*byte* offsets.

For more insights on the Modbus communication protocol, take a look at this
[Modbus article](https://docs.arduino.cc/learn/communication/modbus): all the
functionalities provided by the `ArduinoModbus` library are supported by Opta.

## Instructions

### Setting Up the Arduino IDE

This tutorial will need [the latest version of the Arduino
IDE](https://www.arduino.cc/en/software). If it is your first time setting up
the Opta, check out the [getting started
tutorial](/tutorials/opta/getting-started).

Make sure you install the latest version of the
[ArduinoModbus](https://www.arduino.cc/reference/en/libraries/arduinomodbus/)
and the
[ArduinoRS485](https://www.arduino.cc/reference/en/libraries/arduinors485/)
libraries, as they will be used to implement the Modbus RTU communication
protocol. Additionally, install the
[ArduinoIoTCloud](https://www.arduino.cc/reference/en/libraries/arduinoiotcloud/)
library, needed to upload the data to Arduino Cloud.

### Connecting the Opta and Finder 7M

To have some actual data to upload to the cloud you will need to connect the
Finder 7M energy meter to the power grid and provide an adequate load to be
powered via its 240V output connectors (like a lamp). Use the 12VDC/1A supply
to power the Opta and make sure to correctly setup the RS-485 serial connection
between the Opta and the 7M. You can refer to the following diagram while
connecting your Opta device to the Finder 7M energy meter via the RS-485
interface.

![Connecting Opta and Finder 7M](assets/connection.svg)

For the example code to work, you will need to set the 7M communication
parameters to:

* Modbus address `2`.
* Baudrate `19200`.
* Serial configuration `8-N-1`.
  
This can easily be done using [the Toolbox
application](https://play.google.com/store/apps/details?id=com.findernet.ToolboxNFC)
via NFC.

### Code Overview

The goal of the following example is to read some values from the Finder 7M via
Modbus and print them to the serial console for debug. Moreover, the value of
the E1 energy counter will be uploaded to the Arduino Cloud.

The full code of the example is available [here](assets/Opta7MExample.zip):
after extracting the files the sketch can be compiled and uploaded to the Opta.

Note that the `thingProperties.h` file, generated by the Arduino Cloud during
the dashboard configuration has been slightly modified to read the WiFi SSID
and password from the `config.h` file:

```cpp
#define WIFI_SECRET_SSID      "YOUR SSID"
#define WIFI_SECRET_PASSWORD  "YOUR PASSWORD"

// Read from the 7M every 10 seconds
#define READ_INTERVAL_SECONDS 10

// Use WiFi to connect to Arduino Cloud
#define ARDUINO_CLOUD_USE_WIFI    1
```

### Reading from the 7M

The following headers are required to enable the Modbus RTU protocol, the
connection with the Arduino Cloud, and to import the mathematical function
`pow()` that we'll need later.

The header file `finder-7m.h` contains all the needed definitions, like Modbus
parameters and registers offsets.

```cpp
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <ArduinoIoTCloud.h>
#include <math.h>
#include "finder-7m.h"
#include "config.h"
#include "thingProperties.h"

const uint8_t MODBUS_7M_ADDRESS = 2;

void setup()
{
    Serial.begin(9600);

    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LED_D0, HIGH);
    digitalWrite(LED_D1, HIGH);
    digitalWrite(LED_D2, HIGH);
    digitalWrite(LED_D3, HIGH);

    delay(2000);

    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
    digitalWrite(LED_D0, LOW);
    digitalWrite(LED_D1, LOW);
    digitalWrite(LED_D2, LOW);
    digitalWrite(LED_D3, LOW);

    Serial.println("Finder Opta + 7M example: setup");

    RS485.setDelays(MODBUS_PRE_DELAY_BR, MODBUS_POST_DELAY_BR);

    ModbusRTUClient.setTimeout(200);

    if (ModbusRTUClient.begin(MODBUS_BAUDRATE, MODBUS_SERIAL_PARAMETERS))
    {
        Serial.println("Modbus RTU client started");
    }
    else
    {
        Serial.println("Failed to start Modbus RTU client: reset board to restart.");
        while (1) {}
    }
}
```

The leds on the Opta are flashed to show that we're executing the `setup()`,
then the RS-485 is configured with the Modbus parameters according to [the
  Modbus over serial line
  guide](https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf). The
  Baudrate is set to `19200`, while the serial configuration is `8-N-1`.

The `loop()` function contains the code that actually reads some registers from
the 7M and then prints the values to the serial console for debug:

```cpp
void loop()
{
    uint32_t data;

    Serial.println("** Reading 7M at address " + String(MODBUS_7M_ADDRESS));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ENERGY_COUNTER_XK_E1);
    Serial.println("   energy = " + (data != INVALID_DATA ? String(data) : String("read error")));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_RUN_TIME);
    Serial.println("   run time = " + (data != INVALID_DATA ? String(data) : String("read error")));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_FREQUENCY);
    Serial.println("   frequency = " + (data != INVALID_DATA ? String(convert_t5(data)) : String("read error")));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_U1);
    Serial.println("   voltage = " + (data != INVALID_DATA ? String(convert_t5(data)) : String("read error")));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ACTIVE_POWER_TOTAL);
    Serial.println("   active power = " + (data != INVALID_DATA ? String(convert_t6(data)) : String("read error")));
}
```

In order we read:

* The energy counter E1 in its x1000 version (in 0.1 Wh increments).
* The total run time (s).
* The frequency of the input AC (Hz).
* The voltage of the input AC (V).
* The total instantaneous active power (W).

All the values are 32-bits wide, so we can use a single function to obtain the
raw data for all of them:

```cpp
uint32_t modbus_7m_read32(uint8_t addr, uint16_t reg) 
{
    uint8_t attempts = 3;

    while (attempts > 0)
    {
        digitalWrite(LED_D0, HIGH);

        ModbusRTUClient.requestFrom(addr, INPUT_REGISTERS, reg, 2);
        uint32_t data1 = ModbusRTUClient.read();
        uint32_t data2 = ModbusRTUClient.read();

        digitalWrite(LED_D0, LOW);

        if (data1 != INVALID_DATA && data2 != INVALID_DATA)
        {
            return data1 << 16 | data2;
        }
        else
        {
            attempts -= 1;
            delay(10);
        }
    }

    return INVALID_DATA;
}
```

The `modbus_7m_read32()` function reads two consecutive 16-bits registers
starting from offset `reg` from device with Modbus address `addr`, and composes
them into a single 32-bits value by shifting the first value right by 16 bits.
`ModbusRTUClient.read()` always returns a 32-bit result, with the value `-1`
(`0xFFFFFFFF`) indicating an error. If the code detects any problem, it tries
to perform the read up to three times, before giving up and returining the
error value to signal the calling code in `loop()`.

Some values, like the total run time or the content of counter E1, are real
32-bits  values (indicated by the type codes `T2` and `T3` in the [7M Modbus
communication protocol
document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf))
and they can be used without further processing. Unfortunately other values,
eg. the input current frequency or its voltage, use a special encoding and need
to be decode before they can be used.

```cpp
float convert_t5(uint32_t n)
{
    uint32_t s = (n & 0x80000000) >> 31;
    int32_t e = (n & 0x7F000000) >> 24;
    if (s == 1) {
        e = e - 0x80;
    }
    uint32_t m = n & 0x00FFFFFF;
    return (float)m * pow(10, e);
}
```

The `convert_t5()` function can be used to covert any value using the `T5`
encoding to the corresponding float value. As explained in the [7M Modbus
communication protocol
document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf)
`T5` means that the 32 bits are used as follows:

* The most significant 8 bits are a signed exponent between -128 and 127.
* The least significant 24 bits are an unsigned number or "mantissa".

The code extracts the sign `s` and the unsigned exponent `e`, then depending on
the sign determines the final, signed value of the exponent. The `pow()`
function is used to elevate the mantissa `m` to the power of `e` and the result
is returned as a float.

### Sending data to the cloud

To send data read from the 7M to the cloud you will need to setup an Arduino
Cloud account, register your Opta device, assign it to a thing and add a
property for every variable you want to upload.

In this example we will send to the could a single value: the energy counter
E1. After adding the single `energy` property we can copy the code generated by
the Cloud IDE into our sketch `thingProperties.h`:

```cpp
#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "config.h"

const char SSID[] = WIFI_SECRET_SSID;
const char PASS[] = WIFI_SECRET_PASSWORD;

float energy;

void initProperties()
{
    ArduinoCloud.addProperty(energy, Permission::Read);
}

#if ARDUINO_CLOUD_USE_WIFI == 1
    WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
#else
    EthernetConnectionHandler ArduinoIoTPreferredConnection;
#endif
```

The constants `WIFI_SECRET_SSID`, `WIFI_SECRET_PASSWORD` and
`ARDUINO_CLOUD_USE_WIFI` are defined in `config.h` and allow to configure the
Opta for a WiFi or Ethernet network connection.

The `setup()` function will need some additional code to correctly setup the
connection to the Arduino Cloud, most notably:

```cpp
    ...

    initProperties();

    setDebugMessageLevel(2);
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT, iotConnect);
    ArduinoCloud.addCallback(ArduinoIoTCloudEvent::DISCONNECT, iotDisconnect);
    ArduinoCloud.printDebugInfo();
}
```

## Using the Finder7M library

To simplify all the tasks we performed in this tutorial, it is possible to use
the `Finder7M`. In this case, the `setup()` code is much easier because the
library itself provides built-in functions to configure the RS-485 paramaters:

```cpp
Finder7M f7m;

void setup()
{
    Serial.begin(9600);
    
    f7m.init();

    // Arduino Cloud or other initialization code goes here.
}
```

The `loop()` is also simpler:

```cpp
void loop()
{
    Serial.println("** Reading 7M at address " + String(MODBUS_7M_ADDRESS));

    data = f7m.modbus7MRead32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ENERGY_COUNTER_XK_E1);
    Serial.println("   energy = " + (data != INVALID_DATA ? String(data) : String("read error")));

    Measure a = f7m.getMIDInActiveEnergy(MODBUS_7M_ADDRESS);
    Serial.println("   IN active energy = " + String(a.toFloat()));
}
```

To learn more about the library head over to the [official
repository](https://github.com/dndg/Finder7M).

## Reading from multiple Finder7M devices

If we want to read from the registers of more than one 7M, we can initialize in
our program an array containing the Modbus addresses of the devices that we
want to interact with:

```cpp
const uint8_t addresses[4] = {6, 10, 11, 13};
```

In the `loop()` function we can interate over it in a `for` loop and use the
`Finder7M` library to perform reads:

```cpp
void loop()
{
    for (int i = 0; i < sizeof(addresses); i++)
    {
        Serial.println("** Reading 7M at address " + String(addresses[i]));

        data = f7m.modbus7MRead32((addresses[i]), FINDER_7M_REG_ENERGY_COUNTER_XK_E1);
        Serial.println("   energy = " + (data != INVALID_DATA ? String(data) : String("read error")));

        Measure a = f7m.getMIDInActiveEnergy(addresses[i]);
        Serial.println("   IN active energy = " + String(a.toFloat()));
    }
}
```

## Conclusion

This tutorial demonstrates how to use the `ArduinoRS485` and `ArduinoModbus`
libraries to implement the Modbus RTU protocol between the Opta and a Finder 7M
energy meter. Additionally, it shows how it is possible to use the `Finder7M`
library to easily read counters and other values from a 7M.
