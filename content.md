---
title: 'Getting Started with Opta™ and Finder 7M'
description: "Learn how to read 7M registers using the Modbus protocol on Opta™."
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

The Modbus RTU protocol is one of the protocols available within Opta™. In this tutorial, we will learn how to implement Modbus RTU communications protocol over RS-485 between an Opta™ devices and a Finder 7M energy meter. We will also learn how to correctly convert values read from the 7M registers to valid floating point values and upload them to the cloud.

## Goals

- Learn how to establish RS-485 interface connection between the Opta™ and Finder 7M devices
- Learn how to use the Modbus RTU communication protocol to read 7M registers
- Learn how to convert encoded values to floating point numbers
- Learn how to upload 7M energy readings to the Arduino Cloud

## Required Hardware and Software

### Hardware Requirements

- Opta™ PLC with RS-485 support (x1)
- Finder 7M energy meter (x1)
- 12VDC/1A DIN rail power supply (x1)
- USB-C® cable (x1)
- Wire with either specification for RS-485 connection (x3):
- STP/UTP 24-18AWG (Unterminated) 100-130Ω rated
- STP/UTP 22-16AWG (Terminated) 100-130Ω rated

### Software Requirements

- [Arduino IDE 1.8.10+](https://www.arduino.cc/en/software), [Arduino IDE 2.0+](https://www.arduino.cc/en/software), or [Arduino Web Editor](https://create.arduino.cc/editor)
- If you choose an offline Arduino IDE, you must install the following libraries: `ArduinoRS485`, and `ArduinoModbus`. You can install these libraries via Library Manager of the Arduino IDE.
- The [Arduino Cloud](https://create.arduino.cc/iot/things) will be required to store the 7M enery readings via Wi-Fi® connectivity using the sketch provided in the following section. The Ethernet connection is also available as a connectivity option to leverage Arduino Cloud applications. The Arduino Cloud account is free and is needed to access its features.
- [Example code](assets/Opta_7M_Example.zip)

## Finder 7M and the Modbus Protocol

Finder 7M energy meters provide access to a series of *input registers* (read-only registers) via the Modbus RTU communications protocol over RS-485 serial connection.

As documented in the [7M Modbus communication protocol document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf) any measure accessible from the 7M display is also available on Modbus as a series of 16-bit reads: for example the energy counter E1 is available as a 32-bit value obtained by combining the read of the two 16-bit registers located at offsets 406 and 407. Note that all offsets are *register* offsets, not *byte* offsets.

If you want more insights on the Modbus communication protocol, take a look at this [Modbus article](https://docs.arduino.cc/learn/communication/modbus): all the functionality provided by the `ArduinoModbus` library is supported by Opta™.

## Instructions

### Setting Up the Arduino IDE

If you haven't already, head over [here](https://www.arduino.cc/en/software) and install the most recent version of the Arduino IDE along with the necessary device drivers for your computer. For additional details on Opta™, check out [getting started tutorial](/tutorials/opta/getting-started). Make sure you install the latest version of the [ArduinoModbus](https://www.arduino.cc/reference/en/libraries/arduinomodbus/) and the [ArduinoRS485](https://www.arduino.cc/reference/en/libraries/arduinors485/) libraries, as they will be used to implement the Modbus RTU communication protocol. Also install the [ArduinoIoTCloud](https://www.arduino.cc/reference/en/libraries/arduinoiotcloud/) library, needed to upload the data to Arduino Cloud.

### Connecting the Opta™ and Finder 7M

To have some actual data to upload to the cloud you will need to connect the Finder 7M energy meter to the power grid and provide an adequate load to be powered via its 240V output connectors (like a lamp). Use the 12VDC/1A supply to power the Opta™ and make sure to correctly setup the RS-485 serial connection between the Opta™ and the 7M.
You can refer to the following diagram while connecting your Opta™ device to the Finder 7M energy meter via the RS-485 interface.

![Connecting Opta™ and Finder 7M](assets/connection.svg)

For the example code to work, you will need to set the 7M communication parameters to:

- Modbus address 2
- 19200 baud
- 8 data bits, even parity, 1 stop bit
  
This can easily be done using the Toolbox application via NFC.

### Code Overview

The goal of the following example is to read some values from the Finder 7M via Modbus and print them to the serial console for debug. Moreover the value of the E1 energy counter will be uploaded to the Arduino Cloud.

You can access the complete example code [here](assets/Opta_7M_Example.zip); after extracting the files the `Opta_7M_Example` sketch is available to try with your Opta™ device.

Note that the `thingProperties.h` file, generated by the Arduino Cloud during the dashboard configuration has been slightly modified to read the WiFi SSID and password from the `config.h` file:

```arduino
#define WIFI_SECRET_SSID      "YOUR SSID"
#define WIFI_SECRET_PASSWORD  "YOUR PASSWORD"

// Read from the 7M every 10 seconds
#define READ_INTERVAL_SECONDS 10

// Use WiFi to connect to Arduino Cloud
#define ARDUINO_CLOUD_USE_WIFI    1
```

### Reading from the 7M

The following headers are required to enable the Modbus RTU protocol, the connection with the Arduino Cloud, and to import thge mathematical function `pow()` that we'll need later.

The header file `finder-7m.h` contains all the needed definitions, like Modbus parameters and registers offsets.

```arduino
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

The leds on the Opta™ are flashed to show that we're doing the setup, then the RS 485 is configure with the `MODBUS_PRE_DELAY` and `MODBUS_POST_DELAY` values, needed for a proper operation per Modbus RTU specification. The parameters are pre-determined in `finder-7m.h`, based on the message RTU framing specifications explained in depth in this [guide](https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf). The other parameters `MODBUS_BAUDRATE` and `MODBUS_SERIAL_PARAMETERS` are the default for Modbus RTU connections: 19200 baud, 8 data bits, even parity, and one stop bit.

The `loop()` function contains the code that actually reads some registers from ther 7M and then prints theiur values to the serial console for debug:

```arduino
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

- the energy counter E1 in its x1000 version (in 0.1 Wh increments)
- the total run time (s)
- the frequency of the input AC (Hz)
- the voltage of the input AC (V)
- the total instantaneous active power (W)

All the values are 32 bits wide, so we can use a single function to obtain the raw data for all of them:

```arduino
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

The `modbus_7m_read32()` function reads two consecutive 16-bits registers starting from offset `reg` from device with Modbus address `addr` and compose them into a single 32-bits value by shifting the first value right by 16 bits. `ModbusRTUClient.read()` always returns a 32-bit result, with the value `-1` (`0xFFFFFFFF`) indicating an error. If the code detects any problem, it tried again up to three times, before giving up and returining the same error value to signal the calling code in `loop()`.

Some values, like the total run time or the content of counter E1 are real 32 bits  values (indicated by the type codes `T2` and `T3` in the [7M Modbus communication protocol document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf)) can be used without further processing. Unfortunately other values like, for example, the input current frequency or its voltage use a special encoding and need to be decode before they can be used.

```arduino
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

The `convert_t5()` function can be used to covert any value using the `T5` encoding to the corresponding float value. As explained in the [7M Modbus communication protocol document](https://cdn.findernet.com/app/uploads/2021/09/20090052/Modbus-7M24-7M38_v2_30062021.pdf) `T5` means that the 32 bits are used as follows:

- the most significant 8 bits are a signed exponent between -128 and 127
- the least significant 24 bits are an unsigned number or "mantissa"

The code extracts the sign `s` and the unsigned exponent `e`, then depending on the sign determines the final, signed value of the exponent. The `pow()` function is used to elevate the mantissa `m` to the power of `e` and the result is returned as a float. 

### Sending data to the cloud

To send data read from the 7M to the cloud you will need to setup and Arduino Cloud account, register your Opta™ device, assign it to a thing and add a property for every variable you want to upload.

In this example we will send to the could a single value: the energy counter E1. After adding the a single `energy` property we can copy the code generated by the Cloud IDE into our sketch `thingProperties.h`:

```arduino
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

The constants `WIFI_SECRET_SSID`, `WIFI_SECRET_PASSWORD` and `ARDUINO_CLOUD_USE_WIFI` are defined in `config.h` and allow to configure the Opta™ for a WiFi or Ethernet network connection.

The `setup()` function will need some additional code to correctly setup the connection to the Arduino Cloud:
```
    initProperties();

    setDebugMessageLevel(2);
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    ArduinoCloud.printDebugInfo();
}
```

## Using the Finder7M library

While the goal of this tutorial was to show how to setup the Modbus connection between the Opta™ and the Finder 7M it is also possible to use the Finder7M library to simplify the code considerably. In this case, the setup code is much shorter because the library itself configres the RS485 for a 19200 bound connection with `8N1` parameters:
```
Finder7M f7m;

void setup() {
    Serial.begin(9600);
    
    f7m.init();

    // Arduino Cloud or other initialization code goes here.
}
```

The main loop, is equally shorter:
```
void loop() {
    Serial.println("** Reading 7M at address " + String(MODBUS_7M_ADDRESS));

    data = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ENERGY_COUNTER_XK_E1);
    Serial.println("   energy = " + (data != INVALID_DATA ? String(data) : String("read error")));

    Measure a = f7m.getMIDInActiveEnergy(MODBUS_7M_ADDRESS);
    Serial.println("   energy = " + String(a.toFloat()));
}
```

## Conclusion

This tutorial demonstrates how to use the Arduino ecosystem's `ArduinoRS485` and `ArduinoModbus` libraries, as well as the Arduino IDE, to implement the Modbus RTU protocol between an Opta™ device and the Finder 7M energy meter. Moreover it shows how it is possible to use the Finder7M library to easily read counters and other values from a 7M.


