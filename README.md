# XPLR-IOT-1-location-example

This project is intended to be used with XPLR-IOT-1 from u-blox to get a location using both CellLocate and CloudLocate technologies. This repository contains two examples containing source codes for each approach to get a location using XPLR-IOT-1. Any future updates of this application will be uploaded here. 

The main function of this project is to calculate location based on cellular, Wi-Fi or GNSS information, whichever is required by the user. The user is required to set some configurable parameters in the example as per the requirements and the example will bring back the location based on those parameters. 

The two example projects are configured to build assuming that XPLR-IOT-1 comes with a Bootloader that allows firmware on the device to be updated without the need of a debugger/programmer. One of the major aspects of the firmware is that it uses ubxlib library as much as possible to implement its functionality.

It is to specify that this is an example project and any example code will not be a part of the end product.

More information about XPLR-IOT-1 hardware etc. can be found in the following link: [XPLR-IOT-1 quick start](https://developer.thingstream.io/guides/iot-communication-as-a-service/hardware/xplr-iot-1-quick-start-guide)

All examples requires to setup the cellular connection usign the SARA-R5 cellular module in the XPLR-IOT-1 kit. Although the sample code includes the connection setup, you might experience a long connection time the very first time that the connection is set up or after a SARA R5 firmware upgrade, and this can result in a failure of location example. It is  suggested to read the section "SARA-R5 connection troubleshooting guide" of the [Module connection set up guide](https://developer.thingstream.io/guides/location-services/celllocate-getting-started/module-connection-setup) to optimize the connection set up. 

Additional repositories associated with XPLR-IOT-1 are available for firmware flash, application firmware, Node-RED dashboard and the hardware design:

- Firmware flash on XPLR-IOT-1
https://github.com/u-blox/XPLR-IOT-1

- Firmware running on NORA-B106:
https://github.com/u-blox/XPLR-IOT-1-software

- Node-RED dashboard:
https://github.com/u-blox/XPLR-IOT-1-Node-RED-dashboard

- Hardware design:
https://github.com/u-blox/XPLR-IOT-1-hardware


## Disclaimer
Copyright © u-blox

u-blox reserves all rights in this deliverable (documentation, software, etc., hereafter “Deliverable”).

u-blox grants you the right to use, copy, modify and distribute the Deliverable provided hereunder for any purpose without fee.

THIS DELIVERABLE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS DELIVERABLE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.

In case you provide us a feedback or make a contribution in the form of a further development of the Deliverable (“Contribution”), u-blox will have the same rights as granted to you, namely to use, copy, modify and distribute the Contribution provided to us for any purpose without fee.
