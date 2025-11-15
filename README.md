# Matter Over Thread End Sleepy Device With BLE Commissioning

Implements a motion sensor with an ESP32 and a BMA400 IMU. This code puts together all the bits that seem to be required for a Matter over Thread End Sleepy Device (ESD) with BLE commissioning.

## Lackluster Performance, a Proof of Concept

The ESP32 is evidently not a good device for implementing low power Matter accessories because Matter on ESP32 only works with light sleep which still consumes a lot of power.
This project is a Matter analogue of [Mailbox Sentry](https://github.com/retsyx/MailboxSentry). While the Mailbox Sentry sensor device can operate more than 6 months with a 1300mAh LiPo battery, this Matter implementation barely operates for a few days. So this is just a proof of concept.

## Requirements

* An Espressif ESP32 device with a Thread and BLE radio (for example, the ESP32-C6). If using a battery, the device should ideally have a charging circuit to allow charging the LiPo that will power the device. [The SparkFun Qwiic Pocket Development Board](https://www.sparkfun.com/products/22925) has an onboard charger, and is easy to interface with a suitable accelerometer.
* Bosch BMA400 accelerometer. The [SparkFun BMA400 Qwicc board](https://www.sparkfun.com/products/21207) interfaces easily with the SparkFun Qwiic Pocket.
* LiPo Battery. Because of the high power consumption, an unreasonably large battery is required. If using the linked SparkFun board, use a battery with a JST PH 2.0mm connector. Ensure that the polarity of the battery leads match the polarity of the board connector. The SparkFun Qwicc Pocket's connector polarity is the opposite of most hobby LiPos sold. Battery connector pins can be easily pried out with a utility knife to reverse their position in the connector.

## Building

* Install [esp-idf](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html) and [esp-matter](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/)
* Set the target. For example, esp32-c6 if using the recommended board.
* Build the project, and flash to device

## Commissioning

With your favorite Matter commissioner (HomeKit, etc.) add the accessory.

This project uses develpoment commissioning credentials, so the QR code is always the one below.

![matter provisioning QR code](matter_qr.png)




