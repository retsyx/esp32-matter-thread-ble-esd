# XIAO ESP32-C6 / BMA400 Board

**This board has not been manufactured or tested.**

It is a simple board designed in KiCad.

## Manufacturing

It is a standard 2-layer, 1.6mm FR4 board. Dimensions are 40mm x 32mm.

Manufacturing files are available [here](mfr/).

An example BOM is available [here](mfr/bom.csv).

## Building the Firmware

From the repository root, set the ESP-IDF target to `esp32c6`, then enable **Example Configuration → Use the XIAO ESP32-C6 carrier board** in `idf.py menuconfig` (`CONFIG_BOARD_XIAO_ESP32C6=y`).



