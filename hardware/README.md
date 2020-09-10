This directory contains details of the hardware I used to implement my own BMS.  The schematic is pretty simply and mostly uses components that I happened to have hanging around.

The RS485 module U3 is little board based on the MAX485 chip, which can be found very easily and cheaply [on ebay](https://www.ebay.co.uk/sch/i.html?_nkw=MAX485+RS485+module). The MAX485 needs 5V to power it but is fine with 3.3V logic input. If using a 3.3V MCU then check that its inputs are 5V tolerant, and if not then run the RO->RX through a potential divider.

The voltage regulator module U2 is needed to convert the battery voltage down to 5V to run the MCU and RS485 boards. It may be possible to find a MCU board that already has a suitable voltage regulator onboard.

The PNP transistor output stage for EC and EL is designed to provide a signal voltage only and will not provide much current (although it is short-circuit proof). If more current is required, or if it needs to hold the signal more strongly to 0V when off, then changes in the resistor values would need to be made. If it needs to provide any significant current then a P-Channel MOSFET would be a much better choice instead of the PNP transistor.
```
datasheets                            - Datasheets of component boards used
with-EL-switch                        - Alternative version that supports an external switch to manually disconnect load
Arduino-XP-BMS-schematic.dsn          - Schematic (TinyCad)
Arduino-XP-BMS-schematic.net          - Netlist (Protel)
Arduino-XP-BMS-schematic.pdf          - Schematic (PDF)
Arduino-XP-BMS-stripboard-layout.pdf  - Stripboard layout (PDF)
Arduino-XP-BMS-stripboard-layout.per  - Stripboard layout (VeeCad)
Final-1.jpg                           - Final hardware build
Final-2.jpg                           - Final hardware build
Prototype.jpg                         - Initial breadboard prototype
```