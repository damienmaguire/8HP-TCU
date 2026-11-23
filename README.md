# 8HP-TCU
OpenSource controller for the ZF 8HP series of automatic gearboxes based on the OpenInverter software system
https://openinverter.org/forum/

<img width="1403" height="929" alt="tcuv4" src="https://github.com/user-attachments/assets/2486fc5e-2951-46a5-b27e-60c1235bf647" />


Although the design is intended for use with the BMW ZF GA8P75HZ gearbox for EV conversions, it should work just as well with traditional 8HP gearboxes with internal combustion engine power.

Design files in Kicad 10 format.

Enclosure, header, plug and pins : https://www.aliexpress.com/item/32839361268.html

Prewired plug (recommended) : https://www.aliexpress.com/item/1005004818455812.html

Work is in progress as of 01/25.

![TCU_Adapter_Board](https://github.com/user-attachments/assets/58b4c9c0-83e0-4c65-beef-87e31b3bbbc8)

## Firmware

STM32F107 firmware (openinverter / ZombieVerter based) is in
[Software/firmware](Software/firmware/README.md): build steps,
architecture, parameters and open questions.
